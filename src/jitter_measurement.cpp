//! robotkernel module jitter_measurement
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of module_jitter_measurement.
 *
 * module_jitter_measurement is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_jitter_measurement is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_jitter_measurement; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "jitter_measurement.h"
#include "robotkernel/helpers.h"
#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <semaphore.h>
#include <signal.h>
#include <math.h>
#include <fcntl.h>
#include <inttypes.h>

#include "yaml-cpp/yaml.h"

MODULE_DEF(module_jitter_measurement, module_jitter_measurement::jitter_measurement)

using namespace std;
using namespace robotkernel;
using namespace module_jitter_measurement;
        
//! yaml config construction
/*!
 * \param name name of jm
 * \param node yaml node
 */
jitter_measurement::jitter_measurement(const char* name, const YAML::Node& node) :
    runnable(0, 0, name), module_base("module_jitter_measurement", name, node)
{
    buffer_size           = get_as<unsigned int>(node, "buffer_size", 1000);
    buffer_pos            = 0;
    buffer_act            = 0;
    do_print              = false;
    threaded              = get_as<bool>(node, "threaded", true);
    new_maxever_threshold = get_as<double>(node, "new_maxever_threshold", 0.001);
    dump_to_file          = get_as<string>(node, "dump_to_file", "");

    dump_fd = 0;

    // resize buffers
    buffer[0].resize(buffer_size);
    buffer[1].resize(buffer_size);
    log_diff.resize(buffer_size);
}

//! default destruction
jitter_measurement::~jitter_measurement() {
    if (dump_fd > 0) {
        close(dump_fd);
    }
}

//! additional init function
void jitter_measurement::init() {
    if (dump_to_file != "") {
        dump_fd = open(dump_to_file.c_str(), O_CREAT | O_WRONLY, 0660);
    }
}

void jitter_measurement::tick() {
    std::unique_lock<std::mutex> lock(state_mtx);

    if (state < module_state_op) {
        return;
    }

    // get actual timestamp
    auto now = std::chrono::high_resolution_clock::now();

    local_pdin.last_ts = std::chrono::duration_cast<
        std::chrono::nanoseconds>(now.time_since_epoch()).count();

    pdin->write(pdin_provider, 0, (uint8_t *)&local_pdin, sizeof(local_pdin));
    pdin->trigger();

    buffer[buffer_act][buffer_pos++] = now;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        buffer_act = (buffer_act + 1) % 2; 

        if (threaded) {
            std::unique_lock<std::mutex> lock(sync_mtx);
            do_print = true;
            sync_cond.notify_one();
        } else {
            print();
        }
    }
}

//! print last buffer measurement values
void jitter_measurement::print() {
    log_tp_vec_t& act_buf = buffer[(buffer_act + 1) % 2];

    double dev;
    double cycle = 0, avgjit = 0, maxjit = 0;
    
    struct jitter_pdout *local_pdout = (struct jitter_pdout *)pdout->pop(pdout_consumer);

    // calculate differences and sum of differences
    for (unsigned i = 0; i < buffer_size - 1; ++i) {
        log_diff[i+1] = act_buf[i+1] - act_buf[i];
        cycle += std::chrono::duration<double>(log_diff[i+1]).count();
    }
    cycle /= buffer_size - 1;

    // calculating maximum deviation
    bool have_new_maxever = false;
    log_tp_t new_maxever_time;

    for (unsigned i = 1; i < buffer_size; i++) {
        dev     = fabs(std::chrono::duration<double>(log_diff[i]).count() - cycle); 
        maxjit  = max(dev, maxjit);

        if (maxjit > local_pdin.maxever) {
            // new max ever!
            local_pdin.maxever = maxjit;
            new_maxever_time = act_buf[i - 1];
            have_new_maxever = true;
        
            if (local_pdin.maxever > new_maxever_threshold)
                maxever_t_dev->do_trigger();
        }
        avgjit += (dev * dev);
    }

    avgjit = sqrt(avgjit/(buffer_size - 1));

    if (dump_fd > 0) {
        static char dump_buf[512];
        for (unsigned i = 0; i < buffer_size; i++) {
            uint64_t act_buf_val = chrono::duration_cast<chrono::nanoseconds>(act_buf[i].time_since_epoch()).count();
            uint64_t log_diff_val = log_diff[i].count();
            snprintf(dump_buf, 512, "%" PRIu64 "\t%" PRIu64 "\n", act_buf_val, log_diff_val);
            write(dump_fd, dump_buf, strlen(dump_buf));
            fsync(dump_fd);
        }
    }

    local_pdin.last_cycle = cycle;
    local_pdin.last_max = maxjit;

    if (have_new_maxever) {
        maxever_time = new_maxever_time;
        auto now = std::chrono::high_resolution_clock::now();
        double seconds_ago = std::chrono::duration<double>(now - maxever_time).count();
        log(info, "new max ever is %.3fms ago\n", seconds_ago);

        std::time_t t = std::chrono::high_resolution_clock::to_time_t(maxever_time);
        maxever_time_string = string_printf("%s", strtok(std::ctime(&t), "\n"));
    }

    if (local_pdout->max_ever_clamp != 0 && local_pdin.maxever > local_pdout->max_ever_clamp)
        local_pdin.maxever = local_pdout->max_ever_clamp;

    string running_maxever_time_string;
    auto now = std::chrono::high_resolution_clock::now();
    double seconds_ago = std::chrono::duration<double>(now - maxever_time).count();

    if (maxever_time_string == "") {
        running_maxever_time_string = string_printf("(%.1fs ago)", seconds_ago);
    } else {
        running_maxever_time_string = string_printf("(%s, %.1fs ago)", maxever_time_string.c_str(), seconds_ago);
    }

    log(info, "mean period: %4.0fus, jitter mean:"
            " %4.0fus, max %4.0fus, max ever %4.0fus %s\n",
            cycle*1E6, avgjit*1E6, maxjit*1E6, local_pdin.maxever*1E6, running_maxever_time_string.c_str());
}

//! handler function called if thread is running
void jitter_measurement::run() {

    log(info, "starting jitter thread\n");

    while (running()) {
        std::unique_lock<std::mutex> lock(sync_mtx);
        int ret = sync_cond.wait_for(lock, std::chrono::seconds(1), [&]{return do_print;});
        lock.unlock();

        if (ret) {
            do_print = false;
            print();
        }
    }

    log(info, "stopping jitter thread\n");
}

//! svc_reset_max_ever
/*!
 * \param[in]   req     Service request data.
 * \param[out]  resp    Service response data.
 */
void jitter_measurement::svc_reset_max_ever(
        const struct svc_req_reset_max_ever& req, 
        struct svc_resp_reset_max_ever& resp) 
{
    resp.maxever = local_pdin.maxever;
    local_pdin.maxever = 0;
    local_pdin.maxever_time = 0;
    maxever_time_string[0] = 0;
}

//! State transition from PREOP to SAFEOP
void jitter_measurement::set_state_op_2_safeop() {
    // ====> stop sending commands
    robotkernel::remove_device(pdout_inspect);
    pdout_inspect = nullptr;

    robotkernel::remove_device(pdout);
    pdout->reset_consumer(pdout_consumer);
    pdout_consumer = nullptr;
    pdout = nullptr;
}

//! State transition from PREOP to SAFEOP
void jitter_measurement::set_state_safeop_2_preop() {
    // ====> stop receiving measurements
    stop();

    do {
        // wait until thread has stopped
    } while (running());

    robotkernel::remove_device(maxever_t_dev);      // trigger device new maxever

    robotkernel::remove_device(pdin_inspect);
    pdin_inspect = nullptr;

    robotkernel::remove_device(pdin);               // pd inputs

    // register services
    remove_svc_reset_max_ever();

    pdin->reset_provider(pdin_provider);
    pdin_provider = nullptr;
    pdin = nullptr;

    // destroy trigger device
    maxever_t_dev = nullptr;
}

//! State transition from PREOP to SAFEOP
void jitter_measurement::set_state_preop_2_safeop() {
    // create named process data for inputs
    pdin = make_shared<robotkernel::triple_buffer>(sizeof(struct jitter_pdin), 
            name, string("inputs"), string(
                "- double: max_ever\n"
                "- double: last_max\n"
                "- double: last_cycle\n"
                "- uint64_t: last_ts\n"
                "- double: max_ever_time\n"));
    pdin_provider = make_shared<pd_provider>(name);
    pdin->set_provider(pdin_provider);

    maxever_time_string[0] = 0;
    local_pdin.maxever      = 0.;
    local_pdin.last_max     = 0.;
    local_pdin.last_cycle   = 0.;
    local_pdin.last_ts      = 0;
    local_pdin.maxever_time = 0.;

    // create trigger device for pdin
    maxever_t_dev = make_shared<trigger>(name, "new_maxever");

    // register services
    add_svc_reset_max_ever(name, "reset_max_ever");

    // ====> start receiving measurements
    if (threaded) {
        start();
    }

    // add process data inspection and process data
    robotkernel::add_device(pdin);                 // pd inputs
    robotkernel::add_device(maxever_t_dev);        // trigger device new maxever

    pdin_inspect = make_shared<service_provider_process_data_inspection::pd_inspection>(name, "inputs", pdin);
    robotkernel::add_device(pdin_inspect);
}

//! State transition from PREOP to SAFEOP
void jitter_measurement::set_state_safeop_2_op() {
    // ====> start sending commands
    // create named process data for outputs 
    pdout = make_shared<robotkernel::triple_buffer>(sizeof(struct jitter_pdout), 
            name, string("outputs"), string("- double: max_ever_clamp\n"));
    pdout_consumer = make_shared<pd_consumer>(name);
    pdout->set_consumer(pdout_consumer);
    robotkernel::add_device(pdout);

    pdout_inspect = make_shared<service_provider_process_data_inspection::pd_inspection>(name, "outputs", pdout);
    robotkernel::add_device(pdout_inspect);
}


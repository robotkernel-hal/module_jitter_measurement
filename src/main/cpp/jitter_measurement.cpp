//! robotkernel module jitter_measurement
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
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

#include "yaml-cpp/yaml.h"
#include <string_util/string_util.h>

MODULE_DEF(module_jitter_measurement, module_jitter_measurement::jitter_measurement)

using namespace std;
using namespace std::placeholders;
using namespace robotkernel;
using namespace module_jitter_measurement;
using namespace string_util;
        
//! yaml config construction
/*!
 * \param name name of jm
 * \param node yaml node
 */
jitter_measurement::jitter_measurement(const char* name, const YAML::Node& node) 
: runnable(0, 0, name), module_base("module_jitter_measurement", name, node), 
  service_provider::process_data_inspection::base(name, "jitter") {
    buffer_size           = get_as<unsigned int>(node, "buffer_size", 1000);
    buffer_pos            = 0;
    buffer_act            = 0;
    threaded              = get_as<bool>(node, "threaded", true);
    new_maxever_threshold = get_as<double>(node, "new_maxever_threshold", 0.001);

    // resize buffers
    buffer[0].resize(buffer_size);
    buffer[1].resize(buffer_size);
    log_diff.resize(buffer_size);

    // create named process data for inputs
    string pdin_desc = 
        "- double: max_ever\n"
        "- double: last_max\n"
        "- double: last_cycle\n"
        "- double: last_ts\n"
        "- double: max_ever_time\n";

    pdin = make_shared<robotkernel::triple_buffer>(
            sizeof(struct jitter_pdin), name, string("inputs"), pdin_desc,
            format_string("%s.inputs.trigger", name));

    // create named process data for outputs 
    string pdout_desc = 
        "- double: max_ever_clamp\n";

    pdout = make_shared<robotkernel::triple_buffer>(
            sizeof(struct jitter_pdout), name, string("outputs"), pdout_desc,
            format_string("%s.outputs.trigger", name));

    // set state to init
    state = module_state_init;

    maxever_time_string[0] = 0;
    local_pdin.maxever      = 0.;
    local_pdin.last_max     = 0.;
    local_pdin.last_cycle   = 0.;
    local_pdin.last_ts      = 0.;
    local_pdin.maxever_time = 0.;

    kernel& k = *kernel::get_instance();    
    
    // create trigger device for pdin
    pdin_t_dev    = make_shared<trigger>(name, "inputs");
    maxever_t_dev = make_shared<trigger>(name, "new_maxever");

    // register services
    k.add_service(name, "reset_max_ever",
            service_definition_reset_max_ever,
            std::bind(&jitter_measurement::service_reset_max_ever, this, _1, _2));
}

//! default destruction
jitter_measurement::~jitter_measurement() {
    // destroy trigger device
    pdin_t_dev = nullptr;
    maxever_t_dev = nullptr;
}

void jitter_measurement::tick() {
    // get actual timestamp
    auto now = std::chrono::high_resolution_clock::now();

    local_pdin.last_ts = std::chrono::duration_cast<
        std::chrono::nanoseconds>(now.time_since_epoch()).count();

    pdin->write(0, (uint8_t *)&local_pdin, sizeof(local_pdin));
    pdin_t_dev->trigger_modules();

    buffer[buffer_act][buffer_pos++] = now;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        buffer_act = (buffer_act + 1) % 2; 

        if (threaded)
            sync_cond.notify_one();
        else
            print();
    }
}

//! print last buffer measurement values
void jitter_measurement::print() {
    log_tp_vec_t& act_buf = buffer[(buffer_act + 1) % 2];

    double dev;
    double cycle = 0, avgjit = 0, maxjit = 0;
    
    struct jitter_pdout *local_pdout = (struct jitter_pdout *)pdout->pop();

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
                maxever_t_dev->trigger_modules();
        }
        avgjit += (dev * dev);
    }
    avgjit = sqrt(avgjit/(buffer_size - 1));

    local_pdin.last_cycle = cycle;
    local_pdin.last_max = maxjit;

    if (have_new_maxever) {
        maxever_time = new_maxever_time;
        auto now = std::chrono::high_resolution_clock::now();
        double seconds_ago = std::chrono::duration<double>(now - maxever_time).count();
        log(info, "new max ever is %.3fms ago\n", seconds_ago);

        std::time_t t = std::chrono::high_resolution_clock::to_time_t(maxever_time);
        int wr = snprintf(maxever_time_string, MAXEVER_TIME_STRING_SIZE, "%s", std::ctime(&t));
        maxever_time_string[wr-1] = '\0';
    }

    if (local_pdout->max_ever_clamp != 0 && local_pdin.maxever > local_pdout->max_ever_clamp)
        local_pdin.maxever = local_pdout->max_ever_clamp;

    char running_maxever_time_string[MAXEVER_TIME_STRING_SIZE];
    auto now = std::chrono::high_resolution_clock::now();
    double seconds_ago = std::chrono::duration<double>(now - maxever_time).count();

    if (maxever_time_string[0] == 0)
        snprintf(running_maxever_time_string, MAXEVER_TIME_STRING_SIZE, 
                "(%.1fs ago)", seconds_ago);
    else
        snprintf(running_maxever_time_string, MAXEVER_TIME_STRING_SIZE, 
                "(%s, %.1fs ago)", maxever_time_string, seconds_ago);

    log(info, "mean period: %4.0fus, jitter mean:"
            " %4.0fus, max %4.0fus, max ever %4.0fus %s\n",
            cycle*1E6, avgjit*1E6, maxjit*1E6, local_pdin.maxever*1E6, running_maxever_time_string);
}

//! handler function called if thread is running
void jitter_measurement::run() {
    std::unique_lock<std::mutex> lock(sync_mtx);

    while (running()) {
        if (sync_cond.wait_for(lock, std::chrono::seconds(1))
                == std::cv_status::timeout)
            continue;

        print();
    }
}

//! reset max ever
/*!
 * \param request service request data
 * \param response service response data
 * \return success
 */
int jitter_measurement::service_reset_max_ever(
        const robotkernel::service_arglist_t& request,
        robotkernel::service_arglist_t& response) {
#define RESET_MAX_EVER_RESP_MAXEVER 0
    response.resize(1);
    response[RESET_MAX_EVER_RESP_MAXEVER] = local_pdin.maxever;

    local_pdin.maxever = 0;
    local_pdin.maxever_time = 0;
    maxever_time_string[0] = 0;

    return 0;
}

const std::string jitter_measurement::service_definition_reset_max_ever = 
"response:\n"
"- double: maxever\n";

int jitter_measurement::set_state(module_state_t state) {
    kernel& k = *kernel::get_instance();

    // get transition
    uint32_t transition = GEN_STATE(this->state, state);
    
    log(info, "state %s requested\n", state_to_string(state));

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            k.remove_device(pdout);

            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            stop();

            k.remove_device(maxever_t_dev);      // trigger device new maxever
            k.remove_device(pdin_t_dev);         // trigger device pd inputs
            k.remove_device(pdin);               // pd inputs
            k.remove_device(shared_from_this()); // process data inspection

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
        case init_2_init:
            // ====> re-/open device
            if (state == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open device
            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
            // ====> initial devices            
            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop:
            // ====> start receiving measurements
            if (threaded)
                start();
            
            // add process data inspection and process data
            k.add_device(shared_from_this());   // process data inspection
            k.add_device(pdin);                 // pd inputs
            k.add_device(pdin_t_dev);           // trigger device pd inputs
            k.add_device(maxever_t_dev);        // trigger device new maxever

            if (state == module_state_safeop)
                break;
        case safeop_2_op:
            // ====> start sending commands
            k.add_device(pdout);
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}
        
//! return input process data (measurements)
/*!
 * \param pd return input process data
 */
void jitter_measurement::get_pdin(
        service_provider::process_data_inspection::pd_t& pd) {

    pd.resize(pdin->length);
    pdin->read(0, &pd[0], pdin->length, false);
}

//! return output process data (commands)
/*!
 * \param pd return output process data
 */
void jitter_measurement::get_pdout(
        service_provider::process_data_inspection::pd_t& pd) {

    pd.resize(pdout->length);
    pdout->read(0, &pd[0], pdout->length, false);
}


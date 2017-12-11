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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <semaphore.h>
#include <signal.h>
#include <math.h>

#ifdef __VXWORKS__
#include <vxWorks.h>
#include <sysLib.h>
#include <taskLib.h>
#endif

#include "yaml-cpp/yaml.h"
#include <string_util/string_util.h>

MODULE_DEF(module_jitter_measurement, module_jitter_measurement::jitter_measurement)

using namespace std;
using namespace std::placeholders;
using namespace robotkernel;
using namespace module_jitter_measurement;
using namespace string_util;
        
static const std::string pd_definition_pdin = 
"- uint64_t: maxever\n"
"- uint64_t: last_max\n"
"- uint64_t: last_cycle\n"
"- uint64_t: last_ts\n"
"- double: maxever_time\n";

static const std::string pd_definition_pdout = 
"- uint64_t: maxever\n"
"- uint64_t: last_max\n"
"- uint64_t: last_cycle\n"
"- uint64_t: last_ts\n"
"- double: maxever_time\n";

#if defined ARCH_INTEL
uint64_t jm_rdtsc(void) {
    unsigned a, d;
    __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
    return ((uint64_t)a) | (((uint64_t)d) << 32);
}
#else
uint64_t jm_rdtsc(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)(ts.tv_sec * 1e9 + ts.tv_nsec);
}
#endif

//! yaml config construction
/*!
 * \param name name of jm
 * \param node yaml node
 */
jitter_measurement::jitter_measurement(const char* name, const YAML::Node& node) 
: runnable(0, 0, name), module_base("module_jitter_measurement", name, node), 
  service_provider::process_data_inspection::base(name, "jitter") {
    buffer_size     = get_as<unsigned int>(node, "buffer_size");
    buffer_pos      = 0;
    buffer_act      = 0;
    cps             = get_as<uint64_t>(node, "cps", 1);
    threaded        = get_as<bool>(node, "threaded", true);

    // resize buffers
    buffer[0].resize(buffer_size);
    buffer[1].resize(buffer_size);
    log_diff.resize(buffer_size);

    new_maxever_command = 
        get_as<string>(node, "new_maxever_command", string(""));
    new_maxever_command_threshold = 
        get_as<int>(node, "new_maxever_command_threshold", 50);

    // create named process data for inputs
    string pdin_desc = 
        "uint64_t: max_ever\n"
        "uint64_t: last_max\n"
        "uint64_t: last_cycle\n"
        "uint64_t: last_ts\n"
        "double: max_ever_time\n";

    pdin = make_shared<robotkernel::triple_buffer>(
            sizeof(struct jitter_pdin), name, string("inputs"), pdin_desc);
            
    // create named process data for outputs 
    string pdout_desc = 
        "uint64_t: max_ever_clamp\n";

    pdout = make_shared<robotkernel::triple_buffer>(
            sizeof(struct jitter_pdout), name, string("outputs"), pdout_desc);

    // set state to init
    state = module_state_init;

    is_printing = false;
    maxever_time_string[0] = 0;

    pulse_on_trigger = NO_PULSE;
    pulse_on_new_max_ever = NO_PULSE;
#ifdef HAVE_TERMIOS_H
    if (node["tty_control_signals"]) {
        const YAML::Node& tty_node = node["tty_control_signals"];
        string port = get_as<string>(tty_node, "port", "/dev/ttyS0");
        tty_port = new tty_control_signals(port.c_str());
        string on_trigger = get_as<string>(tty_node, "pulse_on_trigger", "");
        set_pulse_from_string(&pulse_on_trigger, on_trigger);
        string on_new_me = get_as<string>(tty_node, "pulse_on_new_max_ever", "");
        set_pulse_from_string(&pulse_on_new_max_ever, on_new_me);
    } else
        tty_port = NULL;
#else
    if (node["tty_control_signals"]) 
        log(error, "tty_control_signals not supported on this architecture!\n");
#endif

    kernel& k = *kernel::get_instance();    
    
    // create trigger device for pdin
    pdin_t_dev = make_shared<trigger>(name, "inputs");

    // register services
    k.add_service(name, "reset_max_ever",
            service_definition_reset_max_ever,
            std::bind(&jitter_measurement::service_reset_max_ever, this, _1, _2));
    k.add_service(name, "get_cps",
            service_definition_get_cps,
            std::bind(&jitter_measurement::service_get_cps, this, _1, _2));

    calibrate();
}

//! default destruction
jitter_measurement::~jitter_measurement() {
#ifdef HAVE_TERMIOS_H
    if(tty_port)
        delete tty_port;
#endif    
    // destroy trigger device
    pdin_t_dev = nullptr;
}

void jitter_measurement::set_pulse_from_string(pulse_signals_t* which, std::string which_str) {
    if(which_str == "rts")
        *which = PULSE_RTS;
    else if(which_str == "rts_neg")
        *which = PULSE_RTS_NEG;
    else if(which_str == "dtr")
        *which = PULSE_DTR;
    else if(which_str == "dtr_neg")
        *which = PULSE_DTR_NEG;
}
void jitter_measurement::do_pulse(pulse_signals_t which) {
#ifdef HAVE_TERMIOS_H
    switch(which) {
        case PULSE_RTS:
            tty_port->pulse_rts();
            break;
        case PULSE_RTS_NEG:
            tty_port->pulse_rts_neg();
            break;
        case PULSE_DTR:
            tty_port->pulse_dtr();
            break;
        case PULSE_DTR_NEG:
            tty_port->pulse_dtr_neg();
            break;
        case NO_PULSE:
            break;
    }
#endif
}

//! calibrate function for clocks per second
void jitter_measurement::calibrate() {
    if (cps != 1)
        return;

    memset(&local_pdin, 0, sizeof(local_pdin));

    log(info, "calibrating clocks/sec...\n");

    struct timespec ts_time;
    clock_gettime(CLOCK_REALTIME, &ts_time);
    double begin_time = (double)ts_time.tv_sec + (ts_time.tv_nsec / 1E9);
    uint64_t begin = jm_rdtsc();

    struct timespec ts = { 0, 10000000 };
    nanosleep(&ts, NULL);
    
    clock_gettime(CLOCK_REALTIME, &ts_time);
    double end_time = (double)ts_time.tv_sec + (ts_time.tv_nsec / 1E9);
    uint64_t end = jm_rdtsc();
    log(info, "got %17.13f sec diff\n", (end_time - begin_time));
    cps = (end - begin) / (end_time - begin_time); // magic factor to correct cps
    log(info, "got %llu clock/sec\n", cps);
}

//! does one measurement
/*!
 * if log buffer is full, output thread is triggered
 */

inline double get_seconds() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double)ts.tv_sec + 1e-9 * ts.tv_nsec;
}

inline uint64_t get_timestamp() {
    return jm_rdtsc();
}

void jitter_measurement::tick() {
    do_pulse(pulse_on_trigger);

    // get actual timestamp
    uint64_t ts = get_timestamp();

    local_pdin.last_ts = ts;
    pdin->write(0, (uint8_t *)&local_pdin, sizeof(local_pdin));
    pdin_t_dev->trigger_modules();

    buffer[buffer_act][buffer_pos++] = ts;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        buffer_act = (buffer_act + 1) % 2; 

        if (threaded)
            sync_cond.notify_one();
        else
            print();
    }
    //trigger_modules();
}

//! returns last measuremente
double jitter_measurement::last_measurement() { 
    uint64_t tick_act, tick_last;
    int act_pos = buffer_pos;
    int act_buf = buffer_act;

    // get actual tick
    if (act_pos == 0) {
        act_pos = buffer_size - 1;
        act_buf = (act_buf + 1) % 2;
    }
    tick_act = buffer[act_buf][act_pos--];

    // get last tick
    if (act_pos == 0) {
        act_pos = buffer_size - 1;
        act_buf = (act_buf + 1) % 2;
    }
    tick_last = buffer[act_buf][act_pos];

    return (double)(tick_act - tick_last)/cps;
}

//! print last buffer measurement values
void jitter_measurement::print() {
    unsigned int i;
    log_vec_t& act_buf = buffer[(buffer_act + 1) % 2];

    uint64_t dev;
    uint64_t cycle = 0, avgjit = 0, maxjit = 0;
    
    struct jitter_pdout *local_pdout = (struct jitter_pdout *)pdout->pop();

    // calculate differences and sum of differences
    std::adjacent_difference(act_buf.begin(), act_buf.end(), log_diff.begin());
    cycle = std::accumulate(log_diff.begin() + 1, log_diff.end(), 0);
    cycle /= buffer_size - 1;

    // calculating maximum deviation
    double fac = (1E6 / (double)cps);
    uint64_t new_maxever_time = 0;
    for (int i = 1; i < buffer_size; i++) {
        dev     = labs(log_diff[i] - cycle); 
        maxjit  = max(dev, maxjit);
        uint64_t this_maxjit = (uint64_t)(maxjit * fac);

        if (this_maxjit > local_pdin.maxever) {
            // new max ever!
            local_pdin.maxever = this_maxjit;
            new_maxever_time = act_buf[i - 1];
        }
        avgjit += (dev * dev);
    }
    avgjit /= (buffer_size - 1);

    avgjit = (uint64_t)(sqrt((double)avgjit) * fac);
    cycle  = (uint64_t)(cycle  * fac);
    maxjit = (uint64_t)(maxjit * fac);
    local_pdin.last_cycle = cycle;
    local_pdin.last_max = maxjit;

    if (new_maxever_time) {
        do_pulse(pulse_on_new_max_ever);
        double seconds_ago = (get_timestamp() - new_maxever_time) / (double)cps;
        log(info, "new max ever is %.3fms ago\n", seconds_ago);
        local_pdin.maxever_time = get_seconds() - seconds_ago;
        time_t int_ts = (int)local_pdin.maxever_time;
        struct tm* btime = localtime(&int_ts);
        strftime(maxever_time_string, 64, " (at %H:%M:%S", btime);
        unsigned int part_second = (unsigned int)(10000 * (local_pdin.maxever_time - (int)local_pdin.maxever_time));
        if(part_second >= 10000)
            part_second = 9999;
        snprintf(maxever_time_string + strlen(maxever_time_string), 32, ".%04d", part_second);
    }

    if (local_pdout->max_ever_clamp != 0 && local_pdin.maxever > local_pdout->max_ever_clamp)
        local_pdin.maxever = local_pdout->max_ever_clamp;

    char running_maxever_time_string[128];
    if(maxever_time_string[0] == 0)
        running_maxever_time_string[0] = 0;
    else
        snprintf(running_maxever_time_string, 128, "%s, %.1fs ago)",
                maxever_time_string,
                get_seconds() - local_pdin.maxever_time);

    log(info, "mean period: %4lluus, jitter mean:"
            " %2lluus, max %4lluus, max ever %4lluus%s\n",
            cycle, avgjit, maxjit, local_pdin.maxever, running_maxever_time_string);

    if(new_maxever_time && new_maxever_command.size() && local_pdin.maxever > new_maxever_command_threshold) {
        string cmd = format_string("%s %llu %s",
                new_maxever_command.c_str(), local_pdin.maxever, maxever_time_string + 5);
        log(info, "execute new_maxever_command: %s\n", cmd.c_str());
        system(cmd.c_str());
    }
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
"- uint64_t: maxever\n";

//! reset max ever
/*!
 * \param request service request data
 * \param response service response data
 * \return success
 */
int jitter_measurement::service_get_cps(
        const robotkernel::service_arglist_t& request,
        robotkernel::service_arglist_t& response) {
#define GET_CPS_RESP_CPS 0
    response[GET_CPS_RESP_CPS] = cps;

    return 0;
}

const std::string jitter_measurement::service_definition_get_cps =
"response:\n"
"- uint32_t: cps\n";

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


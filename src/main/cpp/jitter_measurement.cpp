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
#include <pthread.h>
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
"uint64_t maxever\n"
"uint64_t last_max\n"
"uint64_t last_cycle\n"
"uint64_t last_ts\n"
"double maxever_time\n";

static const std::string pd_definition_pdout = 
"uint64_t maxever\n"
"uint64_t last_max\n"
"uint64_t last_cycle\n"
"uint64_t last_ts\n"
"double maxever_time\n";

#if !defined(HAVE___RDTSC)
uint64_t __rdtsc(void) {
    unsigned a, d;
    __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
    return ((uint64_t)a) | (((uint64_t)d) << 32);
}
#endif

//! yaml config construction
/*!
 * \param name name of jm
 * \param node yaml node
 */
jitter_measurement::jitter_measurement(const char* name, const YAML::Node& node) 
: runnable(0, 0), module_base("module_jitter_measurement", name, node), 
  service_provider::process_data_inspection::base(name, "jitter") {
    buffer_size     = get_as<unsigned int>(node, "buffer_size");
    buffer[0]       = new uint64_t[buffer_size];
    buffer[1]       = new uint64_t[buffer_size];
    log_diff        = new uint64_t[buffer_size];
    buffer_pos      = 0;
    buffer_act      = 0;
    cps             = get_as<uint64_t>(node, "cps", 1);
    threaded        = get_as<bool>(node, "threaded", true);

    new_maxever_command = 
        get_as<string>(node, "new_maxever_command", string(""));
    new_maxever_command_threshold = 
        get_as<int>(node, "new_maxever_command_threshold", 50);

    memset(buffer[0], 0, sizeof(uint64_t) * buffer_size);
    memset(buffer[1], 0, sizeof(uint64_t) * buffer_size);
    memset(log_diff, 0, sizeof(uint64_t) * buffer_size);

    // create named process data for inputs
    string pdin_desc = 
        "uint64_t: max_ever\n"
        "uint64_t: last_max\n"
        "uint64_t: last_cycle\n"
        "uint64_t: last_ts\n"
        "double: max_ever_time\n";

    pdin = make_shared<robotkernel::process_data_device>(
            sizeof(struct jitter_pdin), name, string("pd.in"), pdin_desc);
            
    // create named process data for outputs 
    string pdout_desc = 
        "uint64_t: max_ever_clamp\n";

    pdout = make_shared<robotkernel::process_data_device>(
            sizeof(struct jitter_pdout), name, string("pd.out"), pdout_desc);

    // create ipc structures
    pthread_mutex_init(&sync_lock, NULL);
    pthread_cond_init(&sync_cond, NULL);

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
    
    // create trigger device for pdin
//    pdin_t_dev = make_shared<trigger_base>(format_string("%s.pd.in.trigger", name));

    calibrate();
}

//! default destruction
jitter_measurement::~jitter_measurement() {
    if (buffer[0]) 
        delete[] buffer[0];
    if (buffer[1]) 
        delete[] buffer[1];

    if (log_diff)
        delete[] log_diff;

#ifdef HAVE_TERMIOS_H
    if(tty_port)
        delete tty_port;
#endif    
    // destroy ipc structures
    pthread_mutex_destroy(&sync_lock);
    pthread_cond_destroy(&sync_cond);

    // destroy trigger device
//    pdin_t_dev.reset();
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

//! register services
void jitter_measurement::register_services() { 
    stringstream base;
    base << name << "."; 

    // register services
    kernel& k = *kernel::get_instance();
    k.add_service(name, base.str() + "reset_max_ever",
            service_definition_reset_max_ever,
            std::bind(&jitter_measurement::service_reset_max_ever, this, _1, _2));
    k.add_service(name, base.str() + "get_cps",
            service_definition_get_cps,
            std::bind(&jitter_measurement::service_get_cps, this, _1, _2));
}

void jitter_measurement::register_pd() {
    kernel& k = *kernel::get_instance();
	k.add_device(shared_from_this());
}
    
void jitter_measurement::unregister_pd() {
    kernel& k = *kernel::get_instance();
	k.remove_device(shared_from_this());
}

//! calibrate function for clocks per second
void jitter_measurement::calibrate() {
    if (cps != 1)
        return;

    buffer_pos  = 0;
    auto& buf = pdin->get_write_buffer();
    memset(&buf[0], 0, buf.size());
#if !defined(NO_RDTSC)
    log(info, "calibrating clocks/sec...\n");

#ifdef __VXWORKS__
    taskDelay(1);
    uint64_t begin = __rdtsc();
    taskDelay(1);
    cps = (uint64_t)((__rdtsc() - begin - 1.2E5) * sysClkRateGet());
#else
    struct timespec ts_time;
    clock_gettime(CLOCK_REALTIME, &ts_time);
    double begin_time = (double)ts_time.tv_sec + (ts_time.tv_nsec / 1E9);
    uint64_t begin = __rdtsc();

    struct timespec ts = { 0, 10000000 };
    nanosleep(&ts, NULL);
    
    clock_gettime(CLOCK_REALTIME, &ts_time);
    double end_time = (double)ts_time.tv_sec + (ts_time.tv_nsec / 1E9);
    uint64_t end = __rdtsc();
    log(info, "got %17.13f sec diff\n", (end_time - begin_time));
    cps = (end - begin) / (end_time - begin_time); // magic factor to correct cps
#endif
    log(info, "got %llu clock/sec\n", cps);
#else
    cps = 1e9;
#endif
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
#if !defined(NO_RDTSC)
    return __rdtsc();
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)(ts.tv_sec * 1e9 + ts.tv_nsec);
#endif
}

void jitter_measurement::trigger() {
    do_pulse(pulse_on_trigger);

    // get actual timestamp
    uint64_t ts = get_timestamp();

    auto& buf = pdin->get_write_buffer();
    struct jitter_pdin *j_pdin = (struct jitter_pdin *)&buf[0];
    j_pdin->last_ts = ts;
    pdin->swap_buffers();
//    pdin_t_dev->trigger_modules();

    buffer[buffer_act][buffer_pos++] = ts;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        buffer_act = (buffer_act + 1) % 2; 

        if (threaded)
            pthread_cond_signal(&sync_cond);
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
    int act_buf = (buffer_act + 1) % 2;

    uint64_t dev;
    uint64_t cycle = 0, avgjit = 0, maxjit = 0;

    auto& buf = pdin->get_write_buffer();
    struct jitter_pdin *pdin = (struct jitter_pdin *)&buf[0];
    
    auto& bufout = pdout->get_read_buffer();
    struct jitter_pdout *pdout = (struct jitter_pdout *)&bufout[0];


    // calculate differences and sum of differences
    for (i = 0; i < buffer_size - 1; ++i) {
        log_diff[i]  = buffer[act_buf][i+1] - buffer[act_buf][i];
        cycle       += log_diff[i];
    }
    cycle /= buffer_size - 1;

    // calculating maximum deviation
    double fac = (1E6 / (double)cps);
    uint64_t new_maxever_time = 0;
    for (i = 0; i < buffer_size - 1; i++) {
        dev     = labs(log_diff[i] - cycle); 
        maxjit  = max(dev, maxjit);
        uint64_t this_maxjit = (uint64_t)(maxjit * fac);
        if(this_maxjit > pdin->maxever) {
            // new max ever!
            pdin->maxever = this_maxjit;
            new_maxever_time = buffer[act_buf][i];
        }
        avgjit += (dev * dev);
    }
    avgjit /= (buffer_size - 1);

    avgjit = (uint64_t)(sqrt((double)avgjit) * fac);
    cycle  = (uint64_t)(cycle  * fac);
    maxjit = (uint64_t)(maxjit * fac);
    pdin->last_cycle = cycle;
    pdin->last_max = maxjit;

    if(new_maxever_time) {
        do_pulse(pulse_on_new_max_ever);
        double seconds_ago = (get_timestamp() - new_maxever_time) / (double)cps;
        log(info, "new max ever is %.3fms ago\n", seconds_ago);
        pdin->maxever_time = get_seconds() - seconds_ago;
        time_t int_ts = (int)pdin->maxever_time;
        struct tm* btime = localtime(&int_ts);
        strftime(maxever_time_string, 64, " (at %H:%M:%S", btime);
        unsigned int part_second = (unsigned int)(10000 * (pdin->maxever_time - (int)pdin->maxever_time));
        if(part_second >= 10000)
            part_second = 9999;
        snprintf(maxever_time_string + strlen(maxever_time_string), 32, ".%04d", part_second);
    }

    if(pdout->max_ever_clamp != 0 && pdin->maxever > pdout->max_ever_clamp)
        pdin->maxever = pdout->max_ever_clamp;

    char running_maxever_time_string[128];
    if(maxever_time_string[0] == 0)
        running_maxever_time_string[0] = 0;
    else
        snprintf(running_maxever_time_string, 128, "%s, %.1fs ago)",
                maxever_time_string,
                get_seconds() - pdin->maxever_time);

    log(info, "mean period: %4lluus, jitter mean:"
            " %2lluus, max %4lluus, max ever %4lluus%s\n",
            cycle, avgjit, maxjit, pdin->maxever, running_maxever_time_string);

    if(new_maxever_time && new_maxever_command.size() && pdin->maxever > new_maxever_command_threshold) {
        string cmd = format_string("%s %llu %s",
                new_maxever_command.c_str(), pdin->maxever, maxever_time_string + 5);
        log(info, "execute new_maxever_command: %s\n", cmd.c_str());
        system(cmd.c_str());
    }
}

//! handler function called if thread is running
void jitter_measurement::run() {
    pthread_mutex_lock(&sync_lock);

    while (running()) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec++;
        if (pthread_cond_timedwait(&sync_cond, &sync_lock, &ts) != 0)
            continue;

        print();
    }

    pthread_mutex_unlock(&sync_lock);
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
    auto& buf = pdin->get_write_buffer();
    struct jitter_pdin *pdin = (struct jitter_pdin *)&buf[0];

#define RESET_MAX_EVER_RESP_MAXEVER 0
    response[RESET_MAX_EVER_RESP_MAXEVER] = pdin->maxever;
    pdin->maxever = 0;
    pdin->maxever_time = 0;
    maxever_time_string[0] = 0;

    return 0;
}

const std::string jitter_measurement::service_definition_reset_max_ever = 
    "response:\n"
    "   uint32_t: maxever\n";

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
    "   uint32_t: cps\n";

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

            k.remove_device(pdin);
//            k.remove_trigger_device(pdin_t_dev);

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
        case preop_2_safeop: {
            // ====> start receiving measurements
            if (threaded)
                start();
            
//            k.add_trigger_device(pdin_t_dev);
            k.add_device(pdin);

            if (state == module_state_safeop)
                break;
        }
        case safeop_2_op: {
            // ====> start sending commands           
            k.add_device(pdout);
            break;
        }
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

    const auto& buf = pdin->get_read_buffer();
    pd.resize(buf.size());
    memcpy(&pd[0], &buf[0], buf.size());
}

//! return output process data (commands)
/*!
 * \param pd return output process data
 */
void jitter_measurement::get_pdout(
        service_provider::process_data_inspection::pd_t& pd) {

    const auto& buf = pdout->get_read_buffer();
    pd.resize(buf.size());
    memcpy(&pd[0], &buf[0], buf.size());
}


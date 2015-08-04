//! robotkernel module jitter_measurement
/*!
 * author: Robert Burger
 *
 * $Id$
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

#include "yaml-cpp/yaml.h"
#include <string_util/string_util.h>

MODULE_DEF(module_jitter_measurement, module_jitter_measurement::jitter_measurement)

using namespace std;
using namespace robotkernel;
using namespace module_jitter_measurement;

#if !defined(NO_RDTSC)
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
: runnable(0, 0), module_base("module_jitter_measurement", name) {
    buffer_size = node["buffer_size"].to<unsigned int>();

    buffer[0]       = new uint64_t[buffer_size];
    buffer[1]       = new uint64_t[buffer_size];
    log_diff        = new uint64_t[buffer_size];
    buffer_pos      = 0;
    buffer_act      = 0;
    cps             = get_as<uint64_t>(node, "cps", 1);
    pd_interface_id = NULL;
    threaded        = get_as<bool>(node, "threaded", true);
    pdout.max_ever_clamp = 0; // disable clamp
    memset(&pdin, 0, sizeof(pdin));

    new_maxever_command = 
        get_as<string>(node, "new_maxever_command", string(""));
    new_maxever_command_threshold = 
        get_as<int>(node, "new_maxever_command_threshold", 50);

    memset(buffer[0], 0, sizeof(uint64_t) * buffer_size);
    memset(buffer[1], 0, sizeof(uint64_t) * buffer_size);
    memset(log_diff, 0, sizeof(uint64_t) * buffer_size);

    // create ipc structures
    pthread_mutex_init(&sync_lock, NULL);
    pthread_cond_init(&sync_cond, NULL);

    // set state to init
    state = module_state_init;

    is_printing = false;
    maxever_time_string[0] = 0;
	    
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

    // destroy ipc structures
    pthread_mutex_destroy(&sync_lock);
    pthread_cond_destroy(&sync_cond);
}

//! register services
void jitter_measurement::register_services() { 
    // register services
    kernel& k = *kernel::get_instance();
    if (k.clnt) {
        string base = k.clnt->name + "." + this->name + ".";
        register_reset_max_ever(k.clnt, base + "reset_max_ever");
        register_get_cps(k.clnt, base + "get_cps");
    }
}

void jitter_measurement::register_pd() {
    if(pd_interface_id)
        return;
    pd_interface_id = kernel::register_interface_cb(
            name.c_str(), 
            "libinterface_process_data_inspection.so", 
            "last_jitter", 0);
}
    
void jitter_measurement::unregister_pd() {
    if(!pd_interface_id)
        return;
    kernel::unregister_interface_cb(pd_interface_id);
    pd_interface_id = NULL;
}

#ifdef __VXWORKS__
#include <vxWorks.h>
#include <sysLib.h>
#include <taskLib.h>
#endif

//! calibrate function for clocks per second
void jitter_measurement::calibrate() {
    if (cps != 1)
        return;

    buffer_pos  = 0;
    memset(&pdin, 0, sizeof(pdin));
#if !defined(NO_RDTSC)
    log(module_info, "calibrating clocks/sec...\n");

#ifdef __VXWORKS__
    taskDelay(1);
    uint64_t begin = __rdtsc();
    taskDelay(1);
    cps = (uint64_t)((__rdtsc() - begin - 1.2E5) * sysClkRateGet());
#else
    uint64_t begin = __rdtsc();
    struct timespec ts = { 0, 1E7 };
    nanosleep(&ts, NULL);
    cps = (__rdtsc() - begin) * 98.3; // magic factor to correct cps
#endif
    log(module_info, "got %llu clock/sec\n", cps);
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
	pdin.last_ts = get_timestamp();
	
	buffer[buffer_act][buffer_pos++] = pdin.last_ts;
	if (buffer_pos >= buffer_size) {
		buffer_pos = 0;
		buffer_act = (buffer_act + 1) % 2; 
		
		if (threaded)
			pthread_cond_signal(&sync_cond);
		else
			print();
	}
	trigger_modules();
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
	if(this_maxjit > pdin.maxever) {
		// new max ever!
		pdin.maxever = this_maxjit;
		new_maxever_time = buffer[act_buf][i];
	}
        avgjit += (dev * dev);
    }
    avgjit /= (buffer_size - 1);

    avgjit = (uint64_t)(sqrt((double)avgjit) * fac);
    cycle  = (uint64_t)(cycle  * fac);
    maxjit = (uint64_t)(maxjit * fac);
    pdin.last_cycle = cycle;
    pdin.last_max = maxjit;

    if(new_maxever_time) {
	    double seconds_ago = (get_timestamp() - new_maxever_time) / (double)cps;
	    pdin.maxever_time = get_seconds() - seconds_ago;
	    time_t int_ts = (int)pdin.maxever_time;
	    struct tm* btime = localtime(&int_ts);
	    strftime(maxever_time_string, 64, " (at %H:%M:%S", btime);
	    unsigned int part_second = 10000 * (pdin.maxever_time - (int)pdin.maxever_time);
	    if(part_second >= 10000)
		    part_second = 9999;
	    snprintf(maxever_time_string + strlen(maxever_time_string), 32, ".%04d", part_second);
    }
	    
    if(pdout.max_ever_clamp != 0 && pdin.maxever > pdout.max_ever_clamp)
        pdin.maxever = pdout.max_ever_clamp;

    char running_maxever_time_string[128];
    if(maxever_time_string[0] == 0)
	    running_maxever_time_string[0] = 0;
    else
	    snprintf(running_maxever_time_string, 128, "%s, %.1fs ago)",
		     maxever_time_string,
		     get_seconds() - pdin.maxever_time);
    
    log(module_info, "mean period: %4lluus, jitter mean:"
            " %2lluus, max %4lluus, max ever %4lluus%s\n",
	   cycle, avgjit, maxjit, pdin.maxever, running_maxever_time_string);

    if(new_maxever_time && new_maxever_command.size() && pdin.maxever > new_maxever_command_threshold) {
	    string cmd = format_string("%s %llu %s",
				       new_maxever_command.c_str(), pdin.maxever, maxever_time_string + 5);
	    log(module_info, "execute new_maxever_command: %s\n", cmd.c_str());
	    system(cmd.c_str());
    }
	   
}

//! handler function called if thread is running
void jitter_measurement::run() {
	// calibrate();

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

//! service callbacks
int jitter_measurement::on_reset_max_ever(ln::service_request& req, 
        ln_service_robotkernel_jitter_measurement_reset_max_ever& svc) {
    svc.resp.maxever = pdin.maxever;
    pdin.maxever = 0;
    pdin.maxever_time = 0;
    maxever_time_string[0] = 0;
    req.respond();
    return 0;
}

int jitter_measurement::on_get_cps(ln::service_request& req, 
        ln_service_robotkernel_jitter_measurement_get_cps& svc) {
    svc.resp.cps = cps;
    req.respond();
    return 0;
}

int jitter_measurement::set_state(module_state_t state) {
    log(module_info, "state change from %s to %s requested\n", 
            state_to_string(this->state), state_to_string(state));

    switch (state) {
        case module_state_init:
            stop();
            unregister_pd();
            break;
        case module_state_preop:
            register_pd();
            if (threaded)
                start();
            break;
        case module_state_safeop:
            break;
        case module_state_op:

            if (this->state < module_state_safeop)
                // invalid state transition
                return -1;
            break;
        default:
            // invalid state
            return -1;
    }

    // set actual state
    this->state = state;

    return 0;
}
        
int jitter_measurement::request(int reqcode, void* ptr) {
    int ret = 0;
    if (trigger_base::request(reqcode, ptr) == 0)
        return 0;

    switch (reqcode) {
        case MOD_REQUEST_REGISTER_SERVICES:
            register_services();
            break;
        case MOD_REQUEST_GET_PDIN: {
            process_data_t *pdg = (process_data_t *)ptr;
            pdg->pd = &pdin;
            pdg->len = sizeof(pdin);
            break;
        }
        case MOD_REQUEST_GET_PDOUT: {
            process_data_t *pdg = (process_data_t *)ptr;
            pdg->pd = &pdout;
            pdg->len = sizeof(pdout);
            break;
        }
        default:
            log(module_verbose, "not implemented request %d\n", reqcode);
            ret = -1;
            break;
    }

    return ret;
}


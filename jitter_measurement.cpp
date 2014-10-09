//! ÜBER-control Module jitter_measurement
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
#include "robotkernel/kernel.h"
#include "config.h"
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

using namespace std;
using namespace robotkernel;

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
    : runnable(0, 0) {
    buffer_size = node["buffer_size"].to<unsigned int>();
    this->name = string(name);

    buffer      = new uint64_t[buffer_size];
    log_diff    = new uint64_t[buffer_size];
    buffer_pos  = 0;
    memset(&pdin, 0, sizeof(pdin));
    pdout.max_ever_clamp = 0; // disable clamp
    cps         = 1;
    pd_interface_id = NULL;
    
    if (node.FindValue("cps"))
        cps = node["cps"].to<uint64_t>();

    memset(buffer, 0, sizeof(uint64_t) * buffer_size);
    memset(log_diff, 0, sizeof(uint64_t) * buffer_size);

    // create ipc structures
    pthread_mutex_init(&sync_lock, NULL);
    pthread_cond_init(&sync_cond, NULL);
    
    // set state to init
    state = module_state_init;

    calibrate();
}

//! default destruction
jitter_measurement::~jitter_measurement() {
    if (buffer) {
        delete[] buffer;
    }

    if (log_diff) {
        delete[] log_diff;
    }

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
    jm_log(name, info, "calibrating clocks/sec...\n");

#ifdef __VXWORKS__
    taskDelay(1);
    uint64_t begin = __rdtsc();
    taskDelay(1);
    cps = (__rdtsc() - begin - 1.2E5) * sysClkRateGet();
#else
    uint64_t begin = __rdtsc();
    struct timespec ts = { 0, 1E7 };
    nanosleep(&ts, NULL);
    cps = (__rdtsc() - begin) * 98.3; // magic factor to correct cps
#endif
    jm_log(name, info, "got %llu clock/sec\n", cps);
#else
    cps = 1e9;
#endif
}

//! does one measurement
/*!
 * if log buffer is full, output thread is triggered
 */
void jitter_measurement::measure() {
#if !defined(NO_RDTSC)
    pdin.last_ts = __rdtsc();
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    pdin.last_ts = (uint64_t)(ts.tv_sec * 1e9 + ts.tv_nsec);
#endif
    buffer[buffer_pos++] = pdin.last_ts;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        print();
        //pthread_cond_signal(&sync_cond);
    }
}

//! returns last measuremente
double jitter_measurement::last_measurement() { 
    uint64_t tick_act, tick_last;
    int act_pos = buffer_pos;

    // get actual tick
    if (act_pos == 0)
        act_pos = buffer_size - 1;
    tick_act = buffer[act_pos];
    
    // get last tick
    if (act_pos == 0)
        act_pos = buffer_size - 1;
    tick_last = buffer[act_pos];

    return (double)(tick_act - tick_last)/cps;
}

//! print last buffer measurement values
void jitter_measurement::print() {
    unsigned int i;

    uint64_t dev;
    uint64_t cycle = 0, avgjit = 0, maxjit = 0;

    // calculate differences and sum of differences
    for (i = buffer_pos; i < buffer_size - 1; ++i) {
        log_diff[i] = buffer[i+1] - buffer[i];
        cycle += log_diff[i];
    }
    cycle /= buffer_size - buffer_pos;

    // calculating maximum deviation 
    for (i = buffer_pos; i < buffer_size - 1; i++) {
        dev = fabsl(((long double)log_diff[i]) - cycle);
        maxjit = max(dev, maxjit);
        avgjit += (dev * dev);
    }
    avgjit /= buffer_size - 1 - buffer_pos;

    double fac = (1E6 / (double)cps);
    avgjit = sqrt((double)avgjit);
    cycle  = cycle  * fac;
    pdin.last_cycle = cycle;
    avgjit = avgjit * fac;
    maxjit = maxjit * fac;
    pdin.last_max = maxjit;
    pdin.maxever = max(pdin.maxever, maxjit);
    if(pdout.max_ever_clamp != 0 && pdin.maxever > pdout.max_ever_clamp)
	    pdin.maxever = pdout.max_ever_clamp;

    trigger_modules();
    
    jm_log(name, info, "mean period: %4lluus, jitter mean:"
            " %2lluus, max %3lluus, max ever %3lluus\n",
            cycle, avgjit, maxjit, pdin.maxever);
}
        
//! handler function called if thread is running
void jitter_measurement::run() {
    jm_log(name, info, "handler running\n");
    calibrate();

    pthread_mutex_lock(&sync_lock);

    while (_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec++;
        if (pthread_cond_timedwait(&sync_cond, &sync_lock, &ts) != 0)
            continue;

        print();
    }

    pthread_mutex_unlock(&sync_lock);

    jm_log(name, info, "handler stopped\n");
}

//! service callbacks
int jitter_measurement::on_reset_max_ever(ln::service_request& req, 
        ln_service_robotkernel_jitter_measurement_reset_max_ever& svc) {
    svc.resp.maxever = pdin.maxever;
    pdin.maxever = 0;
    req.respond();
    return 0;
}
        
int jitter_measurement::on_get_cps(ln::service_request& req, 
        ln_service_robotkernel_jitter_measurement_get_cps& svc) {
    svc.resp.cps = cps;
    req.respond();
    return 0;
}

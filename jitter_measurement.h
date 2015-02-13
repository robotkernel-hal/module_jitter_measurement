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

#ifndef __JITTER_MEASSUREMENT_H__
#define __JITTER_MEASSUREMENT_H__

#include "module_jitter_measurement.h"

#define LN_UNREGISTER_SERVICE_IN_BASE_DETOR  
#include "ln_messages.h"
#undef LN_UNREGISTER_SERVICE_IN_BASE_DETOR

#include "robotkernel/runnable.h"
#include "robotkernel/trigger_base.h"
#include "config.h"

#include "yaml-cpp/yaml.h"

class jitter_measurement :
    public robotkernel::runnable,
    public robotkernel::trigger_base,
    public ln_service_reset_max_ever_base,
    public ln_service_get_cps_base {

    private: 
        //! position in buffer
        unsigned int buffer_act;
        unsigned int buffer_pos;

        //! memory buffer for jitter measurement
        uint64_t *buffer[2];
        uint64_t *log_diff;
        uint64_t cps;

        //! print thread sync
        pthread_mutex_t sync_lock;
        pthread_cond_t sync_cond;

        //! print last buffer measurement values
        void print();

        //! handler function called if thread is running
        void run();

    public:

        module_state_t state;     //! module state
        std::string name;         //! jitter_measurement name
        size_t buffer_size;       //! size of jitter measurement buffer
        robotkernel::kernel::interface_id_t pd_interface_id;
        bool threaded;
	bool is_printing;
	char maxever_time_string[64];
	std::string new_maxever_command;
	
	struct jitter_pdin {
		uint64_t maxever;         //! max ever seen jitter
		uint64_t last_max;
		uint64_t last_cycle;
		uint64_t last_ts;
		double maxever_time; // unix timestamp of last maxever increment!
	} pdin;
	
	struct jitter_pdout {
		uint64_t max_ever_clamp;
	} pdout;

        //! yaml config construction
        /*!
         * \param name of jm 
         * \param node yaml node
         */
        jitter_measurement(const char* name, const YAML::Node& node);

        //! default destruction
        ~jitter_measurement();

        //! register services
        void register_services();
        void register_pd();
        void unregister_pd();

        //! does one measurement
        /*!
         * if log buffer is full, output thread is triggered
         */
        void measure();

        //! calibrate function for clocks per second
        void calibrate();

        //! returns last measuremente
        double last_measurement();

        //! service callbacks
        int on_reset_max_ever(ln::service_request& req, 
                ln_service_robotkernel_jitter_measurement_reset_max_ever& svc);
        int on_get_cps(ln::service_request& req, 
                ln_service_robotkernel_jitter_measurement_get_cps& svc);
};

#endif // __JITTER_MEASSUREMENT_H__


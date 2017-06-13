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

#ifndef __MODULE_JITTER_MEASSUREMENT_H__
#define __MODULE_JITTER_MEASSUREMENT_H__

#include "robotkernel/runnable.h"
#include "robotkernel/trigger_base.h"
#include "robotkernel/module_base.h"

#include "service_provider/process_data_inspection/base.h"

#include "yaml-cpp/yaml.h"

#ifdef HAVE_TERMIOS_H
#include "tty_control_signals.h"
#endif

namespace module_jitter_measurement {
#ifdef EMACS
}
#endif

class jitter_measurement :
    public robotkernel::runnable,
    public robotkernel::module_base,
    public service_provider::process_data_inspection::base {

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
        size_t buffer_size;       //! size of jitter measurement buffer
        bool threaded;
        bool is_printing;
        char maxever_time_string[64];
        std::string new_maxever_command;
        unsigned int new_maxever_command_threshold;
#ifdef HAVE_TERMIOS_H
        tty_control_signals* tty_port;
#endif
        enum pulse_signals_t {
            NO_PULSE,
            PULSE_RTS,
            PULSE_DTR,
            PULSE_RTS_NEG,
            PULSE_DTR_NEG
        };
        pulse_signals_t pulse_on_trigger;
        pulse_signals_t pulse_on_new_max_ever;
        void set_pulse_from_string(pulse_signals_t* which, std::string which_str);
        void do_pulse(pulse_signals_t which);

        struct jitter_pdin {
            uint64_t maxever;         //! max ever seen jitter
            uint64_t last_max;
            uint64_t last_cycle;
            uint64_t last_ts;
            double maxever_time; // unix timestamp of last maxever increment!
        };

        struct jitter_pdout {
            uint64_t max_ever_clamp;
        };
    
        // named process data
        robotkernel::sp_process_data_t pdin;
        robotkernel::sp_process_data_t pdout;

        //! yaml config construction
        /*!
         * \param name of jm 
         * \param node yaml node
         */
        jitter_measurement(const char* name, const YAML::Node& node);

        //! default destruction
        ~jitter_measurement();

        //! set module state
        /*
         * \param state module state to set
         * \return 0 on success
         */
        int set_state(module_state_t state);

        //! register services
        void register_services();
        void register_pd();
        void unregister_pd();

        //! module trigger callback
        /*! does one measurement
         *
         * if log buffer is full, output thread is triggered
         */
        void trigger();

        //! calibrate function for clocks per second
        void calibrate();

        //! returns last measuremente
        double last_measurement();

        //! reset max ever
        /*!
         * \param request service request data
         * \param response service response data
         * \return success
         */
        int service_reset_max_ever(const robotkernel::service_arglist_t& request,
                robotkernel::service_arglist_t& response);
        static const std::string service_definition_reset_max_ever;

        //! reset max ever
        /*!
         * \param request service request data
         * \param response service response data
         * \return success
         */
        int service_get_cps(const robotkernel::service_arglist_t& request,
                robotkernel::service_arglist_t& response);
        static const std::string service_definition_get_cps;

        //! return input process data (measurements)
        /*!
         * \param pd return input process data
         */
        void get_pdin(service_provider::process_data_inspection::pd_t& pd);

        //! return output process data (commands)
        /*!
         * \param pd return output process data
         */
        void get_pdout(service_provider::process_data_inspection::pd_t& pd);
};

#ifdef EMACS
{
#endif
};

#endif // __MODULE_JITTER_MEASSUREMENT_H__


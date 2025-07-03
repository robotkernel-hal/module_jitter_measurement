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
#include "robotkernel/module_base.h"
#include "robotkernel/process_data/triple_buffer.h"

#include "service_provider/process_data_inspection/base.h"

#include "yaml-cpp/yaml.h"

#include <mutex>
#include <chrono>
#include <condition_variable>

#include "config.h"

namespace module_jitter_measurement {
#ifdef EMACS
}
#endif

class jitter_measurement :
    public std::enable_shared_from_this<jitter_measurement>,
    public robotkernel::runnable,
    public robotkernel::module_base
{

    private: 
        //! position in buffer
        unsigned int buffer_act;
        unsigned int buffer_pos;

        //! memory buffer for jitter measurement
        typedef std::chrono::high_resolution_clock::time_point log_tp_t;
        typedef std::vector<log_tp_t> log_tp_vec_t;
        log_tp_vec_t buffer[2];
        
        typedef std::chrono::high_resolution_clock::duration log_dur_t;
        typedef std::vector<log_dur_t> log_dur_vec_t;
        log_dur_vec_t log_diff;
        
        log_tp_t maxever_time;

        bool do_print;

        //! print thread sync
        std::mutex              sync_mtx;
        std::condition_variable sync_cond;
        
        //! print last buffer measurement values
        void print();

        //! handler function called if thread is running
        void run();

    public:
        size_t buffer_size;       //! size of jitter measurement buffer
        std::string dump_to_file;
        int dump_fd;
        bool threaded;

        std::string maxever_time_string;

        double new_maxever_threshold; //!< threshold for trigger on new maxever

        struct jitter_pdin {
            double maxever;         //! max ever seen jitter
            double last_max;
            double last_cycle;
            uint64_t last_ts;
            double maxever_time; // unix timestamp of last maxever increment!
        };

        struct jitter_pdin local_pdin;

        struct jitter_pdout {
            double max_ever_clamp;
        };
    
        // named process data
        robotkernel::sp_process_data_t pdin;
        robotkernel::sp_pd_provider_t pdin_provider;
        service_provider::process_data_inspection::sp_pd_inspection_t pdin_inspect;
        robotkernel::sp_process_data_t pdout;
        robotkernel::sp_pd_consumer_t pdout_consumer;
        service_provider::process_data_inspection::sp_pd_inspection_t pdout_inspect;
        robotkernel::sp_trigger_t maxever_t_dev;

        //! yaml config construction
        /*!
         * \param name of jm 
         * \param node yaml node
         */
        jitter_measurement(const char* name, const YAML::Node& node);

        //! default destruction
        ~jitter_measurement();

        //! additional init function
        virtual void init() override;

        //! State transition from PREOP to SAFEOP
        virtual void set_state_op_2_safeop() override;

        //! State transition from PREOP to SAFEOP
        virtual void set_state_safeop_2_preop() override;
    
        //! State transition from PREOP to SAFEOP
        virtual void set_state_preop_2_safeop() override;

        //! State transition from PREOP to SAFEOP
        virtual void set_state_safeop_2_op() override;

        //! module trigger callback
        /*! does one measurement
         *
         * if log buffer is full, output thread is triggered
         */
        virtual void tick() override;

        //! reset max ever
        /*!
         * \param request service request data
         * \param response service response data
         * \return success
         */
        int service_reset_max_ever(const robotkernel::service_arglist_t& request,
                robotkernel::service_arglist_t& response);
        static const std::string service_definition_reset_max_ever;
};

#ifdef EMACS
{
#endif
};

#endif // __MODULE_JITTER_MEASSUREMENT_H__


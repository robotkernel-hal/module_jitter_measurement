//! ÜBER-control Module jitter_measurement
/*!
  $Id$
  */

#include "module_jitter_measurement.h"
#include "jitter_measurement.h"
#include "config.h"
#include "robotkernel/kernel.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace robotkernel;

//! log to kernel logging facility
void jm_log(std::string mod_name, robotkernel::loglevel lvl, const char *format, ...) {
    char buf[1024];

    // format argument list
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 1024, format, args);
    klog(lvl, "[module_jitter_measurement|%s] %s", mod_name.c_str(), buf);
}


#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

//! configures module
/*!
  \param config configure string
  \return handle on success, NULL otherwise
  */
MODULE_HANDLE mod_configure(const char* name, const char* config) {
    jitter_measurement *j = NULL;

    jm_log(string(name), info, "build by: " BUILD_USER "@" BUILD_HOST "\n");
    jm_log(string(name), info, "build date: " BUILD_DATE "\n");

    stringstream stream(config);
    YAML::Parser parser(stream);
    YAML::Node doc;

    if (!parser.GetNextDocument(doc)) {
        jm_log(string(name), error, "error parsing config file\n");
        goto ErrorExit;
    }

    // parsing yaml config string
    try {
        j = new jitter_measurement(name, doc);
        if (!j) {
            jm_log(string(name), error, "cannot allocate memory: %s\n",
                    strerror(errno));
            goto ErrorExit;
        }
        jm_log(string(name), info, "configured buffer_size %d\n", j->buffer_size);
    } catch (YAML::Exception& e) {
        jm_log(string(name), error, "exception creating: %s\n", e.what());
        jm_log(string(name), error, "got config string: \n====\n%s\n====\n", config);
        return (MODULE_HANDLE)NULL;
    }

    return (MODULE_HANDLE)j;

ErrorExit:
    // delete already allocated structures
    if (j)
        delete j;

    return (MODULE_HANDLE)NULL;
}

//! unconfigure module
/*!
  \param hdl module handle
  \return success or failure
  */
int mod_unconfigure(MODULE_HANDLE hdl) {
    // cast struct
    jitter_measurement *j = (jitter_measurement *)hdl;

    if (j)
        delete j;

    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
  */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    // cast struct
    jitter_measurement *j = (jitter_measurement *)hdl;
    jm_log(j->name, info, "state change form %s to %s requested\n", 
            state_to_string(j->state), state_to_string(state));

    switch (state) {
        case module_state_init:
            j->stop();
            j->unregister_pd();
            break;
        case module_state_preop:
            j->register_pd();
            if (j->threaded)
                j->start();
            break;
        case module_state_safeop:
            break;
        case module_state_op:

            if (j->state < module_state_safeop)
                // invalid state transition
                return -1;
            break;
        default:
            // invalid state
            return -1;
    }

    // set actual state
    j->state = state;

    return 0;
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
  */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    // cast struct and return state
    jitter_measurement *j = (jitter_measurement *)hdl;
    return j->state;
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    // cast struct
    jitter_measurement *j = (jitter_measurement *)hdl;
    j->measure();
}

//! send a request to module
/*! 
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
  */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    int ret = 0;

    // cast struct
    jitter_measurement *j = (jitter_measurement *)hdl;

    switch (reqcode) {
        case MOD_REQUEST_REGISTER_SERVICES:
            j->register_services();
            break;
        case MOD_REQUEST_GET_PDIN: {
            process_data_t *pdg = (process_data_t *)ptr;
            pdg->pd = &j->pdin;
            pdg->len = sizeof(j->pdin);
            break;
        }
        case MOD_REQUEST_GET_PDOUT: {
            process_data_t *pdg = (process_data_t *)ptr;
            pdg->pd = &j->pdout;
            pdg->len = sizeof(j->pdout);
            break;
        }
        case MOD_REQUEST_SET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;
            if (!j->add_trigger_module(*cb))
                ret = -1;
            break;
        }
        case MOD_REQUEST_UNSET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;
            if (!j->remove_trigger_module(*cb))
                ret = -1;
            break;
        }
        default:
            jm_log(j->name, verbose, "not implemented request %d\n", reqcode);
            ret = -1;
            break;
    }

    return ret;
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif



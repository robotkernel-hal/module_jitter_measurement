//! ÜBER-control Module jitter_measurement
/*!
  $Id$
  */

//#include "config.h"
#include "jitter_measurement.h"
#include "robotkernel/kernel.h"
#include "robotkernel/exceptions.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace robotkernel;

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

    klog(module_info, "[module_jitter_measurement|%s] build by: " BUILD_USER "@" BUILD_HOST "\n", name);
    klog(module_info, "[module_jitter_measurement|%s] build date: " BUILD_DATE "\n", name);

    stringstream stream(config);
    YAML::Parser parser(stream);
    YAML::Node doc;

    if (!parser.GetNextDocument(doc)) {
        klog(module_error, "[module_jitter_measurement|%s] error parsing config file\n", name);
        goto ErrorExit;
    }

    // parsing yaml config string
    try {
        j = new jitter_measurement(name, doc);
        if (!j) {
            klog(module_error, "[module_jitter_measurement|%s] cannot allocate memory: %s\n",
                    name, strerror(errno));
            goto ErrorExit;
        }
        klog(module_info, "[module_jitter_measurement|%s] configured buffer_size %d\n", name, j->buffer_size);
    } catch (YAML::Exception& e) {
        klog(module_error, "[module_jitter_measurement|%s| exception creating: %s\n", name, e.what());
        klog(module_error, "[module_jitter_measurement|%s] got config string: \n====\n%s\n====\n", name, config);
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
    jitter_measurement *jm_dev = reinterpret_cast<jitter_measurement *>(hdl);
    if (!jm_dev)
        throw robotkernel::str_exception("[module_jitter_measurement] invalid module "
                "handle to <jitter_measurement *>\n");
    delete jm_dev;
    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
  */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    jitter_measurement *jm_dev = reinterpret_cast<jitter_measurement *>(hdl);
    if (!jm_dev)
        throw robotkernel::str_exception("[module_jitter_measurement] invalid module "
                "handle to <jitter_measurement *>\n");

    return jm_dev->set_state(state);
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
  */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    jitter_measurement *jm_dev = reinterpret_cast<jitter_measurement *>(hdl);
    if (!jm_dev)
        throw robotkernel::str_exception("[module_jitter_measurement] invalid module "
                "handle to <jitter_measurement *>\n");

    return jm_dev->get_state();
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    jitter_measurement *jm_dev = reinterpret_cast<jitter_measurement *>(hdl);
    if (!jm_dev)
        throw robotkernel::str_exception("[module_jitter_measurement] invalid module "
                "handle to <jitter_measurement *>\n");

    jm_dev->measure();
}

//! send a request to module
/*! 
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
  */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    jitter_measurement *jm_dev = reinterpret_cast<jitter_measurement *>(hdl);
    if (!jm_dev)
        throw robotkernel::str_exception("[module_jitter_measurement] invalid module "
                "handle to <jitter_measurement *>\n");

    return jm_dev->request(reqcode, ptr);
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif



============
Introduction
============

This module is always used in combination with a trigger device created by
an other module to do jitter measurements and realtime analysis.

=============
Configuration
=============
   
.. code-block:: yaml
   :caption: main.rkc

   name: jitter_1
   so_file: libmodule_jitter_measurement.so
   config: !include jitter.rkc
   power_up: init
   depends: [ timer, ]

Provide a module configuration with the following content:

.. code-block:: yaml
   :Caption: jitter.rkc

   # Configuration file for module_jitter_measurement.
   #
   # vim: ft=yaml
   
   #########################################################
   # measurement settings
   buffer_size: 1000
   
   #########################################################
   # Trigger device
   trigger:
     # Trigger device name, must be registered to robotkernel
     # before switching to SAFEOP
     dev_name: timer.main.trigger
   
     # Optional priority with which we are triggerd
     #prio: 50
   
     # Optional cpu affinity on which cpu when run on.
     #affinity: [ 2, 3 ]
   
     # Optional trigger mode, direct mode means in callers
     # thread context, no direct mode uses worker thread.
     #direct_mode: True
   
   #########################################################
   # logging settings
   
   # Standard robotkernel module local loglevel.
   #loglevel: verbose

buffer_size
   Specifies the size of the buffer. After this amount the jitter is
   calculated and printed.

Process data
____________

The jitter measurement module provides a cyclic process data with the
actual measurement.

.. code-block:: yaml

   pds:
     module_jitter_measurement/inputs:
       maxever: { type: double }
       last_max: { type: double }
       last_cycle: { type: double }
       last_ts: { type: uint64_t }
       maxever_time: { type: double }
   
    module_jitter_measurement/outputs:
       maxever_clamp: { type: double

The process data device is named **jitter_1.inputs.pd**.

Triggers
________

This modules provides two trigger devices

**jitter_1.inputs.trigger**
   This is triggered every jitter calculation step

**jitter_1.new_maxever.trigger**
   This is triggered if a new maximum jitter value is calculated over
   the whole runtime.


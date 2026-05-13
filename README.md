This modules does jitter measurement on incoming trigger device pulses.

# Configuration

```yaml
- name: jitter_1
  so_file: libmodule_jitter_measurement.so
  config: !include jitter.rkc
  power_up: init
  depends: [ timer, ]
```

```yaml
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
```

buffer_size
:   Specifies the size of the buffer. After this amount the jitter is
    calculated and printed.

# Process data {#process_data}

The jitter measurement module provides a cyclic process data with the
actual measurement.

```yaml
- double: max_ever
- double: last_max
- double: last_cycle
- uint64_t: last_ts
- double: max_ever_time
```

The process data device is named **`<name>`.inputs.pd**.

# Triggers

This modules provides two trigger devices

**`<name>`.inputs.trigger**
:   This is triggered every jitter calculation step

**`<name>`.new_maxever.trigger**
:   This is triggered if a new maximum jitter value is calculated over
    the whole runtime.


This modules does jitter measurement on incoming trigger device pulses.

# Configuration

```yaml
- name: jitter_1
  so_file: libmodule_jitter_measurement.so
  config:
    buffer_size: 1000
  trigger:
  - dev_name: timer.posix_timer.trigger
    direct_mode: true
  power_up: init
  depends: timer
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


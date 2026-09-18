// Compile the production driver with diagnostic-only transaction tracing.
#include "imu_diagnostic_trace.h"
#define i2c_write_timeout_us imu_diagnostic_i2c_write
#define i2c_read_timeout_us imu_diagnostic_i2c_read
#include "lib/hardware_link101/src/lsm6dsox.c"

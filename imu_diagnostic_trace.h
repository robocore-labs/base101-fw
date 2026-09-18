#ifndef IMU_DIAGNOSTIC_TRACE_H
#define IMU_DIAGNOSTIC_TRACE_H
#include "hardware/i2c.h"
void imu_diagnostic_trace_set(bool enabled);
int imu_diagnostic_i2c_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data,
                             size_t len, bool nostop, uint timeout);
int imu_diagnostic_i2c_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data,
                            size_t len, bool nostop, uint timeout);
#endif

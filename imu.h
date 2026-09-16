/*
 * The BNO055 IMU, in NDOF fusion mode.
 *
 * The chip does the sensor fusion, so what comes back is an absolute
 * orientation quaternion alongside the raw vectors, already in ROS units:
 * rad/s, m/s^2 (gravity included, as sensor_msgs/Imu expects), tesla.
 *
 * If the sensor doesn't answer at boot the topics still exist, they just
 * never carry a message -- better a silent topic than made-up data.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

#include "bno055.h"

// Bring up i2c1 and the sensor. Returns false if it didn't answer.
bool imu_begin(void);

bool imu_online(void);

// One fused sample. False on any I2C error.
bool imu_read(bno055_sample_t *out);

#endif // IMU_H

/*
 * The IMU that is soldered to the board: an LSM6DSOX (accelerometer +
 * gyroscope) and an MMC5983MA magnetometer, both on i2c1.
 *
 * Together they are a 9-DOF IMU -- but a raw one. Neither chip fuses
 * anything, so there is no orientation here, only what the sensors measure:
 * acceleration, angular velocity and magnetic field. Attitude is the host's
 * job (imu_filter_madgwick, robot_localization), which is why /imu
 * goes out marked as carrying no orientation estimate.
 *
 * The two are independent devices, so one can answer and the other not; ask
 * each separately.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel[3];    // m/s^2, gravity included, as sensor_msgs/Imu expects
    float gyro[3];     // rad/s
    float temp_c;      // the IMU die, not the room
} imu_sample_t;

// Latest initialization probes and sampling counters; independent of ROS time sync.
typedef struct {
    bool sensor_online, mag_online;
    bool primary_ack, alternate_ack;
    uint8_t primary_id, alternate_id;
    uint32_t read_successes, read_failures, temperature_failures;
    uint32_t init_attempts;
} imu_health_t;

void imu_get_health(imu_health_t *out);
#ifdef IMU_DIAGNOSTIC
void imu_diagnostic_set_baud(uint32_t baud);
uint32_t imu_diagnostic_get_baud(void);
#endif

// Bring up i2c1 and both chips. Returns true if either answered -- check
// imu_online() / imu_mag_online() for which.
bool imu_begin(void);

// Retry configuration of an offline LSM6DSOX once per second; no bus reset.
void imu_update(void);

bool imu_online(void);
bool imu_mag_online(void);

// Acceleration, angular velocity and die temperature. False if the IMU is
// offline or the read failed.
bool imu_read(imu_sample_t *out);

// Magnetic field in tesla. False if the magnetometer is offline or the read
// failed.
bool imu_read_magnetic_field(float tesla[3]);

#endif // IMU_H

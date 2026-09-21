/*
 * The four drive wheels (Waveshare DDSM210, velocity mode).
 *
 * One motor per port: a DDSM210 cannot share a TX line without external
 * gating, so each wheel gets its own PIO UART. Which pins, which corner and
 * which way it spins is the WHEELS table in robot.h; everything here is
 * indexed by position in that table.
 *
 * Wheels that don't answer at boot are remembered as offline and skipped,
 * so a disconnected motor costs one probe at startup and nothing after.
 *
 * Glide shapes body targets once, closes gyro yaw and allocates wheels at
 * 50 Hz. Startup requires a 15-second stationary bias calibration.
 * The motor ramp is set fast to follow software-shaped commands.
 */

#ifndef WHEELS_H
#define WHEELS_H

#include <stdbool.h>
#include <stdint.h>
#include "imu.h"

typedef struct {
    bool calibrated, fresh, saturated;
    unsigned samples, rejected_windows;
    double bias, yaw_rate, correction;
    uint64_t elapsed_us;
} wheels_yaw_status_t;
// Starts a new stationary bias window and brakes all wheels. No pose reset.
void wheels_restart_gyro_calibration(void);
void wheels_observe_gyro(const imu_sample_t *sample, uint64_t now_us, bool valid);
bool wheels_yaw_rate(uint64_t now_us, double *rate);
void wheels_get_yaw_status(uint64_t now_us, wheels_yaw_status_t *out);

// Open a port per wheel, find out who is there, and put them in velocity
// mode. Returns how many answered.
uint8_t wheels_begin(void);

bool wheels_online(uint8_t index);

// Record body velocity (linear.x and angular.z) for software shaping.
// Scales excessive wheel rates together. False for non-finite or overflowing
// inputs, leaving targets unchanged; the caller must brake on rejection.
bool wheels_set_velocity(double linear_m_s, double angular_rad_s);

// Advance body profiles using monotonic dt, allocate and send wheel setpoints.
// A timing gap over 100 ms during active motion brakes and resets profiles.
void wheels_update(void);

// Stop immediately using electric brake and latch until a fresh speed command.
// Clears body profiles; normal zero commands use software shaping.
void wheels_brake(uint8_t index);

// Encoder-only body velocity from left/right mean measured wheel speeds. Requires all four healthy replies
// within the freshness window; never substitutes command targets.
bool wheels_measured_twist(uint64_t now_us, double *linear_m_s, double *angular_rad_s);

// Raw ROS odometry input from fresh front-left/front-right speed feedback.
// Applies the calibrated ICR coefficient to wheel yaw; no IMU or rear averaging.
bool wheels_odometry_twist(uint64_t now_us, double *linear_m_s, double *angular_rad_s);

// Where the wheel is now, in radians, counting full turns since boot.
// Returns 0 for a wheel that is offline or didn't answer this time.
double wheels_read_angle(uint8_t index);

// Shared diagnostics and runtime tuning for the standalone calibrator.
typedef struct {
    bool motion_enabled, at_rest;
    unsigned brake_ok_mask;
    double requested_vx, requested_wz, shaped_vx, shaped_wz;
    double commanded_rpm[4], measured_rpm[4];
    bool online[4], fresh[4];
    uint64_t sample_us[4];
    uint8_t error[4], temperature[4];
} wheels_drive_status_t;
void wheels_get_drive_status(uint64_t now_us, wheels_drive_status_t *out);
bool wheels_configure_motion(double ax, double jx, double aw, double jw, double icr);
#include "calibration_protocol.h"
bool wheels_configure_calibration(const calibration_parameters_t *parameters);
void wheels_get_parameters(calibration_parameters_t *parameters);
#ifdef CALIBRATION_FIRMWARE
// Retry boot-offline motors on existing ports, only with motion disabled.
void wheels_retry_offline(void);
#endif

#endif // WHEELS_H

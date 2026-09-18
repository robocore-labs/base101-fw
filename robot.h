/*
 * base101 — everything you might want to change.
 * ==============================================
 *
 * This is the whole configuration of the robot: what is wired where, which
 * motor is which wheel, which way each one spins, what gets published and
 * how often. Nothing else in the firmware has numbers like these in it, so
 * if you are retuning, re-wiring or renaming a joint, you are in the right
 * file — edit it and reflash.
 *
 * Board pins that are soldered down (the servo bus, I2C, the LED strip) are
 * not here: they live in link101/pins.h, because they are facts about the
 * board rather than choices about the robot. What IS here is everything
 * plugged into the breakout headers.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "link101/pins.h"

// ===========================================================================
//  Wiring
// ===========================================================================

// The IMU soldered to the board: an LSM6DSOX (accel + gyro) and an
// MMC5983MA magnetometer, both on i2c1 — the same bus as the Qwiic
// connector. Addresses are fixed by the layout; see link101/pins.h.
#define IMU_I2C         i2c1
#define IMU_ADDR        LINK101_LSM6DSOX_I2C_ADDR   // 0x6B
#define MAG_ADDR        LINK101_MMC5983_I2C_ADDR    // 0x30
#ifndef IMU_I2C_BAUD
#define IMU_I2C_BAUD    400000      // fast mode
#endif

// Wheel motors. Each DDSM210 needs its own UART -- it cannot share a TX
// line without external gating -- so this is four PIO UARTs on the
// breakout headers. Lower pin is our TX (to the motor's RX).
#define WHEEL_BAUD      115200

// ===========================================================================
//  The wheels
// ===========================================================================
//
// Four independent wheels. Left/right motors face opposite directions.
// Physical forward was verified opposite to the original polarity mapping.
// cmd_vel uses linear.x (m/s) and angular.z (rad/s).
//
// Every motor keeps the factory ID 1: each one is alone on its own bus, so
// the IDs never collide. Only change an id here if you re-addressed a motor.
//
// If a wheel stays silent, swap its tx/rx -- connector wiring varies.

#define WHEEL_COUNT 4

typedef struct {
    uint8_t     tx_pin, rx_pin;
    uint8_t     id;
    const char *joint;
    int8_t      direction;   // motor polarity: +1 or -1
    bool        left;        // physical side, independent of motor polarity
} wheel_cfg_t;

static const wheel_cfg_t WHEELS[WHEEL_COUNT] = {
    { 21, 22, 1, "front_left_wheel_joint",  +1, true  },
    { 27, 28, 1, "front_right_wheel_joint", -1, false }, // ADC1 TX, ADC2 RX
    { 25, 26, 1, "back_left_wheel_joint",   +1, true  },
    { 23, 24, 1, "back_right_wheel_joint",  -1, false },
};

// Wheel driver indices follow the table order above.
// Radius from the hardware controller; track width measured in Fusion.
#define WHEEL_RADIUS_M        0.0363
#define WHEEL_SEPARATION_M    0.2899 // Measured 307 mm outside span minus 17.10 mm width.

#define WHEEL_MAX_RPM     200    // command cap; the hardware ceiling is 210
#define WHEEL_ACCEL_TIME  1      // fast motor ramp follows software-shaped targets
#define WHEEL_SPEED_SCALE 1.0    // fudge factor on commanded speed

// Tested in the standalone terminal at +/-100 RPM. Body-space shaping
// precedes wheel allocation; angular settings still need ground testing.
#define DRIVE_LINEAR_ACCEL_M_S2   0.7
#define DRIVE_LINEAR_JERK_M_S3    2.0
#define DRIVE_YAW_ACCEL_RAD_S2    3.0
#define DRIVE_YAW_JERK_RAD_S3     10.0
#define WHEEL_CONTROL_HZ           50

// Glide yaw closure. ICR is initial feedforward from the ~180/120 floor test.
#define GLIDE_ICR_COEFF             1.5
#define GLIDE_VX_MAX                0.75
#define GLIDE_WZ_MAX                2.0
// Isolation build: disable PI to distinguish feedback-induced jerk from shaping.
#define GLIDE_YAW_FEEDBACK_ENABLED  false
#define GLIDE_YAW_CORRECTION_ACCEL  0.25 // rad/s^2, separate from operator command shaping
#define GLIDE_YAW_CORRECTION_JERK   1.0  // rad/s^3
#define GLIDE_YAW_KP                0.5
#define GLIDE_YAW_KI                0.3
#define GLIDE_YAW_CORRECTION_MAX    0.5
#define GLIDE_YAW_DEADBAND          0.005
#define GLIDE_YAW_LPF_HZ            20.0
#define GYRO_CALIBRATION_US         15000000u
#define GYRO_CALIBRATION_MIN_SAMPLES 750u
#define GYRO_CALIBRATION_MAX_BIAS   0.05
#define GYRO_CALIBRATION_MAX_STDDEV 0.003
#define GYRO_MAX_AGE_US             50000u
#define TOPIC_GYRO_CALIBRATE        "axon/gyro_calibrate" // Bool true: brake, restart 15s calibration.

// ===========================================================================
//  ROS
// ===========================================================================
//
// Active ROS interface: cmd_vel subscription and onboard IMU publishers.

#define NODE_NAME        "axon"
#define ROS_DOMAIN_ID    0
#define ZENOH_MODE       "client"
#define ZENOH_LOCATOR    "serial/cdc#baudrate=921600"

#define TOPIC_CMD_VEL        "cmd_vel"
#define TOPIC_IMU            "imu"
#define TOPIC_IMU_MAG        "imu/mag"
#define TOPIC_IMU_TEMP       "imu/temperature"
#define TOPIC_IMU_STATUS     "imu/status"
#define IMU_STATUS_HZ        1

// Four-timestamp exchange with tools/time_sync_host.py.
#define TOPIC_TIME_SYNC_REQUEST   "axon/time_sync/request"
#define TOPIC_TIME_SYNC_RESPONSE  "axon/time_sync/response"
#define TIME_SYNC_HZ              1
#define TIME_SYNC_MAX_RTT_US      20000
#define TIME_SYNC_TIMEOUT_MS      10000

// Odometry: encoder-only translation and differential wheel yaw.
#define TOPIC_ODOM           "odom"
#define TOPIC_ODOM_RESET     "odom/reset" // std_msgs/Bool: true resets pose only
#define ODOM_FRAME_ID        "odom"
#define ODOM_CHILD_FRAME_ID  "base_link"
#define PUBLISH_ODOM_TF      true // Disable when a host EKF owns this transform.
#define TOPIC_TF             "tf"
#define ODOM_HZ              50
#define ODOM_ENCODER_SIGN    1.0 // Commands and feedback share the physical motor polarity.
#define IMU_SAMPLE_HZ        208
#define ODOM_FEEDBACK_MAX_AGE_US 100000u
#define ODOM_MAX_DT_US       50000u
#define GYRO_Z_SIGN          1.0
// Gyro bias is measured on every boot by the stationary calibration window.

#define IMU_FRAME_ID     "imu_link"

// How often each thing is published. Everything is sampled from the main
// loop, so these are ceilings, not guarantees -- a slow bus read pushes the
// whole loop out.
#define IMU_HZ           50

// Stale cmd_vel actively brakes; repeat brake frames while commands are stale.
#define COMMAND_TIMEOUT_MS     500
#define COMMAND_WATCHDOG_HZ    5

// Neither chip reports per-axis variance, so these nominal diagonals go out
// with every sample. Off-diagonal terms are zero.
//
// There is no orientation covariance because there is no orientation: the
// LSM6DSOX does no fusion, so /imu sets orientation_covariance[0] = -1,
// which is how sensor_msgs/Imu says "no orientation estimate". Fuse on the
// host (imu_filter_madgwick, robot_localization) from imu + imu/mag.
#define IMU_ANGULAR_VEL_COV  0.04     // (rad/s)^2
#define IMU_LINEAR_ACC_COV   0.017    // (m/s^2)^2
#define IMU_MAGNETIC_COV     0.0      // tesla^2 (0 = unknown)

// ===========================================================================
//  Status LED
// ===========================================================================
//
// The six-pixel strip says what the firmware is doing, from across the room:
//
//   fast yellow blink   waiting for the ROS router -- nothing else can start
//   slow green breath   connected, running
//
// Either way it is driven from the main loop, so a strip that stops moving
// means a loop that stopped turning. That is half the point of it.

#define LED_ENABLED     true

// Waiting: a blink, not a breath. It should read as "waiting on you".
#define LED_WAITING_PERIOD_MS 400    // one on/off cycle
#define LED_WAITING_R         40     // yellow, weighted warm -- the green
#define LED_WAITING_G         26     // die is the brighter of the two

// Connected: a slow breath, calm enough to ignore.
#define LED_READY_PERIOD_MS   3000   // one full breath, in and out
#define LED_MIN               3      // channel value at the dimmest point
#define LED_MAX               40     // and at the brightest (kept low on purpose)

#endif // ROBOT_H

#include "ros.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "cdc_serial.h"
#include "easypicoros.h"
#include "serial_hook.h"
#include "imu.h"
#include "link101/lsm6dsox.h"
#include "io.h"
#include "robot.h"
#include "status.h"
#include "usb_descriptors.h"
#include "wheels.h"
#include "time_sync.h"
#include "odometry.h"

static cdc_serial_t zenoh_cdc;
static serial_hook_t zenoh_hook;
static easyp_publisher_t *imu_pub;
static easyp_publisher_t *imu_mag_pub;
static easyp_publisher_t *imu_temp_pub;
static easyp_publisher_t *imu_status_pub;
static uint32_t imu_publish_failures;
static easyp_publisher_t *odom_pub;
static odometry_t odom;
static imu_sample_t latest_imu;
static uint64_t latest_imu_us;
static bool latest_imu_valid;

static void on_odom_reset(void *msg, void *unused) {
    (void)unused;
    if (*(const ros_Bool *)msg) odometry_reset_pose(&odom);
}

// Sampling continues independently of ROS clock synchronization and publication.
static void sample_imu(void) {
    static uint64_t next_sample_us;
    uint64_t now = time_us_64();
    if (now < next_sample_us) return;
    next_sample_us = now + 1000000u / IMU_SAMPLE_HZ;
    latest_imu_valid = imu_read(&latest_imu);
    latest_imu_us = time_us_64();
    wheels_observe_gyro(&latest_imu, latest_imu_us, latest_imu_valid);
    double yaw_rate;
    if (latest_imu_valid) {
        if (wheels_yaw_rate(latest_imu_us, &yaw_rate)) latest_imu.gyro[2] = yaw_rate;
        else latest_imu_valid = false; // Do not publish uncalibrated gyro data.
    }
    wheels_yaw_status_t yaw;
    wheels_get_yaw_status(latest_imu_us, &yaw);
    status_set_mode(yaw.calibrated && yaw.fresh ? STATUS_READY : STATUS_WAITING);
}

// Raw odometry uses only the front encoder pair. The host EKF independently
// fuses this with /link101/imu, so gyro data must not enter this estimate.
static bool odometry_inputs(uint64_t now, double *vx, double *wz) {
    return wheels_odometry_twist(now, vx, wz);
}

static void update_odometry(void) {
    uint64_t now = time_us_64();
    double vx = 0, wz = 0;
    bool valid = odometry_inputs(now, &vx, &wz);
    odometry_update(&odom, now, vx, wz, valid, ODOM_MAX_DT_US);
}

static time_sync_t ros_clock;
static easyp_publisher_t *time_sync_pub;
static int64_t time_sync_values[3];
static ros_Int64MultiArray time_sync_response = {
    .data = { .data = time_sync_values, .n_elements = 3 },
};
static bool host_seen;
static uint64_t last_host_response_us;

static void on_time_sync(void *msg, void *unused) {
    (void)unused;
    const ros_Int64MultiArray *response = msg;
    if (response->data.n_deserialized != 3 || response->layout.data_offset != 0) {
        return;
    }
    uint64_t now = time_us_64();
    if (time_sync_accept(&ros_clock, response->data.data[0],
                         response->data.data[1], response->data.data[2], now)) {
        host_seen = true;
        last_host_response_us = now;
    }
}

static void time_sync_update(void) {
    static uint64_t next_request_us;
    uint64_t now = time_us_64();
    if (now < next_request_us) {
        return;
    }
    next_request_us = now + 1000000u / TIME_SYNC_HZ;
    ros_UInt64 request = now;
    time_sync_start(&ros_clock, now);
    easyp_publish(time_sync_pub, &request);
}

static uint64_t last_cmd_vel_us;
static bool have_cmd_vel;

static void on_gyro_calibrate(void *msg, void *unused) {
    (void)unused;
    if (*(const ros_Bool *)msg) {
        wheels_restart_gyro_calibration();
        have_cmd_vel = false;
    }
}

static void brake_wheels(void) {
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        wheels_brake(i);
    }
}

static void restart_for_connection_loss(const char *reason) {
    brake_wheels();
    status_set_mode(STATUS_WAITING);
    status_printf("[ros  ] %s; braking and rebooting for a clean session\n", reason);
    // Give the four brake frames and status message a chance to leave before
    // the RP2350 watchdog resets USB and the Zenoh client.
    absolute_time_t until = make_timeout_time_ms(50);
    while (!time_reached(until)) io_poll();
    watchdog_reboot(0, 0, 0);
    while (true) tight_loop_contents();
}

static void on_cmd_vel(void *msg, void *unused) {
    (void)unused;
    const ros_TwistStamped *cmd = msg;
    wheels_yaw_status_t yaw;
    wheels_get_yaw_status(time_us_64(), &yaw);
    if (!yaw.calibrated) { have_cmd_vel = false; return; } // Never queue startup motion.
    if (!wheels_set_velocity(cmd->twist.linear.x, cmd->twist.angular.z)) {
        // A malformed command must not keep the last valid motion alive.
        have_cmd_vel = false;
        brake_wheels();
        return;
    }
    // Motion timeout uses monotonic reception time, independent of ROS clock
    // synchronization, expiry or host clock adjustments.
    last_cmd_vel_us = time_us_64();
    have_cmd_vel = true;
}

// Normal zero commands use software shaping; stale commands brake immediately.
// Check freshness each loop, repeating brakes at 5 Hz while stale.
static void cmd_vel_watchdog(void) {
    static uint64_t next_brake_us;
    uint64_t now = time_us_64();
    if (have_cmd_vel &&
        now - last_cmd_vel_us < (uint64_t)COMMAND_TIMEOUT_MS * 1000u) {
        next_brake_us = 0;
        return;
    }
    if (now >= next_brake_us) {
        brake_wheels();
        next_brake_us = now + 1000000u / COMMAND_WATCHDOG_HZ;
    }
}

static bool stamp_at(uint64_t monotonic_us, ros_Time *stamp) {
    int64_t ns;
    if (!time_sync_time(&ros_clock, monotonic_us, &ns)) {
        return false;
    }
    *stamp = (ros_Time){
        .sec = (int32_t)(ns / INT64_C(1000000000)),
        .nanosec = (uint32_t)(ns % INT64_C(1000000000)),
    };
    return true;
}

static bool now_stamp(ros_Time *stamp) {
    return stamp_at(time_us_64(), stamp);
}

static void publish_odometry(void) {
    ros_Time stamp;
    uint64_t now = time_us_64();
    double measured_v, measured_w;
    if (!odom.valid || now < odom.sample_us || now - odom.sample_us > ODOM_MAX_DT_US ||
        !odometry_inputs(now, &measured_v, &measured_w) || !stamp_at(odom.sample_us, &stamp)) return;
    ros_Odometry msg = {0};
    msg.header.stamp = stamp;
    msg.header.frame_id = ODOM_FRAME_ID;
    msg.child_frame_id = ODOM_CHILD_FRAME_ID;
    msg.pose.pose.position.x = odom.x;
    msg.pose.pose.position.y = odom.y;
    msg.pose.pose.orientation.z = sin(odom.yaw * 0.5);
    msg.pose.pose.orientation.w = cos(odom.yaw * 0.5);
    msg.twist.twist.linear.x = odom.vx;
    msg.twist.twist.angular.z = odom.wz;
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05;
    msg.pose.covariance[35] = 0.1; // Wheel yaw remains sensitive to skid.
    msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e6;
    msg.twist.covariance[0] = 0.01;
    msg.twist.covariance[35] = 0.05;
    msg.twist.covariance[7] = msg.twist.covariance[14] =
        msg.twist.covariance[21] = msg.twist.covariance[28] = 1e6;
    easyp_publish(odom_pub, &msg);
}

// The two chips are independent, so this publishes whatever answered: the
// IMU feeds imu and imu/temperature, the magnetometer feeds imu/mag.
static void publish_imu(void) {
    ros_Time stamp;
    // Never label boot-relative time as ROS time. Resume automatically after
    // the host helper supplies a fresh, bounded-delay synchronization sample.
    if (!now_stamp(&stamp)) {
        return;
    }
    imu_sample_t sample;

    double calibrated_yaw;
    if (latest_imu_valid && wheels_yaw_rate(latest_imu_us, &calibrated_yaw) &&
        time_us_64() - latest_imu_us <= ODOM_MAX_DT_US &&
        stamp_at(latest_imu_us, &stamp)) {
        sample = latest_imu;
        ros_Imu imu;
        memset(&imu, 0, sizeof(imu));
        imu.header.stamp    = stamp;
        imu.header.frame_id = IMU_FRAME_ID;
        imu.angular_velocity.x = sample.gyro[0];
        imu.angular_velocity.y = sample.gyro[1];
        imu.angular_velocity.z = sample.gyro[2];
        imu.linear_acceleration.x = sample.accel[0];
        imu.linear_acceleration.y = sample.accel[1];
        imu.linear_acceleration.z = sample.accel[2];

        // -1 in the first element is how sensor_msgs/Imu says "there is no
        // orientation estimate in this message" -- the LSM6DSOX measures,
        // it does not fuse. The quaternion is left zeroed; consumers are
        // required to check this first and ignore it.
        imu.orientation_covariance[0] = -1.0;

        // Diagonal only: neither chip reports per-axis variance.
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] =
            imu.angular_velocity_covariance[8] = IMU_ANGULAR_VEL_COV;
        imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] =
            imu.linear_acceleration_covariance[8] = IMU_LINEAR_ACC_COV;
        if (!easyp_publish(imu_pub, &imu)) {
            imu_publish_failures++;
        }

        ros_Temperature temp;
        memset(&temp, 0, sizeof(temp));
        temp.header.stamp    = stamp;
        temp.header.frame_id = IMU_FRAME_ID;
        temp.temperature = (double)sample.temp_c;
        temp.variance    = 0.0;
        easyp_publish(imu_temp_pub, &temp);
    }

    float field[3];
    if (imu_read_magnetic_field(field) && now_stamp(&stamp)) {
        ros_MagneticField mag;
        memset(&mag, 0, sizeof(mag));
        mag.header.stamp    = stamp;
        mag.header.frame_id = IMU_FRAME_ID;
        mag.magnetic_field.x = field[0];
        mag.magnetic_field.y = field[1];
        mag.magnetic_field.z = field[2];
        mag.magnetic_field_covariance[0] = mag.magnetic_field_covariance[4] =
            mag.magnetic_field_covariance[8] = IMU_MAGNETIC_COV;
        easyp_publish(imu_mag_pub, &mag);
    }
}

// Publish health even before clock synchronization, without a stamped header.
static void publish_imu_status(void) {
    imu_health_t health;
    imu_get_health(&health);
    int64_t stamp;
    bool synced = time_sync_time(&ros_clock, time_us_64(), &stamp);
    wheels_yaw_status_t yaw;
    wheels_get_yaw_status(time_us_64(), &yaw);
    char text[640];
    snprintf(text, sizeof(text),
             "lsm6dsox=%s mag=%s sync=%s "
             "0x%02X_ack=%u who=0x%02X 0x%02X_ack=%u who=0x%02X "
             "reads_ok=%lu reads_failed=%lu temp_failed=%lu imu_publish_failed=%lu init_attempts=%lu "
             "yaw_cal=%s gyro_fresh=%u bias=%.6f yaw_rate=%.5f cal_samples=%u cal_s=%.1f cal_rejected=%u yaw_corr=%.5f saturated=%u yaw_feedback=%u",
             health.sensor_online ? "online" : "offline",
             health.mag_online ? "online" : "offline", synced ? "valid" : "waiting",
             IMU_ADDR, health.primary_ack, health.primary_id,
             LINK101_LSM6DSOX_ADDR_ALT, health.alternate_ack, health.alternate_id,
             (unsigned long)health.read_successes, (unsigned long)health.read_failures,
             (unsigned long)health.temperature_failures, (unsigned long)imu_publish_failures,
             (unsigned long)health.init_attempts,
             yaw.calibrated ? "ready" : "keep_still", yaw.fresh, yaw.bias, yaw.yaw_rate,
             yaw.samples, yaw.elapsed_us/1e6, yaw.rejected_windows, yaw.correction, yaw.saturated, GLIDE_YAW_FEEDBACK_ENABLED);
    ros_String message = { .data = text };
    easyp_publish(imu_status_pub, &message);
}

bool ros_begin(void) {
    // The port zenoh talks over, wrapped so USB keeps running
    // while it waits -- including the wait below for a router that may not
    // be up yet.
    serial_t *link = serial_hook_init(&zenoh_hook,
                                      cdc_serial_init(&zenoh_cdc, CDC_IDX_ZENOH),
                                      io_poll);

    status_printf("[ros  ] connecting to the router on '%s'...\n", ZENOH_LOCATOR);
    easyp_node_config_t node = { .name = NODE_NAME, .domain_id = ROS_DOMAIN_ID };
    if (!easyp_node_init(node, ZENOH_MODE, ZENOH_LOCATOR, link)) {
        return false;
    }
    status_printf("[ros  ] session up; declaring '%s'\n", NODE_NAME);

    time_sync_init(&ros_clock, TIME_SYNC_MAX_RTT_US,
                   (uint64_t)TIME_SYNC_TIMEOUT_MS * 1000u);
    time_sync_pub = easyp_publisher_create(TOPIC_TIME_SYNC_REQUEST, EASYP_TYPE(ros_UInt64));
    if (!time_sync_pub ||
        !easyp_subscriber_create_into(TOPIC_TIME_SYNC_RESPONSE,
                                      EASYP_TYPE(ros_Int64MultiArray),
                                      on_time_sync, NULL, &time_sync_response)) {
        return false;
    }
    odom_pub = easyp_publisher_create(TOPIC_ODOM, EASYP_TYPE(ros_Odometry));
    if (!odom_pub ||
        !easyp_subscriber_create(TOPIC_ODOM_RESET, EASYP_TYPE(ros_Bool), on_odom_reset, NULL)) return false;
    imu_pub = easyp_publisher_create(TOPIC_IMU, EASYP_TYPE(ros_Imu));
    imu_mag_pub = easyp_publisher_create(TOPIC_IMU_MAG, EASYP_TYPE(ros_MagneticField));
    imu_temp_pub = easyp_publisher_create(TOPIC_IMU_TEMP, EASYP_TYPE(ros_Temperature));
    imu_status_pub = easyp_publisher_create(TOPIC_IMU_STATUS, EASYP_TYPE(ros_String));
    if (!imu_pub || !imu_mag_pub || !imu_temp_pub || !imu_status_pub) {
        return false;
    }
    if (!easyp_subscriber_create(TOPIC_GYRO_CALIBRATE, EASYP_TYPE(ros_Bool),
                                on_gyro_calibrate, NULL)) return false;
    if (!easyp_subscriber_create(TOPIC_CMD_VEL, EASYP_TYPE(ros_TwistStamped),
                                on_cmd_vel, NULL)) {
        return false;
    }
    status_printf("[ros  ] cmd_vel drive control ready, IMU %s\n",
                  imu_online() ? "online" : "offline");
    return true;
}

void ros_update(void) {
    static uint64_t imu_due = 0;
    static uint64_t imu_status_due = 0;
    static uint64_t odom_due = 0;
    // A transient serial error is not sufficient evidence for rebooting: USB
    // and Docker can briefly disappear during startup. The accepted host
    // heartbeat below is the sole restart authority.
    easyp_spin_once();
    cmd_vel_watchdog();
    time_sync_update();
    sample_imu();
    update_odometry();
    uint64_t now = time_us_64();
    if (host_seen && now - last_host_response_us >=
                         (uint64_t)ROS_HOST_RESTART_TIMEOUT_MS * 1000u) {
        restart_for_connection_loss("host heartbeat expired");
    }
    if (now >= imu_status_due) {
        imu_status_due = now + 1000000u / IMU_STATUS_HZ;
        publish_imu_status();
    }
    if (now >= odom_due) {
        odom_due = now + 1000000u / ODOM_HZ;
        publish_odometry();
    }
    if (now >= imu_due) {
        imu_due = now + 1000000u / IMU_HZ;
        publish_imu();
    }
}

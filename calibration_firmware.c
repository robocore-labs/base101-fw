/* Standalone manual calibration image. No ROS transport or yaw PI. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "tusb.h"
#include "robot.h"
#include "io.h"
#include "status.h"
#include "wheels.h"
#include "imu.h"
#include "odometry.h"
#include "calibration_protocol.h"
#include "calibration_metrics.h"

static imu_sample_t sample;
static bool sample_valid;
static uint64_t sample_us;
static odometry_t odom;
static calibration_metrics_t metrics;
static calibration_parameters_t telemetry_parameters;
static void brake_all(void) {
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) wheels_brake(i);
}
static void observe_state(calibration_protocol_t *s, uint64_t now) {
    wheels_drive_status_t d; wheels_yaw_status_t y;
    wheels_get_drive_status(now, &d); wheels_get_yaw_status(now, &y);
    bool fresh = true;
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) fresh = fresh && d.fresh[i];
    s->ready = y.fresh && fresh && !s->fault;
    s->at_rest = d.at_rest && !d.motion_enabled;
}
static void add_telemetry(char *reply, unsigned capacity, uint64_t now) {
    wheels_drive_status_t d; wheels_yaw_status_t y; imu_health_t health;
    wheels_get_drive_status(now, &d); wheels_get_yaw_status(now, &y); imu_get_health(&health);
    double vx = 0, wz = 0;
    bool measured_valid = wheels_measured_twist(now, &vx, &wz);
    unsigned len = (unsigned)strlen(reply);
    int written = snprintf(reply + len - 2, capacity - len + 2,
        ",\"brake_ok_mask\":%u,\"yaw_feedback\":%s,"
        "\"gyro\":{\"calibrated\":%s,\"fresh\":%s,\"elapsed_ms\":%llu,\"duration_ms\":15000,"
        "\"samples\":%u,\"rejected\":%u,\"bias\":%.7f,\"rate\":%.7f},"
        "\"imu\":{\"online\":%s,\"valid\":%s,\"sample_us\":%llu,\"raw_z\":%.7f,"
        "\"gyro\":[%.7f,%.7f,%.7f],\"accel\":[%.7f,%.7f,%.7f],\"temperature\":%.5f,\"reads_ok\":%u,\"reads_failed\":%u},"
        "\"motion\":{\"requested\":[%.7f,%.7f],\"shaped\":[%.7f,%.7f],"
        "\"measured\":[%.7f,%.7f],\"measured_valid\":%s},"
        "\"geometry\":{\"radius\":%.7f,\"separation\":%.7f,\"gyro_z_sign\":%.7f},"
        "\"wheels\":[",
        d.brake_ok_mask, telemetry_parameters.yaw_feedback ? "true" : "false", y.calibrated ? "true" : "false", y.fresh ? "true" : "false",
        (unsigned long long)(y.elapsed_us / 1000), y.samples, y.rejected_windows, y.bias, y.yaw_rate,
        health.sensor_online ? "true" : "false", sample_valid ? "true" : "false",
        (unsigned long long)sample_us, sample.gyro[2], sample.gyro[0], sample.gyro[1], sample.gyro[2], sample.accel[0], sample.accel[1], sample.accel[2],
        sample.temp_c, (unsigned)health.read_successes, (unsigned)health.read_failures,
        d.requested_vx, d.requested_wz, d.shaped_vx, d.shaped_wz, vx, wz,
        measured_valid ? "true" : "false", telemetry_parameters.radius ? telemetry_parameters.radius : WHEEL_RADIUS_M, telemetry_parameters.separation ? telemetry_parameters.separation : WHEEL_SEPARATION_M, GYRO_Z_SIGN);
    if (written < 0 || (unsigned)written >= capacity - len + 2) { reply[0] = 0; return; }
    len = (unsigned)strlen(reply);
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        written = snprintf(reply + len, capacity - len,
            "%s{\"online\":%s,\"fresh\":%s,\"sent\":%.6f,\"rpm\":%.6f,\"sample_us\":%llu,\"error\":%u,\"temperature\":%u}",
            i ? "," : "", d.online[i] ? "true" : "false", d.fresh[i] ? "true" : "false",
            d.commanded_rpm[i], d.measured_rpm[i], (unsigned long long)d.sample_us[i],
            (unsigned)d.error[i], (unsigned)d.temperature[i]);
        if (written < 0 || (unsigned)written >= capacity - len) { reply[0] = 0; return; }
        len += (unsigned)written;
    }
    written = snprintf(reply + len, capacity - len,
        "],\"metrics\":{\"samples\":%u,\"gaps\":%u,\"knocks\":%u,\"gyro_yaw\":%.7f,\"encoder_yaw\":%.7f,"
        "\"distance\":%.7f,\"steady_gyro\":%.7f,\"steady_encoder\":%.7f,\"steady_s\":%.7f,"
        "\"accel_peak\":%.7f,\"jerk_peak\":%.7f,\"accel_rms\":%.7f,\"tracking_rms\":%.7f,"
        "\"response_s\":%.7f,\"stop_distance\":%.7f,\"stop_s\":%.7f},"
        "\"odom\":{\"valid\":%s,\"x\":%.7f,\"y\":%.7f,\"yaw\":%.7f}}\n",
        metrics.samples, metrics.gaps, metrics.knocks, metrics.gyro_yaw, metrics.encoder_yaw,
        metrics.distance, metrics.steady_gyro, metrics.steady_encoder, metrics.steady_s,
        metrics.accel_peak, metrics.jerk_peak,
        metrics.elapsed_s>0?sqrt(metrics.accel_energy/metrics.elapsed_s):0,
        metrics.tracking_s>0?sqrt(metrics.tracking_energy/metrics.tracking_s):0,
        metrics.response_s,metrics.stop_distance,metrics.stop_s,
        odom.valid ? "true" : "false", odom.x, odom.y, odom.yaw);
    if (written < 0 || (unsigned)written >= capacity - len) reply[0] = 0;
}
static void send_reply(const char *reply) {
    unsigned length = (unsigned)strlen(reply);
    if (length && tud_cdc_n_write_available(0) >= length) {
        tud_cdc_n_write(0, reply, length); tud_cdc_n_write_flush(0);
    }
}
int main(void) {
    tusb_init(); status_begin();
    wheels_begin(); brake_all(); imu_begin();
    calibration_protocol_t state;
    calibration_parameters_t startup;wheels_get_parameters(&startup);
    startup.vx_max=fmin(startup.vx_max,CALIBRATION_VX_MAX);
    startup.wz_max=fmin(startup.wz_max,CALIBRATION_WZ_MAX);
    wheels_configure_calibration(&startup);
    calibration_init(&state,startup);
    telemetry_parameters=state.parameters;
    char line[512], reply[4096]; unsigned used = 0;
    bool overflow = false, connected = false;
    uint64_t next_sample = 0, next_odom = 0, next_probe = 0;
    while (true) {
        io_poll();
        if (!state.driving && !state.releasing && time_us_64() >= next_probe) {
            wheels_retry_offline(); next_probe = time_us_64() + 1000000u;
        }
        imu_update();
        uint64_t now = time_us_64();
        if (now >= next_sample) {
            imu_sample_t current = {0};
            sample_valid = imu_read(&current);
            for (unsigned i = 0; i < 3; ++i)
                sample_valid = sample_valid && isfinite(current.gyro[i]) && isfinite(current.accel[i]);
            sample_valid = sample_valid && isfinite(current.temp_c);
            if (sample_valid) { sample = current; sample_us = time_us_64(); }
            wheels_observe_gyro(&current, time_us_64(), sample_valid);
            wheels_drive_status_t md; wheels_get_drive_status(time_us_64(),&md);
            double mv=0,mw=0,mg=0;
            bool valid=wheels_measured_twist(time_us_64(),&mv,&mw) && wheels_yaw_rate(time_us_64(),&mg);
            calibration_metrics_observe(&metrics,time_us_64(),current.accel,mg,mv,mw,
                md.requested_vx,md.requested_wz,md.shaped_vx,md.shaped_wz,state.parameters.k_icr,sample_valid && valid,md.motion_enabled);
            next_sample = time_us_64() + 1000000u / IMU_SAMPLE_HZ;
        }
        bool mounted = tud_mounted();
        if (connected && !mounted) {
            calibration_disconnect(&state); brake_all(); used = 0; overflow = false;
        }
        connected = mounted;
        if (calibration_expire(&state, time_us_64())) brake_all();
        // Bound input work so a noisy host cannot starve the watchdog or sensors.
        unsigned budget = 192;
        while (budget-- && tud_cdc_n_available(0)) {
            char ch; tud_cdc_n_read(0, &ch, 1);
            if (ch == '\n' || ch == '\r') {
                if (used || overflow) {
                    line[used] = 0;
                    if (overflow) {
                        calibration_stop(&state, "invalid_request"); brake_all();
                        snprintf(reply, sizeof(reply), "{\"id\":0,\"ok\":false,\"error\":\"line_too_long_or_invalid\"}\n");
                    } else {
                        observe_state(&state, time_us_64());
                        calibration_parameters_t previous = state.parameters;
                        bool brake = calibration_command(&state, line, time_us_64(), reply, sizeof(reply));
                        if (brake) brake_all();
                        bool applied = true;
                        if (state.action == CAL_MEASURE) {
                            calibration_metrics_begin(&metrics,time_us_64(),sample.accel);
                            metrics.body_response=state.measure_body;
                        }
                        if (state.action == CAL_GYRO) wheels_restart_gyro_calibration();
                        if (state.action == CAL_SET) {
                            calibration_parameters_t p = state.parameters;
                            applied = wheels_configure_calibration(&p);
                            if (applied && (p.radius != previous.radius || p.separation != previous.separation)) odom = (odometry_t){0};
                            if (!applied) state.parameters = previous;
                        }
                        if (state.action == CAL_DRIVE || state.action == CAL_RELEASE)
                            applied = wheels_set_velocity(state.vx, state.wz);
                        if (!applied) {
                            calibration_fault(&state, "hardware_rejected"); brake_all();
                            unsigned long id = strtoul(line, NULL, 10);
                            snprintf(reply, sizeof(reply), "{\"id\":%lu,\"ok\":false,\"error\":\"hardware_rejected\"}\n", id);
                        }
                    }
                    if (strstr(reply,"\"ok\":true")) {
                        observe_state(&state,time_us_64());
                        char current_status[32];
                        snprintf(current_status,sizeof(current_status),"%lu status",strtoul(line,NULL,10));
                        if (calibration_command(&state,current_status,time_us_64(),reply,sizeof(reply))) brake_all();
                    }
                    telemetry_parameters=state.parameters;
                    add_telemetry(reply, sizeof(reply), time_us_64()); send_reply(reply);
                }
                used = 0; overflow = false;
            } else if ((unsigned char)ch < 32 || (unsigned char)ch > 126) overflow = true;
            else if (used + 1 < sizeof(line)) line[used++] = ch;
            else overflow = true;
        }
        if (calibration_expire(&state, time_us_64())) brake_all();
        wheels_update();
        wheels_drive_status_t d; wheels_get_drive_status(time_us_64(), &d);
        if ((state.driving || state.releasing) && !d.motion_enabled) {
            calibration_fault(&state, "motor_or_gyro_fault"); brake_all();
        } else if (state.releasing && d.at_rest) {
            calibration_stop(&state, "released"); brake_all();
        }
        if (metrics.motion_start_us && !d.motion_enabled) metrics.active=false;
        observe_state(&state, time_us_64());
        if (time_us_64() >= next_odom) {
            double vx = 0, wz = 0;
            now = time_us_64();
            bool valid = wheels_measured_twist(now, &vx, &wz);
            odometry_update(&odom, now, vx, wz, valid, ODOM_MAX_DT_US);
            next_odom = now + 1000000u / ODOM_HZ;
        }
        status_set_mode(state.ready && state.session ? STATUS_READY : STATUS_WAITING);
        sleep_ms(1);
    }
}

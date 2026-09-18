#include "wheels.h"

#include <math.h>

#include "pico/stdlib.h"

#include "ddsm210.h"
#include "link101/pio_serial.h"
#include "serial_hook.h"

#include "io.h"
#include "robot.h"
#include "status.h"
#include "glide.h"
#include "gyro_yaw.h"
#ifdef BASE101_USE_CALIBRATION_HEADER
#include <calibration.h>
#endif

#define TWO_PI 6.283185307179586

static link101_pio_serial_t ports[WHEEL_COUNT];
static serial_hook_t        hooks[WHEEL_COUNT];
static serial_t            *bus[WHEEL_COUNT];
static bool                 online[WHEEL_COUNT];

// Shape in body space before allocation. Explicit braking disables motion
// and clears the profile; only a fresh body command can release it.
static double target[WHEEL_COUNT]; // latest shaped wheel rates, rad/s
static double requested_linear, requested_angular;
static glide_t controller;
static gyro_yaw_t gyro;
static gyro_yaw_config_t gyro_config = {
    .window_us = GYRO_CALIBRATION_US, .max_gap_us = GYRO_MAX_AGE_US,
    .min_samples = GYRO_CALIBRATION_MIN_SAMPLES,
    .max_bias = GYRO_CALIBRATION_MAX_BIAS,
    .max_stddev = GYRO_CALIBRATION_MAX_STDDEV, .lpf_hz = GLIDE_YAW_LPF_HZ,
};
static bool braked[WHEEL_COUNT];
static bool motion_enabled;
static uint8_t motor_ramp = WHEEL_ACCEL_TIME;
static uint64_t last_update_us, next_update_us;
static double measured[WHEEL_COUNT];
static uint64_t measured_us[WHEEL_COUNT];
static bool measured_valid[WHEEL_COUNT];
static uint8_t motor_error[WHEEL_COUNT], motor_temperature[WHEEL_COUNT];

static void save_feedback(uint8_t index, bool received, const ddsm210_feedback_t *fb) {
    if (received) { motor_error[index] = fb->error_code; motor_temperature[index] = fb->temperature; }
    measured_valid[index] = received && fb->error_code == 0;
    if (measured_valid[index]) {
        // Do not propagate signed zero from mirrored motors into telemetry.
        measured[index] = fb->feedback1 == 0 ? 0.0 :
                          fb->feedback1 * (TWO_PI / 600.0) *
                          WHEELS[index].direction * controller.cfg.radius;
        measured_us[index] = time_us_64();
    }
}

bool wheels_measured_twist(uint64_t now_us, double *linear_m_s, double *angular_rad_s) {
    double left = 0, right = 0;
    unsigned left_count = 0, right_count = 0;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!online[i] || !measured_valid[i] || now_us < measured_us[i] ||
            now_us - measured_us[i] > ODOM_FEEDBACK_MAX_AGE_US) return false;
        double velocity = measured[i] * ODOM_ENCODER_SIGN;
        if (WHEELS[i].left) { left += velocity; left_count++; }
        else { right += velocity; right_count++; }
    }
    if (!left_count || !right_count) return false;
    left /= left_count;
    right /= right_count;
    *linear_m_s = (left + right) * 0.5;
    *angular_rad_s = (right - left) / controller.cfg.separation;
    return true;
}

bool wheels_odometry_twist(uint64_t now_us, double *linear_m_s, double *angular_rad_s) {
    // WHEELS[0/1] are front-left/front-right; speeds already include mounting direction.
    for (uint8_t i = 0; i < 2; i++) {
        if (!online[i] || !measured_valid[i] || now_us < measured_us[i] ||
            now_us - measured_us[i] > ODOM_FEEDBACK_MAX_AGE_US) return false;
    }
    double left = measured[0] * ODOM_ENCODER_SIGN;
    double right = measured[1] * ODOM_ENCODER_SIGN;
    *linear_m_s = (left + right) * 0.5;
    *angular_rad_s = (right - left) / (controller.cfg.separation * controller.cfg.icr);
    return true;
}

bool wheels_yaw_rate(uint64_t now_us, double *rate) {
    return gyro_yaw_get(&gyro, now_us, rate);
}

void wheels_restart_gyro_calibration(void) {
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) wheels_brake(i);
    gyro_yaw_begin(&gyro, &gyro_config);
}

void wheels_get_yaw_status(uint64_t now_us, wheels_yaw_status_t *out) {
    double rate = 0;
    *out = (wheels_yaw_status_t){
        .calibrated = gyro.calibrated, .fresh = gyro_yaw_get(&gyro, now_us, &rate),
        .samples = gyro.samples, .rejected_windows = gyro.rejected_windows,
        .bias = gyro.bias, .correction = controller.correction,
        .saturated = controller.saturated,
        .elapsed_us = gyro.calibrated ? gyro.cfg.window_us :
                      (gyro.window_started && now_us >= gyro.window_start_us ?
                       now_us - gyro.window_start_us : 0),
    };
    out->yaw_rate = rate;
}

void wheels_observe_gyro(const imu_sample_t *sample, uint64_t now_us, bool valid) {
    bool stationary = valid;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!online[i] || !measured_valid[i] || now_us < measured_us[i] ||
            now_us - measured_us[i] > ODOM_FEEDBACK_MAX_AGE_US ||
            fabs(measured[i]) > .003) stationary = false;
    }
    double accel = sqrt(sample->accel[0]*sample->accel[0] +
                        sample->accel[1]*sample->accel[1] + sample->accel[2]*sample->accel[2]);
    for (unsigned i = 0; i < 3; i++)
        if (!isfinite(sample->gyro[i]) || fabs(sample->gyro[i]) > .1) stationary = false;
    if (!isfinite(accel) || fabs(accel - 9.80665) > 1) stationary = false;
    // Sign is measured separately: a stationary window cannot determine it.
    double signed_z = GYRO_Z_SIGN * sample->gyro[2];
    gyro_yaw_observe(&gyro, now_us, signed_z, valid && fabs(signed_z) < 5, stationary);
}

uint8_t wheels_begin(void) {
    uint8_t found = 0;
    const glide_config_t cfg = {
        .radius = WHEEL_RADIUS_M, .separation = WHEEL_SEPARATION_M, .icr = GLIDE_ICR_COEFF,
        .wheel_max = fmin(WHEEL_MAX_RPM, DDSM210_MAX_SPEED_RAW/10.0) *
                     TWO_PI/60.0/fabs(WHEEL_SPEED_SCALE),
        .vx_max = GLIDE_VX_MAX, .wz_max = GLIDE_WZ_MAX,
        .ax_max = DRIVE_LINEAR_ACCEL_M_S2, .jx_max = DRIVE_LINEAR_JERK_M_S3,
        .alpha_max = DRIVE_YAW_ACCEL_RAD_S2, .jalpha_max = DRIVE_YAW_JERK_RAD_S3,
        .yaw_kp = GLIDE_YAW_KP, .yaw_ki = GLIDE_YAW_KI,
        #ifdef CALIBRATION_FIRMWARE
        .yaw_feedback = false,
#else
        .yaw_feedback = GLIDE_YAW_FEEDBACK_ENABLED,
#endif
        .correction_accel = GLIDE_YAW_CORRECTION_ACCEL,
        .correction_jerk = GLIDE_YAW_CORRECTION_JERK,
        .correction_max = GLIDE_YAW_CORRECTION_MAX, .deadband = GLIDE_YAW_DEADBAND,
    };
    gyro_yaw_begin(&gyro, &gyro_config);
    if (!glide_init(&controller, &cfg)) return 0;
    #ifdef BASE101_USE_CALIBRATION_HEADER
    {
        const calibration_parameters_t saved=BASE101_CALIBRATION_PARAMETERS;
        glide_config_t loaded=cfg;
        loaded.ax_max=saved.ax; loaded.jx_max=saved.jx;
        loaded.alpha_max=saved.aw; loaded.jalpha_max=saved.jw;
        loaded.icr=saved.k_icr; loaded.radius=saved.radius; loaded.separation=saved.separation;
        loaded.wheel_max=saved.rpm_max*TWO_PI/60/fabs(WHEEL_SPEED_SCALE);
        loaded.vx_max=saved.vx_max; loaded.wz_max=saved.wz_max;
        loaded.yaw_kp=saved.kp; loaded.yaw_ki=saved.ki; loaded.yaw_feedback=saved.yaw_feedback!=0;
        loaded.correction_max=saved.correction_max; loaded.deadband=saved.deadband;
        loaded.correction_accel=saved.correction_accel; loaded.correction_jerk=saved.correction_jerk;
        if (!glide_init(&controller,&loaded)) return 0;
        motor_ramp=(uint8_t)saved.ramp;
        gyro_config.lpf_hz=saved.lpf;gyro_yaw_begin(&gyro,&gyro_config);
        status_printf("[wheel] using compiled calibration.h\n");
    }
    #endif

    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        const wheel_cfg_t *w = &WHEELS[i];

        // The port, wrapped so USB keeps running while the
        // driver waits for this motor to answer.
        bus[i] = serial_hook_init(&hooks[i],
                                  link101_pio_serial_init(&ports[i],
                                                          w->tx_pin, w->rx_pin,
                                                          LINK101_NO_TXEN, WHEEL_BAUD),
                                  io_poll);
        if (!bus[i]) {
            status_printf("[wheel] %s: no PIO state machine free\n", w->joint);
            continue;
        }

#ifdef CALIBRATION_FIRMWARE
        // Apply braking even before probing/configuring the motor.
        ddsm210_brake(bus[i], w->id);
#endif
        // Asking for its mode is the cheapest way to ask "are you there?".
        online[i] = ddsm210_get_mode(bus[i], w->id) >= 0;
        status_printf("[wheel] %s on GP%u/%u: %s\n", w->joint, w->tx_pin, w->rx_pin,
                      online[i] ? "online" : "no response");
        if (!online[i]) {
            continue;
        }

        braked[i] = true;
#ifdef CALIBRATION_FIRMWARE
        // Mode writes may have no reply; verify with a separate mode query.
        ddsm210_set_mode(bus[i], w->id, DDSM210_MODE_VELOCITY);
        sleep_ms(10);
        if (ddsm210_get_mode(bus[i], w->id) != DDSM210_MODE_VELOCITY) {
            ddsm210_brake(bus[i], w->id); online[i] = false; continue;
        }
#else
        ddsm210_set_mode(bus[i], w->id, DDSM210_MODE_VELOCITY);
#endif
        sleep_ms(10);
        found++;
    }
    return found;
}

bool wheels_online(uint8_t index) {
    return index < WHEEL_COUNT && online[index];
}

// Uniform saturation preserves curvature and bounds the requested body
// command before shaping. Also applied to shaped outputs at the wheel cap.
static bool allocate(double linear, double angular, double *left, double *right) {
    if (!isfinite(linear) || !isfinite(angular)) return false;
    double turn = angular * controller.cfg.separation * 0.5;
    *left = (linear - turn) / controller.cfg.radius;
    *right = (linear + turn) / controller.cfg.radius;
    if (!isfinite(*left) || !isfinite(*right)) return false;
    double limit = controller.cfg.wheel_max;
    double peak = fmax(fabs(*left), fabs(*right));
    if (peak > limit) {
        double scale = limit / peak;
        *left *= scale; *right *= scale;
    }
    return true;
}

bool wheels_set_velocity(double linear_m_s, double angular_rad_s) {
    double left, right, yaw_rate, measured_v, measured_w;
    uint64_t now = time_us_64();
    if (!wheels_yaw_rate(now, &yaw_rate) ||
        !wheels_measured_twist(now, &measured_v, &measured_w)) return false;
    if (!allocate(linear_m_s, angular_rad_s, &left, &right)) return false;
    requested_linear = (left + right) * controller.cfg.radius * 0.5;
    requested_angular = (right - left) * controller.cfg.radius / controller.cfg.separation;
    if (!motion_enabled) last_update_us = 0;
    motion_enabled = true;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) braked[i] = false;
    return true;
}

// Unit conversion + direction + cap, then the bus write for one wheel's
// setpoint. Zero stays in velocity mode, including during reversal.
// Electric braking is reserved for wheels_brake().
static bool send_speed(uint8_t index, double rad_per_sec) {
    if (!wheels_online(index)) {
        return false;
    }
    const wheel_cfg_t *w = &WHEELS[index];

    // rad/s at the wheel -> RPM -> the motor's 0.1 RPM units.
    double rpm = rad_per_sec * w->direction * WHEEL_SPEED_SCALE * 60.0 / TWO_PI;
    int32_t units = (int32_t)lround(rpm * 10.0);

    int32_t limit = (int32_t)lround(controller.cfg.wheel_max * fabs(WHEEL_SPEED_SCALE) * 600 / TWO_PI);
    if (limit > DDSM210_MAX_SPEED_RAW) {
        limit = DDSM210_MAX_SPEED_RAW;
    }
    if (units >  limit) units =  limit;
    if (units < -limit) units = -limit;

    ddsm210_feedback_t fb = {0};
    bool received = ddsm210_set_velocity(bus[index], w->id, (int16_t)units, motor_ramp, &fb);
    save_feedback(index, received, &fb);
    return measured_valid[index];
}

void wheels_update(void) {
    uint64_t now = time_us_64();
    if (now < next_update_us) return;
    if (!motion_enabled) {
        next_update_us = now + 1000000u / WHEEL_CONTROL_HZ;
        for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
            if (!wheels_online(i)) continue;
            ddsm210_feedback_t fb = {0};
            bool received = ddsm210_brake_feedback(bus[i], WHEELS[i].id, &fb);
            save_feedback(i, received, &fb);
        }
        return;
    }
    if (last_update_us && now <= last_update_us) return;
    double dt = last_update_us ? (now - last_update_us) / 1000000.0 :
                               1.0 / WHEEL_CONTROL_HZ;
    last_update_us = now;
    next_update_us = now + 1000000u / WHEEL_CONTROL_HZ;
    bool idle = requested_linear == 0 && requested_angular == 0 &&
                fabs(controller.linear.velocity) < 1e-6 &&
                fabs(controller.linear.acceleration) < 1e-6 &&
                fabs(controller.yaw.velocity) < 1e-6 &&
                fabs(controller.yaw.acceleration) < 1e-6;
    if (idle) dt = 1.0 / WHEEL_CONTROL_HZ;
    double left, right, yaw_rate, measured_v, measured_w;
    bool tracking_good = true;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
        if (fabs(measured[i]/controller.cfg.radius - target[i]*WHEEL_SPEED_SCALE) >
            controller.cfg.wheel_max * .2) tracking_good = false;
    if (!wheels_yaw_rate(now, &yaw_rate) ||
        !wheels_measured_twist(now, &measured_v, &measured_w) ||
        !glide_update(&controller, requested_linear, requested_angular, yaw_rate,
                      tracking_good, dt, &left, &right)) {
        for (uint8_t i = 0; i < WHEEL_COUNT; i++) wheels_brake(i);
        return;
    }
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        target[i] = WHEELS[i].left ? left : right;
        if (wheels_online(i) && !braked[i] && !send_speed(i, target[i])) {
            for (uint8_t j = 0; j < WHEEL_COUNT; j++) wheels_brake(j);
            return;
        }
    }

}

void wheels_brake(uint8_t index) {
    if (index >= WHEEL_COUNT) return;
    motion_enabled = false;
    requested_linear = requested_angular = 0;
    glide_reset(&controller);
    last_update_us = 0;
    next_update_us = time_us_64() + 1000000u / WHEEL_CONTROL_HZ;
    if (!wheels_online(index)) {
        return;
    }
    target[index]  = 0.0;
    braked[index] = true;
    ddsm210_feedback_t fb = {0};
    bool received = ddsm210_brake_feedback(bus[index], WHEELS[index].id, &fb);
    save_feedback(index, received, &fb);
}

double wheels_read_angle(uint8_t index) {
    if (!wheels_online(index)) {
        return 0.0;
    }

    ddsm210_odometry_t odom;
    if (!ddsm210_get_odometry(bus[index], WHEELS[index].id, &odom)) {
        return 0.0;
    }

    // Absolute angle = whole turns since boot + where in this turn we are.
    double turns    = (double)odom.mileage_laps;
    double fraction = (double)odom.position / DDSM210_ENCODER_TICKS;
    return (turns + fraction) * TWO_PI * WHEELS[index].direction;
}

void wheels_get_drive_status(uint64_t now_us, wheels_drive_status_t *out) {
    *out = (wheels_drive_status_t){.motion_enabled = motion_enabled,
        .at_rest = true, .requested_vx = requested_linear, .requested_wz = requested_angular,
        .shaped_vx = controller.linear.velocity, .shaped_wz = controller.yaw.velocity};
    if (fabs(controller.linear.velocity) > 1e-4 || fabs(controller.yaw.velocity) > 1e-4 ||
        fabs(controller.linear.acceleration) > 1e-3 || fabs(controller.yaw.acceleration) > 1e-3)
        out->at_rest = false;
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        out->online[i] = online[i];
        out->sample_us[i] = measured_us[i];
        out->error[i] = motor_error[i]; out->temperature[i] = motor_temperature[i];
        out->fresh[i] = online[i] && measured_valid[i] && now_us >= measured_us[i] &&
            now_us - measured_us[i] <= ODOM_FEEDBACK_MAX_AGE_US;
        out->commanded_rpm[i] = target[i] * WHEEL_SPEED_SCALE * 60 / TWO_PI;
        out->measured_rpm[i] = measured[i] / controller.cfg.radius * 60 / TWO_PI;
        if (braked[i] && out->fresh[i]) out->brake_ok_mask |= 1u << i;
        if (!out->fresh[i] || fabs(measured[i]) > .003) out->at_rest = false;
    }
}

bool wheels_configure_motion(double ax, double jx, double aw, double jw, double icr) {
    wheels_drive_status_t status;
    wheels_get_drive_status(time_us_64(), &status);
    if (!status.at_rest || status.motion_enabled) return false;
    glide_config_t cfg = controller.cfg;
    cfg.ax_max = ax; cfg.jx_max = jx; cfg.alpha_max = aw; cfg.jalpha_max = jw; cfg.icr = icr;
    return glide_init(&controller, &cfg);
}

void wheels_get_parameters(calibration_parameters_t *p) {
    *p=(calibration_parameters_t){.ax=controller.cfg.ax_max,.jx=controller.cfg.jx_max,
        .aw=controller.cfg.alpha_max,.jw=controller.cfg.jalpha_max,.k_icr=controller.cfg.icr,
        .radius=controller.cfg.radius,.separation=controller.cfg.separation,.ramp=motor_ramp,
        .rpm_max=round(controller.cfg.wheel_max*fabs(WHEEL_SPEED_SCALE)*60/TWO_PI),
        .vx_max=controller.cfg.vx_max,.wz_max=controller.cfg.wz_max,
        .lpf=gyro_config.lpf_hz,.kp=controller.cfg.yaw_kp,.ki=controller.cfg.yaw_ki,
        .correction_max=controller.cfg.correction_max,.deadband=controller.cfg.deadband,
        .correction_accel=controller.cfg.correction_accel,.correction_jerk=controller.cfg.correction_jerk,
        .yaw_feedback=controller.cfg.yaw_feedback};
}
bool wheels_configure_calibration(const calibration_parameters_t *p) {
    wheels_drive_status_t status; wheels_get_drive_status(time_us_64(), &status);
    if (!status.at_rest || status.motion_enabled) return false;
    glide_config_t cfg = controller.cfg;
    cfg.ax_max=p->ax; cfg.jx_max=p->jx; cfg.alpha_max=p->aw; cfg.jalpha_max=p->jw;
    cfg.icr=p->k_icr; cfg.radius=p->radius; cfg.separation=p->separation;
    cfg.wheel_max=p->rpm_max*TWO_PI/60/fabs(WHEEL_SPEED_SCALE);
    cfg.vx_max=p->vx_max; cfg.wz_max=p->wz_max;
    cfg.yaw_kp=p->kp; cfg.yaw_ki=p->ki; cfg.yaw_feedback=p->yaw_feedback != 0;
    cfg.correction_max=p->correction_max; cfg.deadband=p->deadband;
    cfg.correction_accel=p->correction_accel; cfg.correction_jerk=p->correction_jerk;
    if (!isfinite(p->rpm_max) || p->rpm_max<20 || p->rpm_max>200 ||
        !isfinite(p->vx_max) || p->vx_max<.05 || p->vx_max>.4 ||
        !isfinite(p->wz_max) || p->wz_max<.1 || p->wz_max>.8 ||
        !isfinite(p->yaw_feedback) || (p->yaw_feedback!=0 && p->yaw_feedback!=1) ||
        !isfinite(p->ramp) || p->ramp < 1 || p->ramp > 255 || floor(p->ramp)!=p->ramp ||
        !isfinite(p->lpf) || p->lpf < 1 || p->lpf > 80) return false;
    double old_radius=controller.cfg.radius;
    if (!glide_init(&controller,&cfg)) return false;
    for (unsigned i=0;i<WHEEL_COUNT;i++) measured[i] *= cfg.radius/old_radius;
    motor_ramp=(uint8_t)p->ramp;
    if (gyro_config.lpf_hz != p->lpf) {
        gyro_config.lpf_hz=p->lpf; wheels_restart_gyro_calibration();
    }
    return true;
}

#ifdef CALIBRATION_FIRMWARE
void wheels_retry_offline(void) {
    if (motion_enabled) return;
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        if (online[i] || !bus[i]) continue;
        ddsm210_brake(bus[i], WHEELS[i].id);
        if (ddsm210_get_mode(bus[i], WHEELS[i].id) < 0) continue;
        ddsm210_set_mode(bus[i], WHEELS[i].id, DDSM210_MODE_VELOCITY);
        sleep_ms(10);
        online[i] = ddsm210_get_mode(bus[i], WHEELS[i].id) == DDSM210_MODE_VELOCITY;
        braked[i] = true;
        ddsm210_feedback_t fb = {0};
        bool received = ddsm210_brake_feedback(bus[i], WHEELS[i].id, &fb);
        save_feedback(i, received, &fb);
    }
}
#endif

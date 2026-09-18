#include "calibration_protocol.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void calibration_init(calibration_protocol_t *s, calibration_parameters_t defaults) {
    if (defaults.radius == 0) {
        defaults.radius = .0363; defaults.separation = .2899; defaults.ramp = 1;
        defaults.rpm_max = 200; defaults.vx_max = .4; defaults.wz_max = .8;
        defaults.lpf = 20; defaults.kp = .5; defaults.ki = .3;
        defaults.correction_max = .5; defaults.deadband = .005;
        defaults.correction_accel = .25; defaults.correction_jerk = 1;
    }
    *s = (calibration_protocol_t){.parameters = defaults, .stop_reason = "boot", .measure_body=true};
}
void calibration_stop(calibration_protocol_t *s, const char *reason) {
    s->driving = s->releasing = false; s->bound_until_us = 0;
    s->vx = s->wz = 0;
    s->stop_reason = reason;
}
void calibration_fault(calibration_protocol_t *s, const char *reason) {
    calibration_stop(s, reason); s->fault = true; s->ready = false;
}
void calibration_disconnect(calibration_protocol_t *s) {
    s->session = false; calibration_stop(s, "disconnect");
}
bool calibration_expire(calibration_protocol_t *s, uint64_t now) {
    if (s->bound_until_us && now >= s->bound_until_us) {
        calibration_stop(s, "trial_deadline"); return true;
    }
    if (s->session && now - s->heartbeat_us >= CALIBRATION_HEARTBEAT_US) {
        s->session = false; calibration_stop(s, "heartbeat_timeout"); return true;
    }
    if (s->driving && now - s->operator_us >= CALIBRATION_OPERATOR_US) {
        calibration_stop(s, "operator_timeout"); return true;
    }
    if (s->releasing && now >= s->release_until_us) {
        calibration_stop(s, "release_timeout"); return true;
    }
    return false;
}
static bool value(const char *word, double low, double high, double *out) {
    char *end;
    double v = strtod(word, &end);
    if (end == word || *end || !isfinite(v) || v < low || v > high) return false;
    *out = v; return true;
}
bool calibration_command(calibration_protocol_t *s, char *line, uint64_t now,
                         char *reply, unsigned capacity) {
    char *args[24], *word = strtok(line, " \t");
    unsigned n = 0;
    while (word && n < 24) { args[n++] = word; word = strtok(NULL, " \t"); }
    unsigned long id = 0;
    char *end;
    bool valid_id = false;
    if (n) {
        id = strtoul(args[0], &end, 10);
        valid_id = *args[0] && !*end && strspn(args[0], "0123456789") == strlen(args[0]) && id <= 2147483647u;
    }
    const char *error = NULL;
    bool brake = calibration_expire(s, now);
    s->action = CAL_NONE;
    if (!valid_id || n < 2 || word) error = "invalid_request";
    else if (!strcmp(args[1], "stop") && n == 2) {
        calibration_stop(s, "requested"); brake = true;
    } else if (!strcmp(args[1], "hello") && n == 3) {
        if (strcmp(args[2], "2")) error = "unsupported_protocol";
        else {
            s->session = true; s->heartbeat_us = now;
            calibration_stop(s, "handshake"); brake = true;
        }
    } else if (!strcmp(args[1], "status") && n == 2) {
        // Reading status does not renew either lease.
    } else if (!s->session) error = "handshake_required";
    else if (!strcmp(args[1], "heartbeat") && n == 2) s->heartbeat_us = now;
    else if (!strcmp(args[1], "calibrate") && n == 2) {
        calibration_stop(s, "calibrating"); s->fault = false; s->ready = false;
        s->action = CAL_GYRO; brake = true;
    } else if (!strcmp(args[1], "drive") && n == 4) {
        double vx, wz;
        if (!value(args[2], -CALIBRATION_VX_MAX, CALIBRATION_VX_MAX, &vx) ||
            !value(args[3], -CALIBRATION_WZ_MAX, CALIBRATION_WZ_MAX, &wz)) {
            error = "invalid_velocity"; calibration_stop(s, "invalid_velocity"); brake = true;
        } else if (!s->ready || s->fault) {
            error = "not_ready"; calibration_stop(s, "not_ready"); brake = true;
        } else {
            s->vx = vx; s->wz = wz; s->operator_us = now;
            s->driving = true; s->releasing = false; s->action = CAL_DRIVE;
            s->stop_reason = "driving";
        }
    } else if (!strcmp(args[1], "release") && n == 2) {
        if (s->driving || s->releasing) {
            s->driving = false; s->releasing = true; s->vx = s->wz = 0;
            s->release_until_us = now + CALIBRATION_RELEASE_US;
            s->stop_reason = "ramping_to_zero"; s->action = CAL_RELEASE;
        } else { calibration_stop(s, "released"); brake = true; }
    } else if (!strcmp(args[1], "set") && n == 7) {
        calibration_parameters_t p = s->parameters;
        if (!value(args[2], .01, 5, &p.ax) || !value(args[3], .01, 50, &p.jx) ||
            !value(args[4], .01, 20, &p.aw) || !value(args[5], .01, 200, &p.jw) ||
            !value(args[6], 1, 5, &p.k_icr)) error = "invalid_parameters";
        else if (!s->at_rest || s->driving || s->releasing) error = "not_at_rest";
        else { s->parameters = p; s->action = CAL_SET; }
    } else if (!strcmp(args[1], "measure") && (n == 2 || n == 3)) {
        if (!s->at_rest || s->driving || s->releasing) error = "not_at_rest";
        else if (n == 3 && strcmp(args[2],"0") && strcmp(args[2],"1")) error="invalid_measurement_mode";
        else { s->measure_body=n==2 || !strcmp(args[2],"1"); s->action = CAL_MEASURE; }
    } else if (!strcmp(args[1], "bound") && n == 3) {
        double ms;
        if (!value(args[2], 100, 30000, &ms)) error = "invalid_trial_duration";
        else if (!s->at_rest || s->driving || s->releasing) error = "not_at_rest";
        else { s->bound_until_us = now + (uint64_t)(ms * 1000); s->action = CAL_BOUND; }
    } else if (!strcmp(args[1], "config") && n == 21) {
        calibration_parameters_t p = s->parameters;
        if (!value(args[2], 0.01, 5, &p.ax) ||
            !value(args[3], 0.01, 50, &p.jx) ||
            !value(args[4], 0.01, 20, &p.aw) ||
            !value(args[5], 0.01, 200, &p.jw) ||
            !value(args[6], 1, 5, &p.k_icr) ||
            !value(args[7], 0.02, 0.1, &p.radius) ||
            !value(args[8], 0.15, 0.5, &p.separation) ||
            !value(args[9], 1, 255, &p.ramp) ||
            !value(args[10], 20, 200, &p.rpm_max) ||
            !value(args[11], 0.05, 0.4, &p.vx_max) ||
            !value(args[12], 0.1, 0.8, &p.wz_max) ||
            !value(args[13], 1, 80, &p.lpf) ||
            !value(args[14], 0, 2, &p.kp) ||
            !value(args[15], 0, 2, &p.ki) ||
            !value(args[16], 0, 1, &p.correction_max) ||
            !value(args[17], 0, 0.05, &p.deadband) ||
            !value(args[18], 0.01, 2, &p.correction_accel) ||
            !value(args[19], 0.01, 20, &p.correction_jerk) ||
            !value(args[20], 0, 1, &p.yaw_feedback)) error = "invalid_parameters";
        else if (floor(p.ramp) != p.ramp || floor(p.rpm_max) != p.rpm_max || floor(p.yaw_feedback) != p.yaw_feedback) error = "invalid_parameters";
        else if (!s->at_rest || s->driving || s->releasing) error = "not_at_rest";
        else { s->parameters = p; s->action = CAL_SET; }
    } else error = "unknown_command_or_arguments";
    if (error) snprintf(reply, capacity, "{\"id\":%lu,\"ok\":false,\"error\":\"%s\"}\n", valid_id ? id : 0, error);
    else snprintf(reply, capacity,
        "{\"id\":%lu,\"ok\":true,\"protocol\":2,\"firmware\":\"base101-calibration\","
        "\"motion_enabled\":%s,\"ready\":%s,\"at_rest\":%s,\"fault\":%s,\"session\":%s,"
        "\"heartbeat_timeout_ms\":500,\"operator_timeout_ms\":350,"
        "\"stop_reason\":\"%s\",\"mcu_us\":%llu,"
        "\"features\":[\"full_config\",\"metrics\",\"bounded_trial\"],"
        "\"parameters\":{\"ax\":%.6f,\"jx\":%.6f,\"aw\":%.6f,\"jw\":%.6f,\"k_icr\":%.6f,\"radius\":%.6f,\"separation\":%.6f,\"ramp\":%.6f,\"rpm_max\":%.6f,\"vx_max\":%.6f,\"wz_max\":%.6f,\"lpf\":%.6f,\"kp\":%.6f,\"ki\":%.6f,\"correction_max\":%.6f,\"deadband\":%.6f,\"correction_accel\":%.6f,\"correction_jerk\":%.6f,\"yaw_feedback\":%.6f}}\n",
        id, (s->driving || s->releasing) ? "true" : "false", s->ready ? "true" : "false",
        s->at_rest ? "true" : "false", s->fault ? "true" : "false", s->session ? "true" : "false",
        s->stop_reason, (unsigned long long)now,
        s->parameters.ax, s->parameters.jx, s->parameters.aw, s->parameters.jw, s->parameters.k_icr, s->parameters.radius, s->parameters.separation, s->parameters.ramp, s->parameters.rpm_max, s->parameters.vx_max, s->parameters.wz_max, s->parameters.lpf, s->parameters.kp, s->parameters.ki, s->parameters.correction_max, s->parameters.deadband, s->parameters.correction_accel, s->parameters.correction_jerk, s->parameters.yaw_feedback);
    return brake;
}

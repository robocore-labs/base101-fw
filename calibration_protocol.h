#ifndef CALIBRATION_PROTOCOL_H
#define CALIBRATION_PROTOCOL_H
#include <stdbool.h>
#include <stdint.h>
#define CALIBRATION_PROTOCOL_VERSION 2
#define CALIBRATION_HEARTBEAT_US 500000u
#define CALIBRATION_OPERATOR_US 350000u
#define CALIBRATION_VX_MAX .4
#define CALIBRATION_WZ_MAX .8
#define CALIBRATION_RELEASE_US 2000000u
typedef struct {
    double ax, jx, aw, jw, k_icr;
    double radius, separation, ramp, rpm_max, vx_max, wz_max;
    double lpf, kp, ki, correction_max, deadband, correction_accel, correction_jerk;
    double yaw_feedback; // 0 or 1, kept numeric for the positional wire format
} calibration_parameters_t;
typedef enum { CAL_NONE, CAL_DRIVE, CAL_RELEASE, CAL_SET, CAL_GYRO, CAL_MEASURE, CAL_BOUND } calibration_action_t;
typedef struct {
    calibration_parameters_t parameters;
    bool session, ready, at_rest, fault, driving, releasing, measure_body;
    uint64_t heartbeat_us, operator_us, release_until_us, bound_until_us;
    double vx, wz;
    const char *stop_reason;
    calibration_action_t action;
} calibration_protocol_t;
void calibration_init(calibration_protocol_t *s, calibration_parameters_t defaults);
// Returns true when hardware must brake before replying.
bool calibration_command(calibration_protocol_t *s, char *line, uint64_t now,
                         char *reply, unsigned capacity);
bool calibration_expire(calibration_protocol_t *s, uint64_t now);
void calibration_disconnect(calibration_protocol_t *s);
void calibration_stop(calibration_protocol_t *s, const char *reason);
void calibration_fault(calibration_protocol_t *s, const char *reason);
#endif

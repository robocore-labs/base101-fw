#ifndef BASE101_CALIBRATION_METRICS_H
#define BASE101_CALIBRATION_METRICS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t start_us, last_us, last_knock_us, motion_start_us;
    unsigned samples, gaps, knocks;
    bool active, previous_valid, previous_steady, body_response;
    double baseline[3], previous_accel[3], previous_gyro, previous_encoder, previous_vx, previous_shaped_vx;
    double gyro_yaw, encoder_yaw, distance, steady_gyro, steady_encoder, steady_s;
    double accel_peak, jerk_peak, accel_energy, tracking_energy, elapsed_s, tracking_s;
    double response_s, stop_distance, stop_s;
} calibration_metrics_t;
void calibration_metrics_begin(calibration_metrics_t *m, uint64_t now, const float accel[3]);
void calibration_metrics_observe(calibration_metrics_t *m, uint64_t now,
    const float accel[3], double gyro, double vx, double encoder,
    double requested_vx, double requested_wz, double shaped_vx, double shaped_wz,
    double icr, bool valid, bool moving);
#endif

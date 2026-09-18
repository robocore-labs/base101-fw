#ifndef BASE101_GYRO_YAW_H
#define BASE101_GYRO_YAW_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t window_us, max_gap_us;
    unsigned min_samples;
    double max_bias, max_stddev, lpf_hz;
} gyro_yaw_config_t;
typedef struct {
    gyro_yaw_config_t cfg;
    uint64_t window_start_us, last_sample_us;
    unsigned samples, rejected_windows;
    double mean, m2, bias, rate;
    bool calibrated, sample_valid, window_started;
} gyro_yaw_t;
void gyro_yaw_begin(gyro_yaw_t *g, const gyro_yaw_config_t *cfg);
// Input is signed body-Z raw gyro. Stationarity must also check wheels,
// acceleration magnitude and the other gyro axes. Motion/gaps restart the window.
void gyro_yaw_observe(gyro_yaw_t *g, uint64_t now_us, double raw_rate, bool valid, bool stationary);
bool gyro_yaw_get(const gyro_yaw_t *g, uint64_t now_us, double *rate);
#endif

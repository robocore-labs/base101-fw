#include "gyro_yaw.h"
#include <math.h>
#include <string.h>
void gyro_yaw_begin(gyro_yaw_t *g, const gyro_yaw_config_t *cfg) {
    memset(g, 0, sizeof(*g)); g->cfg = *cfg;
}
static void restart_window(gyro_yaw_t *g) {
    g->samples = 0; g->mean = g->m2 = 0; g->window_started = false;
}
void gyro_yaw_observe(gyro_yaw_t *g, uint64_t now_us, double raw_rate, bool valid, bool stationary) {
    bool gap = !g->sample_valid || now_us <= g->last_sample_us ||
               now_us - g->last_sample_us > g->cfg.max_gap_us;
    double dt = !gap ? (now_us - g->last_sample_us) / 1e6 : 0;
    valid = valid && isfinite(raw_rate);
    if (!valid) {
        g->sample_valid = false;
        if (!g->calibrated) restart_window(g);
        return;
    }
    g->last_sample_us = now_us; g->sample_valid = true;
    if (!g->calibrated) {
        if (!stationary) { restart_window(g); return; }
        if (gap) restart_window(g);
        if (!g->window_started) { g->window_start_us = now_us; g->window_started = true; }
        g->samples++;
        double delta = raw_rate - g->mean;
        g->mean += delta / g->samples;
        g->m2 += delta * (raw_rate - g->mean);
        if (now_us - g->window_start_us < g->cfg.window_us) return;
        double stddev = g->samples > 1 ? sqrt(g->m2 / (g->samples - 1)) : INFINITY;
        if (g->samples < g->cfg.min_samples || fabs(g->mean) > g->cfg.max_bias ||
            stddev > g->cfg.max_stddev) {
            g->rejected_windows++; restart_window(g); return;
        }
        g->bias = g->mean; g->calibrated = true; gap = true;
    }
    double corrected = raw_rate - g->bias;
    double alpha = gap || g->cfg.lpf_hz <= 0 ? 1 :
                   -expm1(-6.283185307179586 * g->cfg.lpf_hz * dt);
    g->rate += alpha * (corrected - g->rate);
}
bool gyro_yaw_get(const gyro_yaw_t *g, uint64_t now_us, double *rate) {
    if (!g->calibrated || !g->sample_valid || now_us < g->last_sample_us ||
        now_us - g->last_sample_us > g->cfg.max_gap_us) return false;
    *rate = g->rate; return true;
}

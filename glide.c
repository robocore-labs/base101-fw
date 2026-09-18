#include "glide.h"
#include <math.h>
#include <string.h>
static double clamp(double x, double limit) { return fmax(-limit, fmin(limit, x)); }
bool glide_init(glide_t *g, const glide_config_t *cfg) {
    const double positive[] = {cfg->radius, cfg->separation, cfg->icr, cfg->wheel_max,
        cfg->vx_max, cfg->wz_max, cfg->ax_max, cfg->alpha_max, cfg->jx_max, cfg->jalpha_max,
        cfg->correction_accel, cfg->correction_jerk};
    for (unsigned i=0; i<sizeof(positive)/sizeof(*positive); i++)
        if (!isfinite(positive[i]) || positive[i] <= 0) return false;
    const double nonnegative[] = {cfg->yaw_kp, cfg->yaw_ki, cfg->correction_max, cfg->deadband};
    for (unsigned i=0; i<sizeof(nonnegative)/sizeof(*nonnegative); i++)
        if (!isfinite(nonnegative[i]) || nonnegative[i] < 0) return false;
    memset(g, 0, sizeof(*g)); g->cfg = *cfg; return true;
}
void glide_reset(glide_t *g) {
    glide_config_t cfg = g->cfg; memset(g, 0, sizeof(*g)); g->cfg = cfg;
}
static bool allocate(glide_t *g, double correction, double *left, double *right) {
    double angular = (g->yaw.velocity + correction) * g->cfg.icr;
    double turn = angular * g->cfg.separation * .5;
    *left = (g->linear.velocity - turn) / g->cfg.radius;
    *right = (g->linear.velocity + turn) / g->cfg.radius;
    double peak = fmax(fabs(*left), fabs(*right));
    if (peak > g->cfg.wheel_max) {
        double scale = g->cfg.wheel_max / peak;
        *left *= scale; *right *= scale; return true;
    }
    return false;
}
bool glide_update(glide_t *g, double vx, double wz, double gyro_z,
                  bool tracking_good, double dt, double *left, double *right) {
    if (!g || !isfinite(vx) || !isfinite(wz) || !isfinite(gyro_z) ||
        !isfinite(dt) || dt <= 0 || dt > .1) return false;
    // Work on a copy so a rejected update cannot partially advance shaping.
    glide_t next = *g;
    if (!motion_profile_step(&next.linear, clamp(vx, next.cfg.vx_max),
                             next.cfg.ax_max, next.cfg.jx_max, dt) ||
        !motion_profile_step(&next.yaw, clamp(wz, next.cfg.wz_max),
                             next.cfg.alpha_max, next.cfg.jalpha_max, dt)) return false;
    next.tracking_bad_s = tracking_good ? 0 : next.tracking_bad_s + dt;
    bool coast = vx == 0 && wz == 0 && fabs(next.linear.velocity) < 1e-4 &&
        fabs(next.yaw.velocity) < 1e-4 && fabs(next.linear.acceleration) < 1e-3 &&
        fabs(next.yaw.acceleration) < 1e-3;
    if (coast || !next.cfg.yaw_feedback) {
        next.integrator = 0;
        if (coast) next.linear = next.yaw = (motion_profile_t){0};
        if (!motion_profile_step(&next.correction_profile, 0, next.cfg.correction_accel,
                                 next.cfg.correction_jerk, dt)) return false;
        if (fabs(next.correction_profile.velocity) < 1e-4 &&
            fabs(next.correction_profile.acceleration) < 1e-3)
            next.correction_profile = (motion_profile_t){0};
        next.correction = clamp(next.correction_profile.velocity, next.cfg.correction_max);
        next.saturated = allocate(&next, next.correction, left, right);
        *g = next; return true;
    }
    double error = next.yaw.velocity - gyro_z;
    if (fabs(error) < next.cfg.deadband) error = 0;
    double old_integrator = next.integrator;
    double candidate = clamp(old_integrator + next.cfg.yaw_ki * error * dt,
                             next.cfg.correction_max);
    double raw = next.cfg.yaw_kp * error + candidate;
    double correction = clamp(raw, next.cfg.correction_max);
    bool saturated = allocate(&next, correction, left, right);
    // Freeze I when wheel allocation saturates, correction hits its clamp,
    // or motors have failed to track for 200 ms. Allow I to unwind.
    bool constrained = saturated || fabs(raw) > next.cfg.correction_max ||
                       next.tracking_bad_s >= .2;
    bool unwinding = old_integrator * error < 0;
    next.integrator = !constrained || unwinding ? candidate : old_integrator;
    double desired_correction = clamp(next.cfg.yaw_kp * error + next.integrator,
                                      next.cfg.correction_max);
    if (!motion_profile_step(&next.correction_profile, desired_correction,
                             next.cfg.correction_accel, next.cfg.correction_jerk, dt)) return false;
    next.correction = clamp(next.correction_profile.velocity, next.cfg.correction_max);
    next.saturated = allocate(&next, next.correction, left, right);
    *g = next; return true;
}

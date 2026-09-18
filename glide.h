#ifndef BASE101_GLIDE_H
#define BASE101_GLIDE_H
#include "motion_profile.h"
typedef struct {
    double radius, separation, icr, wheel_max;
    double vx_max, wz_max, ax_max, alpha_max, jx_max, jalpha_max;
    double yaw_kp, yaw_ki, correction_max, deadband;
    double correction_accel, correction_jerk;
    bool yaw_feedback;
} glide_config_t;
typedef struct {
    glide_config_t cfg;
    motion_profile_t linear, yaw, correction_profile;
    double integrator, correction, tracking_bad_s;
    bool saturated;
} glide_t;
bool glide_init(glide_t *g, const glide_config_t *cfg);
void glide_reset(glide_t *g);
// Gyro is already bias-corrected and filtered by the shared gyro estimator.
// Outputs wheel rad/s, before motor polarity. Zero mode is COAST.
bool glide_update(glide_t *g, double vx, double wz, double gyro_z,
                  bool tracking_good, double dt, double *left, double *right);
#endif

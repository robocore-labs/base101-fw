#include "motion_profile.h"
#include <math.h>

static double clamp(double x, double limit) {
    return fmax(-limit, fmin(limit, x));
}

bool motion_profile_step(motion_profile_t *p, double target, double accel_limit,
                         double jerk_limit, double dt) {
    if (!p || !isfinite(target) || !isfinite(accel_limit) || accel_limit <= 0 ||
        !isfinite(jerk_limit) || jerk_limit <= 0 || !isfinite(dt) || dt <= 0 ||
        dt > 0.1 || !isfinite(p->velocity) || !isfinite(p->acceleration) ||
        fabs(p->acceleration) > accel_limit + 1e-9) return false;
    // Small integration steps keep the approach to the target stable even
    // when motor transactions delay the control loop. Desired acceleration
    // falls before reaching the target, allowing acceleration to taper off.
    unsigned steps = (unsigned)ceil(dt / 0.005);
    double h = dt / steps;
    for (unsigned i = 0; i < steps; i++) {
        double error = target - p->velocity;
        double desired = copysign(fmin(accel_limit,
            fmin(6.0 * fabs(error), sqrt(jerk_limit * fabs(error)))), error);
        double old_accel = p->acceleration;
        p->acceleration += clamp(desired - old_accel, jerk_limit * h);
        p->velocity += (old_accel + p->acceleration) * 0.5 * h;
    }
    return true;
}

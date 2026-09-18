#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H
#include <stdbool.h>
typedef struct { double velocity, acceleration; } motion_profile_t;
// Velocity tracking with bounded acceleration and jerk. SI units per axis.
// dt is actual monotonic elapsed time; gaps over 100 ms are rejected.
bool motion_profile_step(motion_profile_t *p, double target, double accel_limit,
                         double jerk_limit, double dt);
#endif

#ifndef BASE101_ODOMETRY_H
#define BASE101_ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    double x, y, yaw, vx, wz;
    uint64_t sample_us;
    bool tracking, valid;
} odometry_t;
// Missing inputs break integration; recovery starts a new interval, avoiding stale extrapolation.
void odometry_update(odometry_t *o, uint64_t now_us, double vx, double wz, bool valid, uint64_t max_dt_us);
void odometry_reset_pose(odometry_t *o);
#endif

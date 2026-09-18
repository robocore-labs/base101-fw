#include "odometry.h"
#include <math.h>
#define PI 3.14159265358979323846
void odometry_reset_pose(odometry_t *o) {
    o->x = o->y = o->yaw = 0;
    o->valid = o->tracking = false;
}
void odometry_update(odometry_t *o, uint64_t now_us, double vx, double wz, bool valid, uint64_t max_dt_us) {
    valid = valid && isfinite(vx) && isfinite(wz);
    if (!valid) {
        o->valid = o->tracking = false;
        return;
    }
    if (o->tracking && now_us > o->sample_us && now_us - o->sample_us <= max_dt_us) {
        double dt = (now_us - o->sample_us) / 1000000.0;
        double distance = (o->vx + vx) * 0.5 * dt;
        double turn = (o->wz + wz) * 0.5 * dt;
        double mid = o->yaw + turn * 0.5;
        o->x += distance * cos(mid);
        o->y += distance * sin(mid);
        o->yaw = remainder(o->yaw + turn, 2 * PI);
        if (o->yaw <= -PI) o->yaw += 2 * PI;
    }
    o->vx = vx; o->wz = wz; o->sample_us = now_us;
    o->valid = o->tracking = true;
}

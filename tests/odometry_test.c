// cc -std=c11 -Wall -Wextra -Werror -I. tests/odometry_test.c odometry.c -lm -o /tmp/odometry_test
#include "odometry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846
static void near(double actual, double expected, double tolerance) {
    assert(fabs(actual - expected) < tolerance);
}
int main(void) {
    odometry_t o = {0};
    uint64_t now = 1000000;
    odometry_update(&o, now, 0.5, 0, true, 50000);
    for (int i=0; i<400; i++) {
        now += i%2 ? 6000 : 4000;
        odometry_update(&o, now, 0.5, 0, true, 50000);
    }
    near(o.x, 1.0, 1e-10); near(o.y, 0, 1e-10);
    odometry_reset_pose(&o); assert(!o.valid);
    odometry_update(&o, now, -0.5, 0, true, 50000);
    for (int i=0; i<200; i++) odometry_update(&o, now+=5000, -0.5, 0, true, 50000);
    near(o.x, -0.5, 1e-10);
    odometry_reset_pose(&o);
    odometry_update(&o, now, 1, 1, true, 50000);
    for (int i=0; i<200; i++) odometry_update(&o, now+=5000, 1, 1, true, 50000);
    near(o.x, sin(1), 2e-6); near(o.y, 1-cos(1), 2e-6); near(o.yaw, 1, 1e-10);
    odometry_reset_pose(&o);
    odometry_update(&o, now, 0, PI, true, 50000);
    for (int i=0; i<2000; i++) odometry_update(&o, now+=5000, 0, PI, true, 50000);
    near(o.yaw, 0, 1e-10); near(o.x, 0, 1e-10);
    odometry_reset_pose(&o);
    odometry_update(&o, now, 1, 0, true, 50000);
    odometry_update(&o, now+=5000, 1, 0, false, 50000); assert(!o.valid);
    odometry_update(&o, now+=1000000, 1, 0, true, 50000); near(o.x, 0, 1e-10);
    odometry_update(&o, now+=5000, 1, 0, true, 50000); near(o.x, .005, 1e-10);
    odometry_update(&o, now+=1000000, 1, 0, true, 50000); near(o.x, .005, 1e-10);
    odometry_update(&o, now+=5000, NAN, 0, true, 50000); assert(!o.valid);
    odometry_update(&o, now+=5000, 1, INFINITY, true, 50000); assert(!o.valid);
    odometry_update(&o, now+=5000, 1, 0, true, 50000); near(o.x, .005, 1e-10);
    puts("Odometry: straight/reverse, arcs, wrapping, reset and invalid/gap recovery passed");
}

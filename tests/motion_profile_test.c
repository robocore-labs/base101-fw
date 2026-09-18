#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motion_profile.h"

static void tick(motion_profile_t *p, double target, double a, double j, double dt) {
    double before = p->acceleration;
    assert(motion_profile_step(p, target, a, j, dt));
    assert(fabs(p->acceleration) <= a + 1e-10);
    assert(fabs(p->acceleration-before) <= j*dt + 1e-10);
    assert(isfinite(p->velocity));
}
int main(void) {
    motion_profile_t p = {0};
    for (unsigned i=0;i<300;i++) tick(&p,0.38,0.7,2.0, i%2 ? 0.023:0.02);
    assert(fabs(p.velocity-0.38)<1e-6 && fabs(p.acceleration)<1e-6);
    for (unsigned i=0;i<300;i++) tick(&p,-0.38,0.7,2.0,0.02);
    assert(fabs(p.velocity+0.38)<1e-6);
    for (unsigned i=0;i<300;i++) tick(&p,0,0.7,2.0,0.02);
    assert(fabs(p.velocity)<1e-6 && fabs(p.acceleration)<1e-6);
    p=(motion_profile_t){0};
    for (unsigned i=0;i<10;i++) tick(&p,0.38,0.7,2.0,0.02);
    for (unsigned i=0;i<300;i++) tick(&p,0,0.7,2.0,0.02);
    assert(fabs(p.velocity)<1e-6);
    motion_profile_t before=p;
    assert(!motion_profile_step(&p,NAN,0.7,2,0.02));
    assert(!motion_profile_step(&p,1,0.7,2,0.101));
    assert(!motion_profile_step(&p,1,0,2,0.02));
    assert(p.velocity==before.velocity && p.acceleration==before.acceleration);
    p=(motion_profile_t){0};
    for(unsigned i=0;i<1000;i++) tick(&p,(i/30)%2 ? -1:1,0.7,2.0,0.02);
    puts("PASS: acceleration/jerk bounds, variable dt, forward/reverse settling, stop during acceleration, repeated reversals, invalid inputs/timing gap");
}

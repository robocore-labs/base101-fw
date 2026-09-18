#include "glide.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static glide_config_t config(void) {
    return (glide_config_t){.radius=.0363,.separation=.2899,.icr=1.5,.wheel_max=20.94,
        .vx_max=.75,.wz_max=2,.ax_max=.7,.alpha_max=3,.jx_max=2,.jalpha_max=10,
        .yaw_kp=.5,.yaw_ki=.3,.correction_max=.5,.deadband=.005,
        .yaw_feedback=true,.correction_accel=.25,.correction_jerk=1};
}
int main(void) {
    glide_config_t cfg=config();glide_t g;double l,r;
    assert(glide_init(&g,&cfg));
    double actual_w=0;
    // Plant has a different effective track coefficient and a motor lag.
    for(unsigned i=0;i<1200;i++) {
        double old_v=g.linear.velocity,old_a=g.linear.acceleration;
        assert(glide_update(&g,.2,.6,actual_w,true,.02,&l,&r));
        assert(fabs(g.linear.acceleration)<=.7+1e-10);
        assert(fabs(g.linear.acceleration-old_a)<=2*.02+1e-10);
        assert(fabs(g.linear.velocity-old_v)<=.7*.02+1e-10);
        assert(fabs(l)<=cfg.wheel_max+1e-10 && fabs(r)<=cfg.wheel_max+1e-10);
        assert(fabs(g.correction)<=.5+1e-10);
        actual_w += ((r-l)*cfg.radius/(cfg.separation*2.0)-actual_w)*.2;
    }
    assert(fabs(actual_w-.6)<.01);
    for(unsigned i=0;i<1200;i++) {
        assert(glide_update(&g,0,-.6,actual_w,true,.02,&l,&r));
        actual_w += ((r-l)*cfg.radius/(cfg.separation*2.0)-actual_w)*.2;
    }
    assert(fabs(actual_w+.6)<.01);
    // An operator zero coasts once shaping settles, even if manually pushed.
    for(unsigned i=0;i<600;i++) assert(glide_update(&g,0,0,.1,true,.02,&l,&r));
    assert(l==0 && r==0 && g.integrator==0 && g.correction==0);
    glide_reset(&g);assert(g.cfg.icr==1.5);
    glide_t before=g;
    assert(!glide_update(&g,0,0,NAN,true,.02,&l,&r));assert(memcmp(&g,&before,sizeof(g))==0);
    assert(!glide_update(&g,0,0,0,true,.11,&l,&r));assert(memcmp(&g,&before,sizeof(g))==0);
    // Saturation bounds all outputs and prevents I growth into a hard cap.
    cfg.wheel_max=1;assert(glide_init(&g,&cfg));
    for(unsigned i=0;i<500;i++) assert(glide_update(&g,.75,2,0,true,.02,&l,&r));
    assert(g.saturated && fabs(l)<=1 && fabs(r)<=1 && fabs(g.integrator)<.05);
    // Sustained tracking errors freeze I even without allocation saturation.
    cfg=config();assert(glide_init(&g,&cfg));
    for(unsigned i=0;i<20;i++) assert(glide_update(&g,0,.3,0,false,.02,&l,&r));
    double frozen=g.integrator;
    for(unsigned i=0;i<100;i++) assert(glide_update(&g,0,.3,0,false,.02,&l,&r));
    assert(g.integrator==frozen);
    // Gyro oscillations cannot cause abrupt PI output changes.
    cfg=config();assert(glide_init(&g,&cfg));
    for(unsigned i=0;i<500;i++) {
        double before_accel=g.correction_profile.acceleration, before_correction=g.correction;
        assert(glide_update(&g,.1,.2,i%2?.5:-.5,true,.02,&l,&r));
        assert(fabs(g.correction-before_correction)<=cfg.correction_accel*.02+1e-10);
        assert(fabs(g.correction_profile.acceleration-before_accel)<=cfg.correction_jerk*.02+1e-10);
    }
    // With PI disabled, noisy gyro data cannot affect wheel commands.
    cfg.yaw_feedback=false;glide_t quiet,noisy;
    assert(glide_init(&quiet,&cfg) && glide_init(&noisy,&cfg));
    for(unsigned i=0;i<300;i++) {
        double ql,qr,nl,nr;
        assert(glide_update(&quiet,.1,.2,0,true,.02,&ql,&qr));
        assert(glide_update(&noisy,.1,.2,i%2?.5:-.5,true,.02,&nl,&nr));
        assert(ql==nl && qr==nr && noisy.correction==0);
    }
    cfg.radius=0;assert(!glide_init(&g,&cfg));
    puts("Glide: lag/slip yaw tracking both signs, jerk limits, coast/reset, saturation, anti-windup and invalid-input rejection passed");
}

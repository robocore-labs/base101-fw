#include "gyro_yaw.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    gyro_yaw_config_t cfg = {.window_us=15000000, .max_gap_us=50000,
        .min_samples=750, .max_bias=.05, .max_stddev=.003, .lpf_hz=20};
    gyro_yaw_t g; double rate;
    gyro_yaw_begin(&g,&cfg); uint64_t now=1000000;
    for(unsigned i=0;i<750;i++,now+=20000)
        gyro_yaw_observe(&g,now,-.02+.0005*sin(i),true,true);
    assert(!g.calibrated && !gyro_yaw_get(&g,now,&rate));
    gyro_yaw_observe(&g,now,-.02,true,true);
    assert(g.calibrated && fabs(g.bias+.02)<1e-5);
    assert(gyro_yaw_get(&g,now,&rate) && fabs(rate)<1e-5);
    double bias=g.bias;
    gyro_yaw_observe(&g,now+=20000,.18,true,false);
    assert(gyro_yaw_get(&g,now,&rate) && rate>0 && rate<.201);
    assert(g.bias==bias); // No recalibration during normal movement.
    assert(!gyro_yaw_get(&g,now+50001,&rate));
    gyro_yaw_observe(&g,now+=20000,NAN,true,true);
    assert(!gyro_yaw_get(&g,now,&rate) && g.calibrated);
    gyro_yaw_observe(&g,now+=20000,-.02,true,true);
    assert(gyro_yaw_get(&g,now,&rate) && fabs(rate)<1e-5);
    gyro_yaw_begin(&g,&cfg);
    for(unsigned i=0;i<400;i++) gyro_yaw_observe(&g,now+=20000,-.02,true,true);
    gyro_yaw_observe(&g,now+=20000,.2,true,false);
    assert(!g.window_started && !g.samples);
    for(unsigned i=0;i<400;i++) gyro_yaw_observe(&g,now+=20000,-.02,true,true);
    assert(!g.calibrated);
    gyro_yaw_observe(&g,now+=100000,-.02,true,true);
    assert(g.samples==1); // A gap restarts the continuous window.
    for(unsigned i=0;i<750;i++) gyro_yaw_observe(&g,now+=20000,-.02,true,true);
    assert(g.calibrated);
    gyro_yaw_begin(&g,&cfg);
    for(unsigned i=0;i<751;i++) gyro_yaw_observe(&g,now+=20000,i%2?.01:-.01,true,true);
    assert(!g.calibrated && g.rejected_windows==1);
    gyro_yaw_begin(&g,&cfg);
    for(unsigned i=0;i<751;i++) gyro_yaw_observe(&g,now+=20000,.06,true,true);
    assert(!g.calibrated && g.rejected_windows==1);
    puts("Gyro: full 15s window, bias/filter, freshness, movement/gap restart and noisy/excessive-bias rejection passed");
}

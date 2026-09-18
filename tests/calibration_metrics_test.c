#include "calibration_metrics.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    calibration_metrics_t m; float a[3]={0,0,9.80665f};
    calibration_metrics_begin(&m,1000,a);
    for(unsigned i=0;i<=600;i++) {
        double g=i<100?i*.002:.2,e=g*1.5;
        calibration_metrics_observe(&m,1000+i*10000,a,g,0,e,0,.2,0,g,1.5,true,true);
    }
    assert(m.steady_s>3.9 && fabs(m.steady_encoder/m.steady_gyro-1.5)<1e-10);
    assert(m.gaps==0 && m.knocks==0 && m.tracking_energy<1e-10);
    a[0]=2;calibration_metrics_observe(&m,6021000,a,.2,0,.3,0,.2,0,.2,1.5,true,true);
    assert(m.knocks==1 && m.accel_peak>1.9);
    a[0]=0;calibration_metrics_observe(&m,6121000,a,.2,0,.3,0,.2,0,.2,1.5,true,true);
    assert(m.gaps==1);
    calibration_metrics_observe(&m,6131000,a,NAN,0,0,0,0,0,0,1.5,true,true);
    assert(m.gaps==2);
    calibration_metrics_begin(&m,7000000,a);
    for(unsigned i=0;i<100;i++)
        calibration_metrics_observe(&m,7000000+i*10000,a,0,.1,0,.1,0,.1,0,1.5,true,true);
    for(unsigned i=0;i<=50;i++)
        calibration_metrics_observe(&m,8000000+i*10000,a,0,.1*(1-i/50.0),0,0,0,0,0,1.5,true,true);
    assert(fabs(m.response_s-.01)<1e-9 && fabs(m.stop_distance-.026)<.0001);
    a[0]=0;calibration_metrics_begin(&m,9000000,a);
    for(unsigned i=0;i<50;i++) {
        a[0]=.9f;
        calibration_metrics_observe(&m,9000000+i*10000,a,0,i*.009,0,.4,0,i*.009,0,1.5,true,true);
    }
    assert(m.knocks==0 && m.accel_peak<.001); // Normal 0.9 m/s² acceleration is not a knock.
    a[0]=0;calibration_metrics_begin(&m,10000000,a);m.body_response=false;
    for(unsigned i=0;i<10;i++)
        calibration_metrics_observe(&m,10000000+i*10000,a,0,0,.45,0,.3,0,.3,1.5,true,true);
    assert(m.response_s>=0); // Supported chassis cannot turn, but encoder response is useful.
    puts("calibration full-rate metrics checks passed");
}

#include "calibration_metrics.h"
#include <math.h>
#include <string.h>
void calibration_metrics_begin(calibration_metrics_t *m, uint64_t now, const float accel[3]) {
    memset(m,0,sizeof(*m)); m->active=true; m->body_response=true; m->start_us=now; m->response_s=-1;
    for(unsigned i=0;i<3;i++) m->baseline[i]=accel[i];
}
void calibration_metrics_observe(calibration_metrics_t *m, uint64_t now,
    const float accel[3], double gyro, double vx, double encoder,
    double requested_vx, double requested_wz, double shaped_vx, double shaped_wz,
    double icr, bool valid, bool moving) {
    if (!m->active) return;
    valid=valid && isfinite(gyro) && isfinite(vx) && isfinite(encoder);
    for(unsigned i=0;i<3;i++) valid=valid && isfinite(accel[i]);
    if (!valid) {m->gaps++;m->previous_valid=false;return;}
    if (!m->motion_start_us && moving) m->motion_start_us=now;
    double dt=m->last_us && now>m->last_us ? (now-m->last_us)/1e6 : 0;
    bool interval=m->previous_valid && dt>0 && dt<=.05;
    bool steady=moving && fabs(requested_vx)<.001 && fabs(requested_wz)>.05 &&
        fabs(shaped_wz-requested_wz)<fabs(requested_wz)*.05 &&
        m->motion_start_us && now-m->motion_start_us>=2000000u;
    if (interval) {
        m->gyro_yaw+=(gyro+m->previous_gyro)*.5*dt;
        m->encoder_yaw+=(encoder+m->previous_encoder)*.5*dt;
        m->distance+=(vx+m->previous_vx)*.5*dt;
        if (steady && m->previous_steady) {
            m->steady_gyro+=(gyro+m->previous_gyro)*.5*dt;
            m->steady_encoder+=(encoder+m->previous_encoder)*.5*dt;
            m->steady_s+=dt;
        }
        double delta2=0,jerk2=0;
        for(unsigned i=0;i<3;i++) {
            double d=accel[i]-m->baseline[i];delta2+=d*d;
            double j=(accel[i]-m->previous_accel[i])/dt;jerk2+=j*j;
        }
        // Conservative bound on rotational acceleration at the known 40 mm
        // lever arm. No assumption about the accelerometer's X/Y mounting.
        double angular_accel=fabs(gyro-m->previous_gyro)/dt;
        double commanded_accel=fabs(shaped_vx-m->previous_shaped_vx)/dt;
        double residual=fmax(0,sqrt(delta2)-commanded_accel-.04*(angular_accel+gyro*gyro));
        m->accel_peak=fmax(m->accel_peak,residual);
        m->jerk_peak=fmax(m->jerk_peak,sqrt(jerk2));
        m->accel_energy+=residual*residual*dt; m->elapsed_s+=dt;
        if (moving) {
            double error=vx-shaped_vx;
            if(fabs(requested_wz)>.01) error=encoder-shaped_wz*icr; // reported per-axis below
            m->tracking_energy+=error*error*dt;m->tracking_s+=dt;
        }
        if(residual>.8 && (!m->last_knock_us || now-m->last_knock_us>80000u)) {
            m->knocks++;m->last_knock_us=now;
        }
        if(moving && m->response_s<0 && m->motion_start_us) {
            double req=fabs(requested_vx)>.001 ? requested_vx : requested_wz;
            double measured=fabs(requested_vx)>.001 ? vx : m->body_response ? gyro : encoder/icr;
            if(fabs(req)>.001 && measured/req>=.9) m->response_s=(now-m->motion_start_us)/1e6;
        }
        if(moving && requested_vx==0 && requested_wz==0) {
            m->stop_distance+=fabs(vx+m->previous_vx)*.5*dt;m->stop_s+=dt;
        }
    } else if(m->last_us) m->gaps++;
    m->samples++;m->last_us=now;m->previous_valid=true;m->previous_steady=steady;
    m->previous_shaped_vx=shaped_vx;
    m->previous_gyro=gyro;m->previous_encoder=encoder;m->previous_vx=vx;
    for(unsigned i=0;i<3;i++) m->previous_accel[i]=accel[i];
}

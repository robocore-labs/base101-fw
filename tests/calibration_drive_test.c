/* Actual calibration firmware helpers and shared wheels, with deterministic buses/time. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define CALIBRATION_FIRMWARE 1
#include "../wheels.c"
#define main calibration_main
#include "../calibration_firmware.c"
#undef main
static uint64_t clock_us;
static int16_t speeds[4];
static uint8_t ramps[4];
static bool received[4] = {true, true, true, true};
static unsigned index_of(serial_t *s) {
    serial_t *inner = ((serial_hook_t *)s)->inner;
    for (unsigned i = 0; i < 4; ++i) if (inner == &ports[i].base) return i;
    assert(false); return 0;
}
uint64_t time_us_64(void) { return clock_us; }
void sleep_ms(unsigned ms) { clock_us += ms * 1000; }
serial_t *link101_pio_serial_init(link101_pio_serial_t *p, uint tx, uint rx, int enable, uint32_t baud) {
    (void)tx; (void)rx; (void)enable; (void)baud; return &p->base;
}
int ddsm210_get_mode(serial_t *s, uint8_t id) { (void)s; (void)id; return DDSM210_MODE_VELOCITY; }
bool ddsm210_set_mode(serial_t *s, uint8_t id, uint8_t mode) {
    (void)s; (void)id; (void)mode; return false; // Correct mode, no write acknowledgement.
}
bool ddsm210_set_velocity(serial_t *s, uint8_t id, int16_t rpm, uint8_t accel, ddsm210_feedback_t *fb) {
    (void)id; unsigned i = index_of(s); speeds[i] = rpm; ramps[i]=accel;
    if (fb) *fb = (ddsm210_feedback_t){.feedback1 = rpm};
    return received[i];
}
bool ddsm210_brake_feedback(serial_t *s, uint8_t id, ddsm210_feedback_t *fb) {
    return ddsm210_set_velocity(s, id, 0, 0, fb);
}
bool ddsm210_brake(serial_t *s, uint8_t id) { return ddsm210_brake_feedback(s, id, NULL); }
bool ddsm210_get_odometry(serial_t *s, uint8_t id, ddsm210_odometry_t *out) {
    (void)s; (void)id; *out = (ddsm210_odometry_t){0}; return true;
}
void io_poll(void) {}
void status_begin(void) {}
void status_set_mode(status_mode_t mode) { (void)mode; }
void status_printf(const char *fmt, ...) { (void)fmt; }
void tusb_init(void) {}
bool tud_mounted(void) { return true; }
uint32_t tud_cdc_n_write_available(unsigned instance) { (void)instance; return 4096; }
uint32_t tud_cdc_n_write(unsigned instance, const void *data, uint32_t length) { (void)instance; (void)data; return length; }
void tud_cdc_n_write_flush(unsigned instance) { (void)instance; }
uint32_t tud_cdc_n_available(unsigned instance) { (void)instance; return 0; }
uint32_t tud_cdc_n_read(unsigned instance, void *data, uint32_t length) { (void)instance; (void)data; (void)length; return 0; }
bool imu_begin(void) { return true; }
void imu_update(void) {}
bool imu_read(imu_sample_t *out) { *out = (imu_sample_t){.accel = {0, 0, 9.80665}, .gyro = {0, 0, .02}, .temp_c = 25}; return true; }
void imu_get_health(imu_health_t *out) { *out = (imu_health_t){.sensor_online = true, .read_successes = 3000}; }
static void tick(unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        clock_us += 5000;
        imu_sample_t s; imu_read(&s);
        wheels_observe_gyro(&s, clock_us, true);
        wheels_update();
    }
}
int main(int argc, char **argv) {
    assert(wheels_begin() == 4); // No mode-write reply must not mark the motors offline.
    brake_all(); tick(10);
    assert(!wheels_set_velocity(.1, .3)); // Startup calibration gates movement.
    tick(3010);
    wheels_yaw_status_t y; wheels_get_yaw_status(clock_us, &y);
    assert(y.calibrated && y.fresh && fabs(y.bias - .02) < 1e-6);
    assert(!controller.cfg.yaw_feedback);
    wheels_drive_status_t d; wheels_get_drive_status(clock_us, &d);
    assert(d.at_rest && d.brake_ok_mask == 15 && !d.motion_enabled);
    // Mirrored motors must emit ordinary zero, not negative-zero %g edge cases.
    for (unsigned i = 0; i < 4; ++i) assert(d.measured_rpm[i] == 0 && !signbit(d.measured_rpm[i]));
    online[0] = false;
    wheels_retry_offline(); tick(5);
    assert(wheels_online(0));
    assert(wheels_configure_motion(.7, 2, 3, 10, 1.5));
    assert(wheels_set_velocity(.1, 0)); tick(500);
    wheels_get_drive_status(clock_us, &d);
    assert(d.motion_enabled && !d.at_rest);
    assert(speeds[0] > 0 && speeds[1] < 0 && speeds[2] > 0 && speeds[3] < 0);
    double vx, wz; assert(wheels_measured_twist(clock_us, &vx, &wz));
    assert(fabs(vx - .1) < .002 && fabs(wz) < .001);
    assert(!wheels_configure_motion(1, 3, 4, 12, 2));
    assert(controller.cfg.icr == 1.5);
    assert(wheels_set_velocity(0, .3)); tick(500);
    assert(wheels_measured_twist(clock_us, &vx, &wz));
    assert(fabs(wz - .45) < .005); // Same ICR feedforward as production.
    assert(wheels_set_velocity(0, 0)); tick(400); brake_all();
    assert(wheels_configure_motion(1, 3, 4, 12, 2));
    assert(controller.cfg.icr == 2 && controller.cfg.ax_max == 1);
    assert(!wheels_configure_motion(NAN, 3, 4, 12, 2));
    assert(controller.cfg.ax_max == 1);
    assert(wheels_set_velocity(.1, 0)); tick(100);
    received[1] = false; tick(5);
    wheels_get_drive_status(clock_us, &d);
    assert(!d.motion_enabled && !d.fresh[1]);
    for (unsigned i = 0; i < 4; ++i) assert(speeds[i] == 0);
    received[1] = true; tick(5);
    assert(wheels_set_velocity(.1, 0)); tick(100);
    imu_sample_t invalid = {0}; wheels_observe_gyro(&invalid, clock_us, false);
    clock_us += 20000; wheels_update();
    wheels_get_drive_status(clock_us, &d);
    assert(!d.motion_enabled);
    for (unsigned i = 0; i < 4; ++i) assert(speeds[i] == 0);
    assert(!wheels_set_velocity(.1, 0));
    tick(10);
    calibration_protocol_t defaults; calibration_init(&defaults,(calibration_parameters_t){.ax=.7,.jx=2,.aw=3,.jw=10,.k_icr=1.5});
    calibration_parameters_t parameters=defaults.parameters;
    parameters.radius=.05; parameters.separation=.33; parameters.ramp=2; parameters.rpm_max=150;
    assert(wheels_configure_calibration(&parameters));
    assert(wheels_set_velocity(.1,.3));tick(500);
    assert(wheels_measured_twist(clock_us,&vx,&wz));
    assert(fabs(vx-.1)<.002 && fabs(wz-.45)<.005);
    for(unsigned i=0;i<4;i++)assert(ramps[i]==2 && abs(speeds[i])<=1500);
    assert(!wheels_configure_calibration(&defaults.parameters));
    brake_all();tick(10);parameters=defaults.parameters;parameters.lpf=15;
    assert(wheels_configure_calibration(&parameters));assert(!wheels_set_velocity(.1,0));
    tick(3010);assert(wheels_set_velocity(.1,0));brake_all();tick(10);
    assert(wheels_configure_calibration(&defaults.parameters));tick(3010);
    if (argc == 2 && !strcmp(argv[1], "--json")) {
        calibration_protocol_t state; char reply[4096], input[] = "123 hello 2";
        calibration_init(&state, (calibration_parameters_t){.ax=1, .jx=3, .aw=4, .jw=12, .k_icr=2});
        telemetry_parameters=state.parameters;
        observe_state(&state, clock_us);
        calibration_command(&state, input, clock_us, reply, sizeof(reply));
        imu_read(&sample); sample_valid = true; sample_us = clock_us;
        add_telemetry(reply, sizeof(reply), clock_us);
        assert(strlen(reply) > 0 && strlen(reply) < 4096 && reply[strlen(reply)-1] == '\n');
        fputs(reply, stdout);
    } else puts("calibration shared-drive checks passed");
    return 0;
}

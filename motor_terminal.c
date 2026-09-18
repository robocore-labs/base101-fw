/* Bounded motor experiments over the sole USB CDC, with no ROS dependency. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "tusb.h"
#include "ddsm210.h"
#include "link101/pio_serial.h"
#include "serial_hook.h"
#include "robot.h"
#include "io.h"
#include "status.h"
#include "motion_profile.h"

#define TWO_PI 6.283185307179586
static link101_pio_serial_t ports[WHEEL_COUNT];
static serial_hook_t hooks[WHEEL_COUNT];
static serial_t *bus[WHEEL_COUNT];
static bool online[WHEEL_COUNT];
static double target[WHEEL_COUNT], commanded[WHEEL_COUNT]; // physical RPM
static double software_accel = 6.0; // rad/s^2; zero disables software ramp
static uint8_t hardware_accel = 1; // 0.1 ms/RPM
static int selected = -1; // all, or one wheel index
static uint64_t until_us, next_tick_us, next_report_us;
static bool moving, faulted;
static enum { MODE_HW, MODE_LINEAR, MODE_SHAPE } mode = MODE_SHAPE;
static motion_profile_t linear_profile, yaw_profile;
static double ax = 0.7, jx = 2.0, aw = 3.0, jw = 10.0;
static uint64_t last_tick_us;
static bool at_rest(void) {
    if (moving) return false;
    for (unsigned i = 0; i < WHEEL_COUNT; i++)
        if (fabs(commanded[i]) > 0.01) return false;
    return fabs(linear_profile.acceleration) < 0.001 && fabs(yaw_profile.acceleration) < 0.001;
}

static void stop(bool brake) {
    moving = false;
    if (brake) { linear_profile = (motion_profile_t){0}; yaw_profile = (motion_profile_t){0}; }
    for (unsigned i = 0; i < WHEEL_COUNT; i++) {
        target[i] = 0;
        if (brake && online[i]) {
            commanded[i] = 0;
            ddsm210_brake(bus[i], WHEELS[i].id);
        }
    }
}

static void settings(void) {
    status_printf("mode=%s ax=%.3f jx=%.3f aw=%.3f jw=%.3f\r\n",
                  mode == MODE_SHAPE ? "shape" : mode == MODE_HW ? "hw" : "linear", ax, jx, aw, jw);
    status_printf("settings wheel=%d (-1=all) hw=%u (%.1f ms/RPM) sw=%.2f rad/s^2 moving=%u fault=%u\r\n",
                  selected, hardware_accel, hardware_accel * 0.1, software_accel, moving, faulted);
}

static void help(void) {
    status_printf("motor_terminal v2; physical wheel RPM, motor polarity applied automatically\r\n");
    status_printf("help | status | hw 1..255 | sw 0..100 (rad/s^2; 0=hardware-only)\r\n");
    status_printf("wheel all|0|1|2|3 (FL FR BL BR); change selection only at rest\r\n");
    status_printf("run LEFT_RPM RIGHT_RPM SECONDS (max +/-200 RPM, 0.1..10 seconds)\r\n");
    status_printf("stop = ramp to zero; brake or ! = immediate electric brake\r\n");
    status_printf("mode shape|hw|linear; shape AX JX AW JW (at rest); clear = check motors/reset fault\r\n");
    settings();
}

static bool number(const char *s, double *out) {
    char *end;
    *out = strtod(s, &end);
    return end != s && *end == '\0' && isfinite(*out);
}

static void command(char *line) {
    char *args[5];
    unsigned n = 0;
    char *word = strtok(line, " \t");
    while (word && n < 5) { args[n++] = word; word = strtok(NULL, " \t"); }
    if (!n) return;
    if (word) { status_printf("ERR too many arguments\r\n"); return; }
    if (n == 1 && !strcmp(args[0], "help")) { help(); return; }
    if (n == 1 && !strcmp(args[0], "status")) { settings(); return; }
    if (n == 1 && (!strcmp(args[0], "stop") || !strcmp(args[0], "brake"))) {
        stop(!strcmp(args[0], "brake")); status_printf("OK %s\r\n", args[0]); return;
    }
    if (n == 1 && !strcmp(args[0], "clear")) {
        stop(true);
        bool good = true;
        for (unsigned i = 0; i < WHEEL_COUNT; i++)
            good = good && online[i] && ddsm210_get_mode(bus[i], WHEELS[i].id) == DDSM210_MODE_VELOCITY;
        if (!good) { status_printf("ERR clear: motor mode/reply missing; fault remains\r\n"); return; }
        faulted = false; last_tick_us = time_us_64(); next_tick_us = last_tick_us + 20000;
        status_printf("OK cleared; stopped\r\n"); return;
    }
    double a, b, c, d;
    if (n == 2 && !strcmp(args[0], "mode") && at_rest()) {
        if (!strcmp(args[1], "shape") && selected == -1) mode = MODE_SHAPE;
        else if (!strcmp(args[1], "hw")) mode = MODE_HW;
        else if (!strcmp(args[1], "linear")) mode = MODE_LINEAR;
        else { status_printf("ERR mode; shape requires wheel all\r\n"); return; }
        linear_profile = (motion_profile_t){0}; yaw_profile = (motion_profile_t){0};
        settings(); return;
    }
    if (n == 5 && !strcmp(args[0], "shape") && at_rest() &&
        number(args[1], &a) && number(args[2], &b) &&
        number(args[3], &c) && number(args[4], &d) &&
        a >= 0.01 && a <= 5 && b >= 0.01 && b <= 50 &&
        c >= 0.01 && c <= 20 && d >= 0.01 && d <= 200) {
        ax = a; jx = b; aw = c; jw = d; settings(); return;
    }
    if (n == 2 && !strcmp(args[0], "hw") && number(args[1], &a) &&
        a >= 1 && a <= 255 && floor(a) == a) {
        if (!at_rest()) { status_printf("ERR settings only at rest\r\n"); return; }
        hardware_accel = (uint8_t)a; settings(); return;
    }
    if (n == 2 && !strcmp(args[0], "sw") && number(args[1], &a) && a >= 0 && a <= 100) {
        if (!at_rest()) { status_printf("ERR settings only at rest\r\n"); return; }
        software_accel = a; mode = a == 0 ? MODE_HW : MODE_LINEAR;
        linear_profile = (motion_profile_t){0}; yaw_profile = (motion_profile_t){0};
        settings(); return;
    }
    if (n == 2 && !strcmp(args[0], "wheel") && at_rest()) {
        if (!strcmp(args[1], "all")) selected = -1;
        else if (number(args[1], &a) && a >= 0 && a < WHEEL_COUNT && floor(a) == a) selected = (int)a;
        else { status_printf("ERR wheel index\r\n"); return; }
        if (selected != -1 && mode == MODE_SHAPE) mode = MODE_HW;
        settings(); return;
    }
    if (n == 4 && !strcmp(args[0], "run") && number(args[1], &a) &&
        number(args[2], &b) && number(args[3], &c) && fabs(a) <= 200 &&
        fabs(b) <= 200 && c >= 0.1 && c <= 10) {
        if (faulted) { status_printf("ERR fault latched; reboot after checking motor feedback\r\n"); return; }
        if (mode == MODE_SHAPE) {
            for (unsigned i = 0; i < WHEEL_COUNT; i++) {
                if (!online[i]) { status_printf("ERR shape requires all four motors online\r\n"); return; }
            }
        }
        bool any = false;
        for (unsigned i = 0; i < WHEEL_COUNT; i++) {
            target[i] = (selected < 0 || selected == (int)i) && online[i] ? (WHEELS[i].left ? a : b) : 0;
            any = any || ((selected < 0 || selected == (int)i) && online[i]);
        }
        if (!any) { status_printf("ERR no selected motor online\r\n"); return; }
        if (at_rest()) last_tick_us = time_us_64();
        moving = true;
        until_us = time_us_64() + (uint64_t)(c * 1000000);
        status_printf("OK run left=%.2f right=%.2f duration=%.2fs hw=%u sw=%.2f\r\n", a, b, c, hardware_accel, software_accel);
        return;
    }
    status_printf("ERR invalid command or selection while moving; help\r\n");
}

static void update(void) {
    if (faulted) return;
    uint64_t now = time_us_64();
    if (moving && now >= until_us) { stop(false); status_printf("DONE duration; ramping to zero\r\n"); }
    if (now < next_tick_us) return;
    double dt = last_tick_us ? (now - last_tick_us) / 1000000.0 : 0.02;
    last_tick_us = now;
    next_tick_us = now + 20000;
    double shaped_left = 0, shaped_right = 0;
    if (mode == MODE_SHAPE) {
        // No trajectory evolves while exactly idle; USB enumeration and
        // startup delays must not trigger the active-motion timing watchdog.
        if (at_rest()) dt = 0.02;
        double l = target[0] * TWO_PI / 60 * WHEEL_RADIUS_M;
        double r = target[1] * TWO_PI / 60 * WHEEL_RADIUS_M;
        if (!motion_profile_step(&linear_profile, (l+r)*0.5, ax, jx, dt) ||
            !motion_profile_step(&yaw_profile, (r-l)/WHEEL_SEPARATION_M, aw, jw, dt)) {
            faulted = true; stop(true);
            status_printf("FAULT profile timing/state dt=%.4fs; all motors braked\r\n", dt); return;
        }
        shaped_left = (linear_profile.velocity - yaw_profile.velocity * WHEEL_SEPARATION_M * 0.5) / WHEEL_RADIUS_M * 60 / TWO_PI;
        shaped_right = (linear_profile.velocity + yaw_profile.velocity * WHEEL_SEPARATION_M * 0.5) / WHEEL_RADIUS_M * 60 / TWO_PI;
        if (fmax(fabs(shaped_left), fabs(shaped_right)) > 210) {
            faulted = true; stop(true);
            status_printf("FAULT shaped speed exceeded limit; all motors braked\r\n"); return;
        }
    }
    bool report = now >= next_report_us;
    if (report) next_report_us = now + 200000;
    if (report && mode == MODE_SHAPE)
        status_printf("body v=%.3f a=%.3f w=%.3f alpha=%.3f dt=%.4f\r\n",
            linear_profile.velocity, linear_profile.acceleration,
            yaw_profile.velocity, yaw_profile.acceleration, dt);
    for (unsigned i = 0; i < WHEEL_COUNT; i++) {
        if (!online[i]) continue;
        double delta = target[i] - commanded[i];
        double step = software_accel * 60.0 / TWO_PI / 50.0;
        if (mode == MODE_LINEAR && software_accel > 0) delta = fmax(-step, fmin(step, delta));
        if (mode == MODE_SHAPE) commanded[i] = WHEELS[i].left ? shaped_left : shaped_right;
        else commanded[i] += delta;
        if (fabs(commanded[i]) < 1e-6) commanded[i] = 0;
        ddsm210_feedback_t fb;
        bool ok = ddsm210_set_velocity(bus[i], WHEELS[i].id,
            (int16_t)lround(commanded[i] * WHEELS[i].direction * 10), hardware_accel, &fb);
        if (!ok) {
            faulted = true; stop(true);
            status_printf("FAULT wheel=%u no reply; all motors braked\r\n", i);
            return;
        }
        if (report) status_printf("wheel=%u target=%.1f sent=%.1f measured=%.1f RPM hw_echo=%u temp=%u error=0x%02X\r\n",
            i, target[i], commanded[i], fb.feedback1 * WHEELS[i].direction * 0.1,
            fb.accel_time, fb.temperature, fb.error_code);
        if (fb.error_code) {
            faulted = true; stop(true); status_printf("FAULT wheel=%u error=0x%02X; all motors braked\r\n", i, fb.error_code); return;
        }
    }
}

int main(void) {
    io_begin(); status_begin();
    for (unsigned i = 0; i < WHEEL_COUNT; i++) {
        bus[i] = serial_hook_init(&hooks[i], link101_pio_serial_init(&ports[i],
            WHEELS[i].tx_pin, WHEELS[i].rx_pin, LINK101_NO_TXEN, WHEEL_BAUD), io_poll);
        online[i] = bus[i] && ddsm210_get_mode(bus[i], WHEELS[i].id) >= 0;
        if (online[i]) {
            ddsm210_set_mode(bus[i], WHEELS[i].id, DDSM210_MODE_VELOCITY);
            sleep_ms(10); ddsm210_brake(bus[i], WHEELS[i].id);
        }
        status_printf("wheel=%u %s online=%u\r\n", i, WHEELS[i].joint, online[i]);
    }
    status_set_mode(STATUS_READY);
    char line[128]; unsigned used = 0; bool overflow = false, connected = false;
    while (true) {
        io_poll();
        bool mounted = tud_mounted();
        if (connected && !mounted) { stop(true); used = 0; overflow = false; }
        if (!connected && mounted) help();
        connected = mounted;
        while (tud_cdc_n_available(0)) {
            char ch; tud_cdc_n_read(0, &ch, 1);
            if (ch == '!' || ch == 3) { stop(true); used = 0; overflow = false; status_printf("OK brake\r\n"); }
            else if (ch == '\r' || ch == '\n') {
                line[used] = 0;
                if (!overflow) command(line); else status_printf("ERR line too long\r\n");
                used = 0; overflow = false;
            } else if (ch == 8 || ch == 127) { if (used) used--; }
            else if (used + 1 < sizeof(line)) line[used++] = ch;
            else overflow = true;
        }
        if (mounted) update();
        sleep_ms(1);
    }
}

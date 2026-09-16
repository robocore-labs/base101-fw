#include "wheels.h"

#include <math.h>

#include "pico/stdlib.h"

#include "ddsm210.h"
#include "link101/pio_serial.h"
#include "serial_hook.h"

#include "io.h"
#include "robot.h"
#include "status.h"

#define TWO_PI 6.283185307179586

static link101_pio_serial_t ports[WHEEL_COUNT];
static serial_hook_t        hooks[WHEEL_COUNT];
static serial_t            *bus[WHEEL_COUNT];
static bool                 online[WHEEL_COUNT];
static double               last_command[WHEEL_COUNT];

uint8_t wheels_begin(void) {
    uint8_t found = 0;

    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        const wheel_cfg_t *w = &WHEELS[i];

        // The port, wrapped so USB and the lidar keep running while the
        // driver waits for this motor to answer.
        bus[i] = serial_hook_init(&hooks[i],
                                  link101_pio_serial_init(&ports[i],
                                                          w->tx_pin, w->rx_pin,
                                                          LINK101_NO_TXEN, WHEEL_BAUD),
                                  io_poll);
        if (!bus[i]) {
            status_printf("[wheel] %s: no PIO state machine free\n", w->joint);
            continue;
        }

        // Asking for its mode is the cheapest way to ask "are you there?".
        online[i] = ddsm210_get_mode(bus[i], w->id) >= 0;
        status_printf("[wheel] %s on GP%u/%u: %s\n", w->joint, w->tx_pin, w->rx_pin,
                      online[i] ? "online" : "no response");
        if (!online[i]) {
            continue;
        }

        ddsm210_set_mode(bus[i], w->id, DDSM210_MODE_VELOCITY);
        sleep_ms(10);
        found++;
    }
    return found;
}

bool wheels_online(uint8_t index) {
    return index < WHEEL_COUNT && online[index];
}

void wheels_set_speed(uint8_t index, double rad_per_sec) {
    if (!wheels_online(index)) {
        return;
    }
    if (fabs(rad_per_sec - last_command[index]) < 1e-6) {
        return;   // same as last time; leave the bus alone
    }
    last_command[index] = rad_per_sec;

    const wheel_cfg_t *w = &WHEELS[index];

    // rad/s at the wheel -> RPM -> the motor's 0.1 RPM units.
    double rpm = rad_per_sec * w->direction * WHEEL_SPEED_SCALE * 60.0 / TWO_PI;
    int32_t units = (int32_t)(rpm * 10.0);

    int32_t limit = WHEEL_MAX_RPM * 10;
    if (limit > DDSM210_MAX_SPEED_RAW) {
        limit = DDSM210_MAX_SPEED_RAW;
    }
    if (units >  limit) units =  limit;
    if (units < -limit) units = -limit;

    ddsm210_set_velocity(bus[index], w->id, (int16_t)units, WHEEL_ACCEL_TIME, NULL);
}

double wheels_read_angle(uint8_t index) {
    if (!wheels_online(index)) {
        return 0.0;
    }

    ddsm210_odometry_t odom;
    if (!ddsm210_get_odometry(bus[index], WHEELS[index].id, &odom)) {
        return 0.0;
    }

    // Absolute angle = whole turns since boot + where in this turn we are.
    double turns    = (double)odom.mileage_laps;
    double fraction = (double)odom.position / DDSM210_ENCODER_TICKS;
    return (turns + fraction) * TWO_PI * WHEELS[index].direction;
}

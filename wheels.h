/*
 * The four drive wheels (Waveshare DDSM210, velocity mode).
 *
 * One motor per port: a DDSM210 cannot share a TX line without external
 * gating, so each wheel gets its own PIO UART. Which pins, which corner and
 * which way it spins is the WHEELS table in robot.h; everything here is
 * indexed by position in that table.
 *
 * Wheels that don't answer at boot are remembered as offline and skipped,
 * so a disconnected motor costs one probe at startup and nothing after.
 */

#ifndef WHEELS_H
#define WHEELS_H

#include <stdbool.h>
#include <stdint.h>

// Open a port per wheel, find out who is there, and put them in velocity
// mode. Returns how many answered.
uint8_t wheels_begin(void);

bool wheels_online(uint8_t index);

// Drive one wheel, in rad/s at the wheel. Direction and the speed cap from
// robot.h are applied here. Repeating a speed is free -- an unchanged
// command is not re-sent.
void wheels_set_speed(uint8_t index, double rad_per_sec);

// Where the wheel is now, in radians, counting full turns since boot.
// Returns 0 for a wheel that is offline or didn't answer this time.
double wheels_read_angle(uint8_t index);

#endif // WHEELS_H

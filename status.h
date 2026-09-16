/*
 * Telling the human what is going on: the debug port and the LED strip.
 *
 * Two ways of saying the same thing. status_printf() narrates the boot over
 * USB CDC #2 -- what was found, what answered, what didn't -- and the LED
 * strip breathes blue for as long as the main loop keeps turning.
 *
 * Once zenoh is up the debug port goes quiet (status_quiet()), because in
 * steady state every byte of USB bandwidth belongs to the transport. The
 * LED keeps breathing, so the board still tells you it is alive.
 */

#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>

// Claim the LED strip. USB is already up by this point; see io_begin().
void status_begin(void);

// Print a line to the debug port. For boot narration and state changes --
// not for per-message traffic. Bytes are dropped rather than blocking when
// nobody is reading, so an unopened port can never stall the firmware.
void status_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Stop printing. Called once the session is up; there is no way back, and
// that is deliberate.
void status_quiet(void);

// Advance the breathing animation. Cheap, and only touches the LEDs when
// the brightness actually changes. Call it often.
void status_update(void);

#endif // STATUS_H

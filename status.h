/* Main-loop LED status: yellow while waiting, green while running.
 * Text output is enabled only for imu_diagnostic, on its single CDC port.
 * Normal firmware never writes text into the zenoh transport.
 */

#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>

typedef enum {
    STATUS_WAITING = 0,   // waiting for the ROS router
    STATUS_READY,         // connected and running
} status_mode_t;

// Claim the LED strip, starting in STATUS_WAITING.
void status_begin(void);

// Switch what the strip is saying. Takes effect on the next
// status_update().
void status_set_mode(status_mode_t mode);

// Diagnostic-only non-blocking text output; a no-op in normal firmware.
void status_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Stop diagnostic text output. A no-op in normal firmware.
void status_quiet(void);

// Advance the breathing animation. Cheap, and only touches the LEDs when
// the brightness actually changes. Call it often.
void status_update(void);

#endif // STATUS_H

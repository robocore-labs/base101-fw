#include "io.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "status.h"

void io_begin(void) {
    tusb_init();
    // Bound enumeration wait so boot does not require an attached host.
    uint32_t started = time_us_32();
    while (!tud_ready() && (time_us_32() - started) < 5000000) {
        tud_task();
        sleep_ms(10);
    }
}

void io_poll(void) {
    tud_task();
    status_update(); // Keep LEDs running inside blocking bus/zenoh waits.
}

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "time_sync.h"

int main(void) {
    const int64_t epoch = INT64_C(1700000000000000000);
    time_sync_t clock;
    int64_t stamp = -1;
    time_sync_init(&clock, 20000, 10000000);
    assert(!time_sync_time(&clock, 1000000, &stamp));
    assert(stamp == -1);
    assert(!time_sync_accept(&clock, 1000000, epoch, epoch, 1005000));

    // Two milliseconds each way, one millisecond host processing.
    time_sync_start(&clock, 1000000);
    assert(!time_sync_accept(&clock, 999999, epoch, epoch, 1005000));
    assert(!time_sync_accept(&clock, 1000000, epoch, epoch, 999999));
    assert(!time_sync_accept(&clock, 1000000, epoch + 1, epoch, 1005000));
    assert(!time_sync_accept(&clock, 1000000, epoch, epoch + 6000000, 1005000));
    assert(!time_sync_accept(&clock, 1000000, epoch, epoch, 1020001));
    assert(time_sync_accept(&clock, 1000000,
                            epoch + 1002000000, epoch + 1003000000, 1005000));
    assert(clock.offset_ns == epoch);
    assert(time_sync_time(&clock, 1005000, &stamp));
    assert(stamp == epoch + 1005000000);
    assert(time_sync_time(&clock, 1105000, &stamp));
    assert(stamp == epoch + 1105000000);
    assert(!time_sync_accept(&clock, 1000000, epoch, epoch, 1005000));
    puts("PASS: unsynced clock, reply matching, latency limits, processing compensation");

    assert(!time_sync_time(&clock, 1004999, &stamp));
    assert(time_sync_time(&clock, 11004999, &stamp));
    assert(!time_sync_time(&clock, 11005000, &stamp));
    time_sync_start(&clock, 12000000);
    assert(time_sync_accept(&clock, 12000000,
                            epoch + 12002000000, epoch + 12003000000, 12005000));
    assert(time_sync_time(&clock, 12005000, &stamp));
    assert(stamp == epoch + 12005000000);

    // Resynchronization follows a host wall-clock step; monotonic time is
    // never changed by this module.
    time_sync_start(&clock, 13000000);
    assert(time_sync_accept(&clock, 13000000,
                            epoch + 10002000000, epoch + 10003000000, 13005000));
    assert(clock.offset_ns == epoch - 3000000000);
    puts("PASS: expiry, resynchronization, host clock adjustment");

    time_sync_start(&clock, 14000000);
    assert(!time_sync_accept(&clock, 14000000, 0, 1, 14005000));
    assert(!time_sync_accept(&clock, 14000000, INT64_MAX-1, INT64_MAX, 14005000));
    assert(!time_sync_time(&clock, UINT64_MAX, &stamp));
    time_sync_start(&clock, UINT64_MAX);
    assert(!time_sync_accept(&clock, INT64_MAX, epoch, epoch, UINT64_MAX));
    puts("PASS: invalid timestamps, ROS range, integer overflow guards");
    return 0;
}

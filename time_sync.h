#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stdint.h>

// Offset estimate from a four-timestamp exchange. No Pico or ROS dependency.
typedef struct {
    uint64_t request_us;
    uint64_t last_sync_us;
    uint64_t max_rtt_ns;
    uint64_t timeout_us;
    int64_t offset_ns;
    bool pending;
    bool synced;
} time_sync_t;

void time_sync_init(time_sync_t *clock, uint32_t max_rtt_us, uint64_t timeout_us);
void time_sync_start(time_sync_t *clock, uint64_t local_us);
bool time_sync_accept(time_sync_t *clock, int64_t echoed_us,
                      int64_t host_receive_ns, int64_t host_send_ns,
                      uint64_t local_receive_us);
// False until synced, after expiry, or outside the ROS signed-second range.
bool time_sync_time(const time_sync_t *clock, uint64_t local_us, int64_t *ros_ns);

#endif

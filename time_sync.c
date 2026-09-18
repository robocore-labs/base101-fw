#include "time_sync.h"
#include <limits.h>

#define NS_PER_SECOND INT64_C(1000000000)
#define MAX_ROS_NS ((int64_t)INT32_MAX * NS_PER_SECOND + NS_PER_SECOND - 1)

void time_sync_init(time_sync_t *clock, uint32_t max_rtt_us, uint64_t timeout_us) {
    *clock = (time_sync_t){
        .max_rtt_ns = (uint64_t)max_rtt_us * 1000u,
        .timeout_us = timeout_us,
    };
}

void time_sync_start(time_sync_t *clock, uint64_t local_us) {
    clock->request_us = local_us;
    clock->pending = true;
}

bool time_sync_accept(time_sync_t *clock, int64_t echoed_us,
                      int64_t host_receive_ns, int64_t host_send_ns,
                      uint64_t local_receive_us) {
    if (!clock->pending || echoed_us < 0 ||
        (uint64_t)echoed_us != clock->request_us ||
        local_receive_us < clock->request_us ||
        local_receive_us > (uint64_t)INT64_MAX / 1000u ||
        host_receive_ns <= 0 || host_send_ns < host_receive_ns ||
        host_send_ns > MAX_ROS_NS) {
        return false;
    }
    uint64_t elapsed_ns = (local_receive_us - clock->request_us) * 1000u;
    uint64_t processing_ns = (uint64_t)(host_send_ns - host_receive_ns);
    // Reject delayed responses, including long host scheduling stalls.
    if (elapsed_ns > clock->max_rtt_ns || processing_ns > elapsed_ns) {
        return false;
    }
    uint64_t network_ns = elapsed_ns - processing_ns;
    int64_t half_network_ns = (int64_t)(network_ns / 2u);
    if (host_send_ns > MAX_ROS_NS - half_network_ns) {
        return false;
    }
    // Symmetric-path estimate of host time at board receipt. Host processing
    // is subtracted, rather than mistaken for transport delay.
    int64_t host_now_ns = host_send_ns + half_network_ns;
    clock->offset_ns = host_now_ns - (int64_t)(local_receive_us * 1000u);
    clock->last_sync_us = local_receive_us;
    clock->synced = true;
    clock->pending = false;
    return true;
}

bool time_sync_time(const time_sync_t *clock, uint64_t local_us, int64_t *ros_ns) {
    if (!clock->synced || local_us < clock->last_sync_us ||
        local_us - clock->last_sync_us >= clock->timeout_us ||
        local_us > (uint64_t)INT64_MAX / 1000u) {
        return false;
    }
    int64_t local_ns = (int64_t)(local_us * 1000u);
    if (clock->offset_ns > 0 && local_ns > INT64_MAX - clock->offset_ns) {
        return false;
    }
    int64_t estimate = local_ns + clock->offset_ns;
    if (estimate < 0 || estimate > MAX_ROS_NS) {
        return false;
    }
    *ros_ns = estimate;
    return true;
}

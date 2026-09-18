#!/usr/bin/env python3
"""Respond to board clock probes in the real robot's ROS domain/RMW."""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, UInt64


class TimeSyncHost(Node):
    def __init__(self):
        super().__init__("axon_time_sync")
        # Real robot wall clock, independent of use_sim_time or /clock.
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.response = self.create_publisher(
            Int64MultiArray, "/axon/time_sync/response", 1)
        self.request = self.create_subscription(
            UInt64, "/axon/time_sync/request", self.on_request, 1)
        self.get_logger().info("Serving Axon clock probes using host system time")

    def on_request(self, request):
        received_ns = self.wall_clock.now().nanoseconds
        response = Int64MultiArray()
        response.data = [request.data, received_ns, 0]
        response.data[2] = self.wall_clock.now().nanoseconds
        self.response.publish(response)


def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncHost()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

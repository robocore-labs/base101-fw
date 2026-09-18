/* ROS interface over zenoh on USB CDC #0.
 * Receives cmd_vel (TwistStamped), commands wheel targets and publishes IMU data.
 * Stale commands actively brake using monotonic time. IMU stamps require
 * fresh host clock synchronization. Publishes encoder-only wheel odometry.
 * Topic names, rates and frame IDs are in robot.h.
 */

#ifndef ROS_H
#define ROS_H

#include <stdbool.h>

// Connect to the zenoh router, declare the node, and create every publisher
// and subscriber. Blocks until the router shows up -- USB
// keeps running while it waits. Returns false only if the node itself could
// not be declared.
bool ros_begin(void);

// Receive whatever arrived (command callbacks fire from in here), then
// publish whatever is due. Call it every time round the main loop.
void ros_update(void);

#endif // ROS_H

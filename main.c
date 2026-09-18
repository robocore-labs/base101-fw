/*
 * base101 firmware — a ROS 2 node on a Link101 board (RP2350A).
 * ============================================================
 *
 * The robot's motor control runs here, on the board, and the host talks to
 * it in ROS: /cmd_vel commands the drive wheels, and IMU data
 * is published over zenoh. There is no bridge process on
 * the host and no serial protocol to speak -- with rmw_zenoh running, this
 * board is just another node in the graph.
 *
 * USB exposes one CDC port, exclusively for zenoh. Status is shown by LEDs.
 * Four PIO UARTs drive the DDSM210 wheels; i2c1 hosts the onboard IMU.
 *
 * Everything you would want to change -- pins, motor IDs, joint names,
 * rates, topic names -- is in robot.h. Start there.
 *
 * Read the rest in this order: main.c for the shape, then robot.h for the
 * robot, then whichever of wheels.c / imu.c / ros.c
 * you care about. Each one is about a hundred lines and stands alone.
 */

#include "pico/stdlib.h"

#include "imu.h"
#include "io.h"
#include "robot.h"
#include "ros.h"
#include "status.h"
#include "wheels.h"

// Bring the robot up. Missing wheels and sensors are skipped.
static void setup(void) {
    io_begin();        // Bring up the sole USB CDC port for zenoh.
    status_begin();

    status_printf("\n\n=== base101 firmware ===\n");

    uint8_t found = wheels_begin();
    status_printf("[wheel] %u of %u wheels answered\n", found, WHEEL_COUNT);
    // Start stopped, actively -- not in whatever state the motors happened
    // to power up in. Matters most for a robot that lost power mid-drive:
    // without this, a wheel could resume its last remembered speed the
    // instant wheels_begin() puts it back in velocity mode.
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        wheels_brake(i);
    }



    imu_begin();

    // Blocks until the router is up -- the strip blinks yellow throughout,
    // which shows that initialization is waiting for the router.
    // USB keeps running while it waits, so the board stays
    // usable however long that takes.
    if (!ros_begin()) {
        status_printf("[ros  ] FATAL: could not declare the node\n");
        while (true) {
            io_poll();
            status_update();
        }
    }

    // Connected; stay yellow until stationary gyro calibration succeeds.
    status_set_mode(STATUS_WAITING);


}

// Receive commands, publish sensor data, service USB, and breathe.
static void loop(void) {
    ros_update();
    wheels_update();   // shape body velocity and send wheel setpoints
    imu_update();
    io_poll();
    status_update();
}

int main(void) {
    stdio_init_all();

    setup();
    while (true) {
        loop();
    }
}

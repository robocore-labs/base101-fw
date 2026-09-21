# base101 firmware

Firmware for the Link101 board (RP2350A), running a ROS 2 node over USB
zenoh through Pico-ROS and zenoh-pico. Robot configuration is in `robot.h`.

## Current ROS interface

| Direction | Topic | Type | Behavior |
|---|---|---|---|
| sub | `/link101/cmd_vel` | `geometry_msgs/TwistStamped` | Body velocity: twist.linear.x in m/s, twist.angular.z in rad/s. |
| pub | `/link101/imu` | `sensor_msgs/Imu` | Acceleration and angular velocity, up to 50 Hz. |
| pub | `/link101/imu/mag` | `sensor_msgs/MagneticField` | Magnetic field in tesla. |
| pub | `/link101/imu/temperature` | `sensor_msgs/Temperature` | IMU die temperature in Celsius. |
| pub | `/link101/imu/status` | `std_msgs/String` | Sensor initialization, identity probes, sync and read/publish counters, 1 Hz. |

The `twist` body velocity in cmd_vel is allocated as `(linear.x - angular.z * separation / 2) / radius`
for the left wheels and `(linear.x + angular.z * separation / 2) / radius`
for the right wheels. Other velocity components and the header frame are ignored; commands must be
expressed in the robot body frame. The watchdog uses local reception time,
not header timestamps, so clock adjustments do not affect motion timeout. Radius is 0.0363 m
and separation 0.2886 m, matching the hardware controller config. Excessive
wheel targets are scaled together to preserve the requested turn ratio.

Body velocity is shaped in software before wheel allocation, at up to 50 Hz
using actual monotonic elapsed time. Tested linear settings are 0.7 m/s²
acceleration and 2 m/s³ jerk; angular limits are 3 rad/s² and 10 rad/s³ and
still need ground testing. Hardware acceleration byte 1 lets the motor follow
the shaped targets. Normal zero commands taper down through the same profile.
Shaped wheel rates are uniformly saturated at the speed cap. A sudden target
change may briefly retain previous acceleration while the jerk limit reduces
it; explicit electric braking bypasses shaping. Non-finite/overflowing commands
brake immediately. A profile timing gap over 100 ms during active motion also
brakes and clears the profiles. Silence for
500 ms triggers active braking, with brake frames repeated at 5 Hz while
stale. Fresh commands resume from the braked state. The watchdog runs in the
cooperative main loop, so blocking communications can delay its response.

Glide now shapes body commands once, then closes yaw rate using the gyro.
The existing tested acceleration/jerk limits are retained. Effective-track
feedforward starts at `GLIDE_ICR_COEFF 1.5`, estimated from the wood-floor turn;
PI feedback is currently disabled (`GLIDE_YAW_FEEDBACK_ENABLED false`) for a
floor comparison after continuous jerk was observed with feedback enabled.
Calibration, gyro publication, feedforward and the operator ramps remain active.
When re-enabled, PI correction passes through its own acceleration/jerk limits
before wheel allocation, including a smooth return to zero in COAST mode. Gains, correction
cap, deadband and low-pass frequency are in `robot.h`. Uniform wheel saturation
preserves curvature, and the integrator freezes at saturation or after 200 ms
of excessive motor tracking error (it can still unwind). Once a zero command
has settled, COAST disables correction so the robot does not fight being pushed.
Watchdog braking clears glide state and preserves the calibrated gyro bias.
Stale gyro over 50 ms, failed motor replies or missing/faulted wheel feedback
actively brake all wheels; subsequent motion needs a fresh command and inputs.
Yaw control runs at the existing 50 Hz motor tick; IMU sampling requests 208 Hz
on the cooperative main loop. This implementation has no FIFO or second core.

On each ROS firmware startup, wheels remain actively braked while a continuous
15-second stationary window estimates body-Z gyro bias. The window begins with
the first valid stationary samples after ROS connects; USB and ROS remain live.
Commands received before calibration completes are ignored, never queued.
Keep the entire robot still. All four wheel replies must be fresh and near zero,
gravity magnitude plausible, and gyro axes quiet. Movement, failed reads or
sample gaps restart the window. The window needs at least 750 samples, yaw
standard deviation <=0.003 rad/s and absolute bias <=0.05 rad/s. Rejected windows
retry while motion remains blocked. Bias is held in RAM and remeasured each boot.
No heading or absolute yaw is calibrated. Gyro sign +1 was verified using a
positive-turn pulse before enabling the feedback loop.

`/link101/imu/status` reports `yaw_cal=keep_still` or `ready`, `cal_s`, `cal_samples`,
`cal_rejected`, `bias`, `gyro_fresh`, `yaw_rate`, `yaw_corr` and `saturated`.
LEDs stay yellow until calibration is ready and gyro data is fresh, then green.
`/link101/imu` and temperature publication wait for calibration; mag publication remains
independent. `/link101/imu` Z angular velocity is the same corrected/filtered yaw rate
used by glide; other gyro axes remain raw. To recalibrate without rebooting:

```bash
ros2 topic pub --once /link101/gyro_calibrate std_msgs/msg/Bool '{data: true}'
```

This brakes all wheels and starts a new stationary window without resetting pose.

Odometry publishes `/link101/odom/raw` (`nav_msgs/msg/Odometry`) at 50 Hz,
with `odom` as the parent and `base_link` as the child. It is deliberately raw:
linear and angular velocity come only from the front-left and front-right motor
encoder speeds, corrected for motor polarity, wheel radius, effective separation
and `k_icr`. Midpoint heading and trapezoidal velocity integration produce a
basic wheel pose. The IMU does not enter this estimate. Both front-wheel replies
must be less than 100 ms old; missing inputs or integration gaps over 50 ms break
the interval without stale extrapolation.

The firmware does not publish TF. A host EKF should fuse `/link101/odom/raw` with
`/link101/imu`, publish the fused `/odom`, and be the sole publisher of
`odom -> base_link`. `/link101/odom/reset` (`std_msgs/msg/Bool`, `data: true`)
resets only the raw wheel pose without changing commands or gyro calibration.
Pose yaw covariance is 0.1 and twist yaw-rate covariance is 0.05 to reflect skid.
All stamped publications require fresh host clock synchronization; sampling and
integration continue without it.

Wheel drive responses supply feedback without additional queries during motion.
While braked, 50 Hz brake frames return measured speeds without releasing the
brake. The command watchdog still checks each loop and repeats active brakes at
5 Hz. Wheels are initialized and actively braked at startup. The old wheel-array commands, arm commands,
joint states and motor telemetry are excluded from the active ROS interface.

Arm support is preserved in `attic/arm/`, including servo sources and snapshots
of the previous ROS interface and robot configuration. `pico_feetech` remains
on disk as a submodule but is neither built nor linked.

The accelerometer and gyroscope sample internally at 208 Hz. Firmware requests
the latest register sample independently at 208 Hz; ROS
publishing remains at 50 Hz and uses the cached sample with its acquisition
stamp. The cooperative loop and blocking motor transactions can lower the
actual sampling rate. There is no FIFO: intermediate samples are not buffered
or averaged. The magnetometer remains at 100 Hz.

IMU timestamps use synchronized host system time (see Clock synchronization below).
`/link101/imu` has no orientation estimate
(`orientation_covariance[0] = -1`); acceleration includes gravity. Sensor
initialization failures are logged, and missing sensors publish no readings.

USB exposes one CDC port. In normal firmware it carries only zenoh; text logs
are disabled and LEDs show status. The standalone `imu_diagnostic` instead
uses its single port for continuous sensor readings. The previous lidar USB
passthrough is preserved in `attic/lidar/` and is not built or initialized.

To diagnose missing IMU data, run `ros2 topic echo /link101/imu/status`.
This status topic does not require clock synchronization. WHO_AM_I `0x6C`
is expected at 0x6B or 0x6A; identity probes refresh after each initialization attempt.
`lsm6dsox=offline` means initialization failed. An online sensor with increasing
`reads_failed` has sampling transaction failures. `reads_ok` increasing means
sampling succeeds; `imu_publish_failed` then distinguishes publication errors.
An offline LSM6DSOX retries configuration once per second, without bus clearing
or software sensor resets. Status includes init_attempts.

## Clock synchronization

Run `tools/time_sync_host.py` alongside the real robot ROS stack, with the same
ROS domain and RMW configuration. It uses host system time; this is not a
simulation `/clock` bridge. Ensure the host itself has the desired time source
(e.g. NTP). For the drive container, after starting it:

```bash
docker cp tools/time_sync_host.py base101-drive-1:/tmp/link101_time_sync.py
docker exec -it base101-drive-1 bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /tmp/link101_time_sync.py'
```

For regular use, launch this helper with the robot stack. Only one time-server
instance should serve a board. It adds these topics:

| Direction at firmware | Topic | Type | Payload |
|---|---|---|---|
| pub | `/link101/time_sync/request` | `std_msgs/UInt64` | Board transmit time, monotonic microseconds. |
| sub | `/link101/time_sync/response` | `std_msgs/Int64MultiArray` | `[echoed_board_us, host_receive_ns, host_send_ns]`, empty layout. |

The board probes once per second, subtracts host processing time from the
round trip, and assumes symmetric transport to estimate the clock offset.
Unmatched, duplicate, invalid, or round-trip-over-20-ms responses are rejected.
Actual accuracy depends on path asymmetry and scheduling; it is not yet
measured on the robot. The host's ROS signed-second range is enforced.

IMU, magnetic-field and temperature messages pause until first synchronization
and if no acceptable sample arrives for 10 seconds. They resume automatically
on synchronization. The standalone IMU diagnostic remains independent of this
helper. Drive commands and the stale-command brake continue using monotonic
reception time, even while unsynchronized or if host wall time changes.
Stamped command headers are not used to determine command freshness yet.

Run the portable clock tests without the Pico SDK:

```bash
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -I. time_sync.c tests/time_sync_test.c -o /tmp/base101-time-sync-test
/tmp/base101-time-sync-test
```

## Hardware

| Bus | Pins | Notes |
|---|---|---|
| Wheels | GP21–28 | Four PIO UARTs, 115200. One motor per port — a DDSM210 can't share a TX line. |
| Lidar | TX 4, RX 5 | hardware `uart1`, 460800 (RPLidar C1) |
| IMU | SDA 14, SCL 15 | `i2c1`: LSM6DSOX at 0x6B, MMC5983MA at 0x30. Both soldered to the board; the Qwiic connector is the same bus. |
| LED strip | GP18 | six WS2812 pixels |

PIO state machines are claimed at initialization: eight for wheel UARTs and
one for the LEDs. The archived servo bus is not initialized.

USB (VID:PID `1209:AC01`) presents one port: `RoboCore Link101 Zenoh` in normal
firmware, or `RoboCore Link101 Debug` in the diagnostic image. The udev rules
below provide the corresponding stable name.

The normal firmware also treats the time-sync exchange as a host heartbeat.
The watchdog remains disabled until the first valid host response. After that,
60 seconds without a valid response
brakes every wheel and reboots the RP2350, forcing USB and Zenoh to establish a
fresh session when the Docker stack returns. Transport errors alone never
trigger a reboot.

## Building

Needs the [pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.x and an
`arm-none-eabi` toolchain.

```bash
git submodule update --init          # the libraries in lib/
export PICO_SDK_PATH=~/pico/pico-sdk
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build -j$(nproc)
```

Flash by holding BOOT while plugging in and copying `build/base101_firmware.uf2`
to the `RPI-RP2` drive, or `picotool load -f build/base101_firmware.uf2`.

Normal firmware has no USB boot log; its only port belongs to zenoh.
The LED strip shows startup and loop status:

| Strip | Means |
|---|---|
| fast yellow blink | waiting for the ROS router — nothing else can start |
| slow green breath | connected and running |
| frozen | the main loop stopped turning |

It is driven from the main loop and from inside every blocking wait, so it
keeps moving even while the board waits for a router that isn't up yet.
Colours and rates are in `robot.h`.

## Standalone IMU diagnostic

Build a firmware image that prints onboard sensor data without a ROS router:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target imu_diagnostic -j4
```

Hold BOOT while plugging in, then copy `build/imu_diagnostic.uf2` to the
`RPI-RP2` drive. This replaces the normal firmware; flash
`base101_firmware.uf2` again to restore robot operation.

Read the diagnostic's sole USB port with `screen /dev/link101-debug 115200`
(or its `/dev/ttyACM*` device if udev rules are absent). This image replaces
the zenoh interface with diagnostic text; normal firmware has no debug port.
Motors and ROS are never initialized. Detection status repeats every second;
acceleration, gyroscope, temperature and magnetometer readings print at 10 Hz.
Offline sensors and failed reads are reported explicitly. Reboot to retry
initialization. The production IMU helper returns 0 C if the separate
temperature read fails.

At rest, acceleration magnitude should be about 9.81 m/s² and gyro readings
near zero. Tilt the board to move gravity between acceleration axes; rotate
it to change gyro readings. Magnetic readings are in microtesla and may be
strongly affected by nearby motors or magnets. Capture the boot log for
address/WHO_AM_I diagnostics if the accelerometer/gyro stays offline.

## Host setup

### 1. udev rules

```bash
./install.sh
```

gives the single port a stable symlink:

- Normal firmware: `/dev/link101-zenoh` for zenoh.
- Diagnostic firmware: `/dev/link101-debug` for sensor readings.

Re-run the installer after updating the rules and reconnect the board.

### 2. zenoh router with serial transport

The firmware is a zenoh **client** connecting through its serial link, so the
host must run a router listening on that port. Serial transport is **not
enabled in stock zenohd builds** — you need one compiled with
`--features transport_serial`.

Easiest is the Docker image in [`docker/`](docker/), which rebuilds zenohd
with that feature on top of `eclipse/zenoh:latest`:

```bash
cd docker
docker compose up --build      # maps /dev/link101-zenoh, listens serial + tcp/7447
```

From source instead:

```bash
git clone https://github.com/eclipse-zenoh/zenoh && cd zenoh
cargo build --release -p zenohd --features transport_serial
# the baudrate token is required by the locator syntax, and meaningless on USB CDC
./target/release/zenohd -l 'serial//dev/link101-zenoh#baudrate=921600'
```

Keep the router protocol-compatible with the vendored zenoh-pico (currently
**1.9.0**, see `lib/zenoh-pico/include/zenoh-pico.h`).

### 3. ROS 2 with rmw_zenoh

```bash
# option A: single router — also listen on the tcp port rmw_zenoh expects
zenohd -l 'serial//dev/link101-zenoh#baudrate=921600' -l 'tcp/[::]:7447'

# option B: keep rmw_zenohd, and federate the serial router into it
zenohd -l 'serial//dev/link101-zenoh#baudrate=921600' -e 'tcp/localhost:7447'
```

Then, with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`:

```bash
ros2 topic list
ros2 topic echo /link101/imu
# Continuous low-speed command; replace linear.x with 0.0 to test a ramped stop.
ros2 topic pub -r 20 /link101/cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}'
```

## Libraries

Everything below the robot is a library, pulled in as a submodule under
`lib/`:

| Library | What it does |
|---|---|
| [`pico_serial`](https://github.com/robocore-labs/pico_serial) | The `serial_t` interface every driver speaks, and `serial_hook.h`. |
| [`pico_cdc_serial`](https://github.com/robocore-labs/pico_cdc_serial) | A USB CDC interface as a `serial_t`. The one place USB stops. |
| [`hardware_link101`](https://github.com/robocore-labs/hardware_link101) | The board: PIO and UART ports, pin map, LED strip, CAN, and the onboard LSM6DSOX + MMC5983MA. |
| [`pico_feetech`](https://github.com/robocore-labs/pico_feetech) | Archived arm dependency; not built or linked. |
| [`pico_ddsm`](https://github.com/robocore-labs/pico_ddsm) | DDSM210 wheel motors. |
| [`pico_zenoh`](https://github.com/robocore-labs/pico_zenoh) | zenoh-pico for bare metal, over any `serial_t`. |
| [`easypicoros`](https://github.com/robocore-labs/easyp) | Typed ROS publishers and subscribers on top of Pico-ROS. |

zenoh-pico, picoros and micro-CDR stay vendored in `lib/` rather than
submoduled: they are upstream projects we track, and picoros carries a local
patch (a `user_data` pointer on `picoros_subscriber_t`, without which a typed
subscriber facade is impossible — easypicoros' build checks for it).

### How the pieces fit

Every bus in the firmware is a `serial_t`, and the drivers take one. So a
wheel motor and the zenoh link are all the same kind of
thing, and nothing below the application knows what USB is.

The interesting part is **`io_poll()`**. Drivers block waiting for a reply and
call `serial_task()` while they wait, so every bus is wrapped with
`serial_hook_init()` to run `io_poll()` there: USB and the
LED keep going *inside* a wheel transaction or zenoh waiting for a
router that isn't up yet. One hook, everywhere, with one rule — `io_poll()`
must never touch a wrapped bus or call into zenoh, or it would be calling
itself.

### Standalone motor terminal

Build with `picobuild`, then flash **explicitly** with
`picoflash build/motor_terminal.uf2`. The single USB CDC identifies as
`Link101 Motor Terminal` / `RoboCore Link101 Debug`; ROS is not linked into this target.
Connect with `python3 tools/motor_terminal.py /dev/ttyACM0` (use the actual port).
`help` prints the command list; `quit` or Ctrl-C in this client sends electric brake.

Commands: `hw 100` sets the motor ramp to 10 ms/RPM; larger values are gentler.
`sw 6` sets the software ramp in rad/s²; `sw 0` isolates the motor's own ramp.
`wheel all` or `wheel 0` selects all wheels or FL=0, FR=1, BL=2, BR=3 at rest.
`run 10 10 2` requests physical left/right RPM for two seconds, then ramps to zero.
A new `run` during a run can test reversal. Experiments are bounded to ten seconds
and ±200 RPM. `stop` ramps to zero; `brake` or `!` applies electric braking.
The v2 terminal defaults to `mode shape`, hardware acceleration byte 1,
and body limits `shape 0.7 2 3 10`: linear acceleration 0.7 m/s², linear jerk
2 m/s³, yaw acceleration 3 rad/s² and yaw jerk 10 rad/s³. These are tuning
candidates, not validated driving settings. Shaping uses a velocity tracker
with bounded acceleration and jerk, tapering acceleration near the target.
Sudden target changes can briefly continue the previous acceleration while
jerk limiting brings it down; this is not an emergency-stop trajectory.
`run` still takes left/right RPM and converts them into body velocity before
shaping. All four motors must be online for body-space experiments.

Use `mode hw` to bypass all software shaping, `mode linear` for the old
per-wheel ramp, and `mode shape` for body-space jerk limiting. `sw 0` selects
hardware-only mode; `sw 6` selects the old ramp. Change modes, hardware ramp
and shaping limits only at rest. The shaped mode reports body velocity,
acceleration, yaw rate, yaw acceleration and actual update interval.
The shaper uses actual elapsed time; a control gap over 100 ms brakes and
latches a fault. Acceleration and jerk are independently tunable with
`shape AX JX AW JW` (limits: AX 0.01–5, JX 0.01–50, AW 0.01–20,
JW 0.01–200, all SI units).
Feedback reports target, command and measured physical RPM, echoed acceleration,
temperature and error flags at 5 Hz. Missing replies or motor errors brake all
online motors and latch a fault until reboot. USB removal brakes immediately.
No movement experiment runs automatically at startup.

### Detailed IMU diagnostic

`imu_diagnostic` v8 waits for the CONFIG button (GP17, active low, 30ms
debounce) before initializing or accessing I2C. Open its sole debug serial
port first, then press the button to capture the entire trace. A held button
at boot must be released and pressed again. The waiting message repeats
once per second, with the build identifier.

The diagnostic defaults to I2C1 at **100 kHz**. The magnetometer is completely
excluded from initialization and sampling, including identity probes. Motor
and ROS initialization are also excluded. This isolation is diagnostic-only;
the ROS firmware still publishes its magnetometer readings.

Initialization performs ordinary I2C controller/pin setup, identity reads and
sampling configuration, with every LSM6DSOX register transaction logged.
No GPIO clock pulses, manually generated STOP, controller deinitialization,
software sensor reset or reset polling is performed.
Transaction logs include direction, address, register, requested size, return
count, elapsed microseconds and read value. Negative counts are SDK errors.
The driver is the same production source compiled with diagnostic tracing;
verbose transactions are enabled only during initialization, including retries.
Serial output drains with a bounded wait instead of dropping full-FIFO writes.

After initialization, live WHO_AM_I probes and health counters repeat at
1 Hz, and IMU/temperature readings print at 10 Hz. Commands: `probe`, `regs`
(CTRL1_XL, CTRL2_G, CTRL3_C at both addresses; expected 0x58, 0x54, 0x44),
`scan` (one-byte reads over nonreserved addresses, skipping magnetometer 0x30),
and `retry` (ordinary I2C initialization and LSM6DSOX configuration; resets counters).
`rate 100000` or `rate 400000` reinitializes at the selected rate; all
subsequent probes, scans and automatic configuration retries preserve that rate. Flash explicitly:
`picoflash build/imu_diagnostic.uf2`.

`samples` compares registers 0x22–0x2D as twelve independent one-byte reads,
2/6/12-byte bursts with the former 1ms timeout, and a 12-byte burst with a
5ms timeout. It also checks temperature separately. Results include both I2C
phase return counts, elapsed time and received bytes; individual transactions
are not a coherent IMU sample. This comparison runs automatically after the
first online sample failure per initialization attempt. Failed production-driver
sample transactions are also traced with address/register context. At 100kHz,
a 12-byte read needs over 1ms on the wire. Hardware traces confirmed the 1ms
budget failed and the 5ms budget succeeded, so the shared LSM6DSOX driver now
uses a bounded 5ms timeout. No resets are added.

## Calibration UI

The separate `calibration_firmware` target and Python3 server provide a browser
UI with stationary gyro calibration, hold-to-drive controls, independent
operator/server watchdogs, live telemetry, 19 tuning sliders, bounded wheel/spin/
braking/reference trials, MCU transient measurements, repeated-run candidates,
and persistent results. Yaw PI starts disabled for the quiet baseline. See
[tools/calibration/README.md](tools/calibration/README.md) for the explicit UF2,
server command, controls, and API. The guided calibration plan is in
[CALIBRATION_UI_PLAN.md](CALIBRATION_UI_PLAN.md).


The calibration UI can run each step's subtests automatically. At the end,
**Save calibration.h** exports all accepted settings as a build input for the ROS
firmware. Rebuild base101_firmware and flash build/base101_firmware.uf2 to keep
those settings across reboots.

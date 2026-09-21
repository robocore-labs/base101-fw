# base101 firmware session handoff — 2026-09-17

## Where we stopped

The user asked to save context and stop for the day. Do not run motors on resume
without establishing that the user is watching and the robot has clear space.
The user normally flashes firmware themselves. Do not flash automatically.

**Current robot image:** latest PI-disabled comparison build, flashed by user.
The user reports it is incredibly smooth. Calibrated gyro measurement and
ICR feedforward remain active. The prior PI-enabled image achieved a visually
correct half-turn but was extremely jerky throughout ordinary driving.

**Latest source and built image:** PI feedback disabled for an isolation test,
with a new acceleration/jerk-limited correction path ready for re-enabling PI.
This image is now confirmed flashed. The user reports smooth motion; a new
gyro-target half-turn completed with 179.88 degrees gyro / 306.74 degrees encoders.
All firmware targets build successfully, and portable/integration checks pass.

Next flash, when the user resumes:

```bash
picoflash build/base101_firmware.uf2
```

Keep the robot completely still during the startup bias window. Wait for green
LEDs and `/imu/status` containing `yaw_cal=ready gyro_fresh=1` before testing.
The calibrated gyro can still measure a half-turn with PI disabled.

## Workspace and workflow

- Firmware workspace: `/home/cdr/working/base101-fw`.
- Robot ROS tree: sibling `/home/cdr/working/base101`; user manages its changes.
- ROS runs in Docker container `base101-drive-1`, Jazzy, rmw_zenoh.
- Router container: `zenoh_router`. Both containers have been restarted during
  the session; fresh ROS graph entries do not guarantee live stamped messages.
- Source ROS in Docker: `/opt/ros/jazzy/setup.bash`, optionally
  `/ros_ws/install/setup.bash`.
- Board serial: usually `/dev/ttyACM0`. Normal firmware serial is binary zenoh;
  never open it with a text reader while router owns it.
- User uses `python3 -m serial.tools.miniterm` for standalone diagnostic text.
  Type a command plus Enter; Ctrl+T then E toggles local echo.
- Convenience scripts: `picobuild`, `picoflash`. Always pass the UF2 filename
  explicitly: multiple targets exist and a default newest-UF2 choice can be wrong.
- Many changes were already uncommitted before odometry/glide work. Do not reset
  or discard them. Libraries under `lib/` are Git submodules and contain edits.
- No agent commits, staging, or flashing were done in this session.
- Managed sandbox can fail with `bwrap: loopback: Failed RTM_NEWADDR`. Request
  escalation for the failed operation when policy permits; do not bypass policy.

## Hardware and IMU investigation

Board uses RP2350A, four DDSM210 motors, and I2C1 on GP14 SDA / GP15 SCL.
LSM6DSOX accel/gyro: expected WHO_AM_I 0x6C, primary address 0x6B,
alternate 0x6A. MMC5983MA magnetometer: address 0x30.

Initially IMU worked standalone but ROS reported offline, particularly with
motor power on. User suspected UART backpowering from motors keeping MCU/IMU
powered. Power-arrangement changes did not resolve the failure. Motors were
stationary; simply powering them was enough to affect the IMU.

Two distinct issues were established:

1. **Sampling timeout:** at 100 kHz, the shared driver had a 1000 us timeout
   for a 12-byte gyro+accel burst. Single-byte, 2-byte and 6-byte reads worked;
   the 12-byte burst timed out. A 5000 us burst succeeded (~1478 us including
   register-address phase). Shared LSM6DSOX driver timeout is now 5000 us.
   User confirmed sampling works. This timeout fix is in ROS too.
2. **Pin conflict:** inspecting hardware files revealed GP19 routes to IMU
   INT1 through JP1, and GP20 routes to magnetometer interrupt through JP2.
   The original front-right motor UART occupied GP19/GP20. User moved this
   motor wiring to ADC1/ADC2 and confirmed the IMU works with motors powered.
   Pin reassignment is integrated into ROS and motor terminal targets.

Hardware repository inspected:
https://github.com/robocore-labs/link101-hardware

Carrier U3 LSM6DSOX INT1 pin 4 goes through JP1 to INT_IMU / GP19; INT2 pin 9
is unconnected. Repository specifies solder jumpers normally open; actual bridge
state was never independently verified. Avoid claiming which jumper was bridged.

**Separate schematic defect:** U3 CS pin 12 is marked unconnected in carrier
schematic/PCB. ST requires CS tied high to VDDIO for I2C. This is a real design
issue if the assembled board matches the repository. A physical CS repair was
NOT confirmed. Moving the motor UART resolved the observed powered-motor issue;
do not attribute that resolved issue solely to floating CS.

User explicitly requested removal of bus-reset/recovery complexity:
- Ordinary I2C initialization only; no GPIO bus-clear pulses/deinit recovery.
- Shared LSM6DSOX init no longer performs software reset/polling.
- Offline IMU retries register configuration once per second without bus reset.
- Normal ROS I2C: 400 kHz. Standalone diagnostic defaults 100 kHz.

`imu_diagnostic` v8:
- Waits for CONFIG GP17 active-low, release then press, 30 ms debounce.
- Does not initialize I2C before the button, so terminal sees entire init.
- Extremely verbose transaction logging during initialization and on failures.
- Magnetometer entirely disabled in this target, not just unpublished.
- Commands: `probe`, `regs`, `scan`, `samples`, `retry`,
  `rate 100000`, `rate 400000`, `help`.
- `samples` compares byte and burst lengths, former 1 ms versus corrected 5 ms.
- `scan` skips magnetometer 0x30; no resets/configuration writes.
- Trace wrapper compiles actual shared LSM6DSOX source with diagnostic I2C wrappers.

## Active motor wiring, geometry and polarity

One DDSM210 per PIO UART, 115200 baud, factory ID 1 on every independent bus.

| Wheel | MCU TX | MCU RX | Motor polarity | Side |
|---|---|---|---|---|
| Front left | GP21 | GP22 | +1 | left |
| Front right | GP27 / ADC1 | GP28 / ADC2 | -1 | right |
| Back left | GP25 | GP26 | +1 | left |
| Back right | GP23 | GP24 | -1 | right |

Lower-listed pin is MCU TX to motor RX. Original polarity was left -1/right +1.
ROS tests showed odom signs inverted relative to commands; user confirmed positive
commands physically drove backward. We reversed all motor polarities and removed
an extra odometry inversion. Commands and feedback now share physical polarity;
`ODOM_ENCODER_SIGN` is +1. Physical forward and ~5 cm travel were confirmed.

Radius: 36.3 mm (72.6 mm outside tyre diameter), inherited from robot config.
Original separation: 288.6 mm. User measured 307 mm outside-to-outside span and
17.10 mm wheel width in Fusion. Centre-plane separation = 307 - 17.10 = 289.90 mm.
Latest `WHEEL_SEPARATION_M` is 0.2899. This ~0.45% correction cannot explain
large encoder turn errors. Surface is wood with very sticky rubber tyres.

DDSM210 velocity feedback is signed 0.1 RPM, feedback slot 1 = 0x01.
Feedback conversion: raw * 2*pi/600 * wheel polarity * radius -> m/s.
Speed cap 200 RPM; motor protocol maximum 210 RPM.

## Motion shaping before gyro feedback

Hardware-only ramps were extremely quiet, but much slower than the documentation
suggested and unresponsive. Aggressive hardware ramps or simple software ramps
caused clicks/choking/vibration. User observed standalone motor-terminal tests.

Portable `motion_profile.c/h` implemented body-space jerk-limited tracking:
- Linear acceleration 0.7 m/s^2, jerk 2.0 m/s^3.
- Yaw acceleration 3.0 rad/s^2, jerk 10.0 rad/s^3 (less physically validated).
- Actual monotonic elapsed time, internal <=5 ms integration steps.
- Motor hardware acceleration byte 1, fast internal motor velocity loop remains.
- Uniform wheel saturation. Normal zero uses shaped velocity commands, not brake.
- Startup/watchdog/invalid inputs use active electric brake.
- Profile gaps >100 ms during active motion brake/reset.

User confirmed quiet, smooth +/-100 RPM forward/reverse/stop shaping in terminal.
`motor_terminal` remains standalone, independent of ROS/sensors. It supports
hardware, old linear and shaped modes, limits, durations and feedback logging.
Tools include `tools/motor_terminal.py`.

Command watchdog: 500 ms from monotonic reception time, checks each ROS loop,
repeats active brake at 5 Hz. Brakes clear shaping/controller state and latch
until a fresh command. Cooperative blocking communications can delay checks.

## ROS interface and clock synchronization

One USB CDC only. Extra debug/lidar serial interfaces were removed.
ST3215/arm, old servo telemetry and legacy ROS scaffolding preserved in
`attic/arm`; not linked. Lidar passthrough preserved in `attic/lidar`; not built.

Active topics:
- Subscribe `/link101/cmd_vel`: geometry_msgs/msg/TwistStamped, use linear.x/angular.z.
- Publish `/link101/imu`, `/link101/imu/mag`, `/link101/imu/temperature`, `/link101/imu/status`.
- Publish `/link101/odom/raw`: nav_msgs/msg/Odometry, front-wheel encoder-only.
- Firmware publishes no TF; the host EKF owns odom -> base_link.
- Subscribe `/link101/odom/reset`: std_msgs/msg/Bool, true resets raw pose only.
- Subscribe `/link101/gyro_calibrate`: std_msgs/msg/Bool, true brakes all wheels
  and restarts stationary calibration, without resetting odometry pose.

TwistStamped header is not used for command freshness; watchdog uses reception.
IMU orientation covariance[0] remains -1; no attitude/orientation fusion.

Clock exchange:
- Firmware `/link101/time_sync/request`: UInt64, MCU monotonic microseconds, 1 Hz.
- Host `/link101/time_sync/response`: Int64MultiArray, empty layout, data_offset 0,
  exactly [echo_us, host_receive_ns, host_send_ns]. Host system wall time.
- Midpoint estimate, reject RTT >20 ms; synchronization expires after 10 s.
- Stamped publication requires fresh valid sync. IMU sampling and odometry
  integration continue independently of sync. `/link101/imu/status` always publishes.
- Host node is `/base101_time`; helper specification/example in
  `tools/time_sync_host.py`. Zenoh router clocks do not replace this exchange.

**Unresolved synchronization issue:** following stack/router restarts, firmware
sometimes reports `sync=waiting` indefinitely while requests/responses flow and
IMU reads remain healthy. Host-side request/response observation was ~0.7-1.1 ms,
but this does not establish board RTT or prove callbacks accepted responses.
Router logged Unknown remote interest errors. Exact cause not established.
Restarting firmware with the host stack already up restored sync. Do not silently
increase RTT tolerance; add acceptance/rejection/callback diagnostics if it recurs.

To command tests, use `/cmd_vel_agent` through existing `twist_mux`, not a second
publisher directly on `/cmd_vel`:
- Agent priority 50, joystick `/cmd_vel_joy` 100, keyboard `/cmd_vel_key` 90,
  navigation `/cmd_vel_nav` 10, source timeout 0.5 s.
- User can stop/override with higher-priority teleop; test aborts on competing
  applied commands. Do not take over higher-priority joystick input.
- Publish test commands at 20 Hz, send zeros for several seconds after each
  phase and on exceptions, destroy publisher when done.
- Ensure fresh IMU/odom as appropriate and stationary baseline before movement.

## Odometry decisions and measured results

Initially added wheel translation + gyro yaw according to step D in `specs.md`.
User saw RViz yaw drifting at rest and explicitly requested pure encoder odom.
That request persists: DO NOT switch odometry back to gyro without authorization.

Current odometry:
- Average front/rear measured speed per side, vx=(left+right)/2.
- wz=(right-left)/physical separation; vy=0 nonholonomic assumption.
- Integrate using monotonic dt, trapezoidal velocities and midpoint heading;
  wrap yaw (-pi, pi]. Publish at 50 Hz, observed ~46.6-47.5 Hz.
- All four motor replies must be healthy/fresh <=100 ms.
- Missing inputs or >50 ms integration gaps break interval; no extrapolation
  across missing data. Stamped output pauses when inputs/time sync invalid.
- Wheel drive replies provide feedback during motion. Braked wheels are polled
  using 50 Hz brake frames returning feedback without releasing brake.
- `/odom` frame odom, child base_link; twist in child frame. Same stamp/pose in TF.
- Nominal pose covariance x/y .05, yaw .1; twist vx .01, yaw rate .05;
  unestimated terms 1e6. No effective-track correction applied to encoder odom.
- Exact ROS Jazzy RIHS01 hashes sourced from installed message JSON in Docker.
  Added geometry covariance/transform, nav Odometry, tf2 TFMessage types to
  `lib/easypicoros/types`. Firmware-encoded CDR decoded successfully by rclpy.

Tests after physical polarity correction, on floor:
1. Forward +0.05 m/s for 1 s, then zeros: encoder forward 0.05173 m,
   lateral 0.000186 m, yaw +0.445 degrees, final twist zero. User saw about 5 cm.
2. Reverse -0.05 m/s for 1 s: encoder -0.05028 m, yaw +0.115 degrees,
   final twist zero.
3. CCW +0.2 rad/s for 1 s: encoder +11.03 degrees, final twist zero.
4. First half-turn using encoder angle as stopping target: encoder 179.85
   degrees, user physically observed only ~120 degrees. In-place slip matters.
5. With calibrated gyro and PI enabled, half-turn using integrated gyro as
   stopping target: gyro 181.07 degrees, encoder 355.53 degrees, final twist
   zero. User confirmed physically perfect half-turn, but EXTREMELY JERKY.
   The encoder/gyro ratio varies; do not treat 1.5 as a universal measured truth.

Straight distance checks were only short (~5 cm), so radius is not conclusively
calibrated. A longer measured straight run is still useful. Sticky tyres and
skid steering make effective track larger than physical separation; floor,
payload, speed and direction can affect the ratio.

## Glide and startup calibration — implemented core, not entire spec

Portable `glide.c/h` and `gyro_yaw.c/h` now integrate with `wheels.c` and `ros.c`.
Implemented core:
- One existing body command jerk-limited ramp, no double ramp.
- Effective track feedforward initial `GLIDE_ICR_COEFF 1.5`.
- Bias-corrected, low-pass-filtered gyro yaw (20 Hz filter).
- Bounded yaw PI (KP .5, KI .3, correction cap .5 rad/s, deadband .005 rad/s).
- Uniform wheel saturation and integrator anti-windup on saturation, correction
  clamp and sustained excessive motor tracking error (>200 ms).
- COAST: correction stops after zero command settles; does not hold heading when
  pushed. Watchdog/explicit brake resets glide state, preserves gyro bias.
- Gyro stale >50 ms or motor failure actively brakes all wheels.
- Gyro control runs at 50 Hz motor tick. Requested IMU reads 208 Hz, cooperative
  loop often substantially slower due to blocking motor transactions.
- No IMU FIFO, hardware timestamp batching, second core or full step-C calibration
  routine. Do not describe this as the complete generic glide specification.

A pre-integration +0.15 rad/s, 1-second CCW pulse verified gyro sign +1:
stationary Z mean -0.01965 rad/s; turn mean +0.07307; bias-subtracted +0.09273.
Stationary calibration cannot determine sign. Sensor/body axes currently assumed
aligned per URDF; sensor Z sign was physically checked with that pulse.

Startup bias calibration:
- Continuous 15-second stationary window, begins when ROS sampling is active.
- All four wheel replies fresh and speeds near zero, gyro axes <.1 rad/s,
  acceleration magnitude within 1 m/s^2 of gravity.
- Movement, invalid reads or >50 ms sample gaps restart the window.
- >=750 samples, yaw standard deviation <=.003 rad/s, |bias| <=.05 rad/s.
- Rejected windows retry with motion still blocked; commands ignored, not queued.
- Bias is RAM only, measured each boot. Does not estimate absolute heading.
- Yellow LEDs until ready/fresh, green afterward. ROS/USB remain serviced.
- `/imu` and temperature wait for calibration; magnetometer independent.
- `/imu` Z rate is exactly the corrected/filtered rate used for control;
  X/Y gyro rates remain raw. Odometry stays encoder-only.
- Actual successful boot: 1454 samples, 15 seconds, zero rejected windows,
  bias -0.019718 rad/s (~-1.13 degrees/s raw drift).
- Status fields: yaw_cal, gyro_fresh, bias, yaw_rate, cal_samples, cal_s,
  cal_rejected, yaw_corr, saturated, and now yaw_feedback.

## Current jerk investigation and latest unflashed change

User reports jerk everywhere, including direct `/cmd_vel`, not just stopping.
They asked whether the IMU is too sensitive. Do not conclude sensor sensitivity
is the proven root cause: calibration and gyro half-turn worked; measurement
noise/vibration plus abrupt controller correction is a plausible mechanism.

Previously PI correction was added directly after the smooth command ramp.
That could inject abrupt motor-command changes, defeating tested shaping.

Latest isolation build:
- `GLIDE_YAW_FEEDBACK_ENABLED false` (PI OFF).
- Calibration, gyro publication, feedforward, geometry and operator ramps kept.
  No simultaneous gain/filter/deadband/ramp change, so comparison is meaningful.
- Added `correction_profile`, a separate jerk-limited correction path used when
  PI is re-enabled: acceleration .25 rad/s^2, jerk 1.0 rad/s^3.
- COAST tapers correction to zero rather than dropping it abruptly.
- `/imu/status` includes `yaw_feedback=0` to distinguish the comparison image.
- Current correction PI gains/deadband/filter remain the previous values, but
  are inactive. Correction limits are initial tuning values, not ground-proven.

Build and tests passed, no flash confirmed. The robot may still be on the old
PI-enabled image. Establish image state before interpreting further tests.

## Next steps in order

1. User flashes latest explicit ROS UF2; wait for stationary calibration ready.
2. Read `/imu/status`, confirm `yaw_feedback=0` and sync valid.
3. With user observing and clear floor, repeat gentle forward/reverse and a
   small turn with PI OFF. Then gyro-target half-turn if small motions are smooth.
   Check physical jerk, not just angle. Retain measured command/gyro/odom data.
4. If PI-off smoothness returns, re-enable PI with the new correction profile,
   initially gentler gains if warranted. Test each change separately. Compare
   gyro noise, commanded yaw, yaw_corr, motor feedback and timing. Avoid raising
   filter strength, gains and limits together without a controlled comparison.
5. If PI-off still jerky, compare feedforward factor 1.0 vs 1.5 and inspect
   motor update interval/quantization; yaw ramp limits have less ground validation
   than linear limits. Do not blame IMU when feedback is disabled.
6. Tune physical yaw tracking and smoothness before adding FIFO/multicore. Measure
   actual IMU/control dt and latency; add FIFO or separate control scheduling only
   if evidence shows timing limits are the problem.
7. Calibrate encoder effective separation separately if user wants accurate
   encoder yaw. Gyro yaw closure improves command tracking, not encoder odometry.
   A gyro-stopped half-turn can produce ~360 encoder degrees on this floor.
8. Longer measured straight runs check radius/translation scale; CW and CCW
   repeated turns check asymmetry and surface-dependent effective track.
9. If sync=waiting recurs, instrument board sync callback and rejection reasons
   (echo mismatch, RTT, layout/count, range) and reconnect declarations. Host
   prompt responses alone do not prove board reception or synchronization.
10. Full specs remaining: generic per-wheel glide API, complete calibration
    routine C, FIFO/hardware sensor timestamps, timing-driven core split if needed.

## Validation performed

Committed-source portable checks under `tests/` (files may remain untracked):
- motion_profile_test.c: acceleration/jerk limits, variable dt, reversal/stop,
  non-finite inputs and timing gaps.
- time_sync_test.c: portable clock-sync validation from earlier work.
- odometry_test.c: straight/reverse/arcs, yaw wrapping, reset, invalid/gap recovery.
- gyro_yaw_test.c: full window, bias/filter, freshness, movement/gap restart,
  noisy/excessive-bias rejection.
- glide_test.c: simulated lag/slip yaw tracking both directions, command shaping,
  coast/reset, saturation/anti-windup, invalid input/state preservation; latest
  tests verify correction rate/jerk bounds under oscillating gyro and identical
  wheel commands with PI disabled regardless of gyro noise.

Temporary hardware-mocked wheel integration test:
`/tmp/base101-odom-feedback/test.c` checks actual wheels.c with mock serial/motors,
startup gate, physical command polarity, encoder-only feedback, stale gyro stop,
failed motor stop and recalibration braking. Latest test passed.

Temporary serializer fixture test:
`/tmp/base101-odom-serialization`: firmware picoserdes encoded Odometry/TF;
actual ROS Jazzy rclpy decoded frames/stamps/pose/twist/covariances/TF correctly.
Temporary files can disappear; do not rely on them existing after resume.

Final checks: `cmake --build build -j4`, `git diff --check`, latest glide portable
check and actual wheel integration mock all passed. No test should be mistaken
for physical validation of the newest PI-disabled image.


## End-of-day update — latest authoritative state

The user briefly asked to enable the correction-limited PI and blue calibration
LEDs, then explicitly corrected that request: they had uploaded the previously
unflashed PI-disabled image, found it incredibly smooth, and wanted only a
180-degree test. The interrupted tool was a read, not an edit. No PI enable or
blue-LED changes were made. Calibration LEDs are still yellow.

The final test verified current status before commanding motion:
- `yaw_cal=ready gyro_fresh=1 sync=valid yaw_feedback=0`.
- Bias -0.019121 rad/s; 1459 calibration samples, 15 seconds, no rejected windows.
- CCW half-turn uses calibrated gyro integration for stopping, not encoder yaw.
- Maximum command 0.3 rad/s, taper as target approaches, 20 Hz commands through
  `/cmd_vel_agent`, stale-feedback and competing-command aborts, zeros afterward.
- Final gyro turn **179.8758 degrees**.
- Final encoder turn **306.7444 degrees**.
- Final encoder twist (0, 0); gyro -0.0001223 rad/s; elapsed including stop 15.79 s.
- Test publisher destroyed after zeros. No motion publisher remains from our test.
- User had reported incredibly smooth operation before this test. Physical angle
  and smoothness of this particular final half-turn have not separately been
  confirmed by the user yet; do not invent that confirmation.

Resume from the successful PI-OFF baseline, not the previous jerky PI-ON image.
The new correction ramp exists in source but remains physically untested with
PI enabled. If re-enabling PI later, make it an explicit isolated comparison.
The earlier suggestion to flash the isolation image is already completed.


## Final user observations

The user confirmed the final PI-disabled gyro-target half-turn was physically
perfect, then clarified it became **slightly jerky toward the end**. Treat this
as a smooth baseline with a remaining terminal-taper issue, not entirely
jerk-free. The earlier PI-enabled continuous/extreme jerk is a different issue.

The user reports the IMU is **40 mm behind the robot centre**. This supersedes
any assumption that the sensor is dead centre. It does not independently verify
sensor axis orientation, height, or the relationship between robot centre and
URDF base_link. Do not silently modify the sibling URDF using incomplete data.
The URDF previously read has an IMU lever arm that should be checked against this
physical measurement when the user resumes.

Gyro angular velocity is shared across a rigid body and does not require a
centre-of-robot lever-arm correction. Accelerometer readings do differ with
position: a_sensor = a_centre + alpha cross r + omega cross (omega cross r).
A rearward 40 mm offset therefore adds tangential/centripetal acceleration during
turns. Account for the measured mounting transform if fusing accelerometer data;
current encoder-only odometry and gyro yaw integration do not use acceleration
for motion estimation. Calibration only uses gravity magnitude for stationarity.

Next smoothness experiment: inspect the host half-turn taper
`w_command = min(0.3, 1.5 * remaining_gyro_angle)` and its transition to zero near
0.01 rad remaining. PI was disabled in the final test, so the mild end jerk cannot
be attributed to yaw PI injection. Test a smooth planned deceleration/less noisy
terminal command and gyro deadband before changing working motor ramps or adding
lever-arm corrections to gyro. This is a hypothesis, not a proven cause.

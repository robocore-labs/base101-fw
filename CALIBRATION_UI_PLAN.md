# Robot calibration UI working plan

Agreed on 2026-09-18. This document describes planned work; the connection foundation and first manual browser workflow are implemented as described below. The approved automatic calibration UI implementation is described in the latest progress entry below.

## Goal

Build a guided calibration and tuning interface for the robot, with manual driving available throughout. Establish repeatable measurements, compare trials, and export accepted settings for production firmware.

Use `/home/cdr/working/mod101/configurator` as the visual and architectural reference: its dark surfaces, teal accents, typography, panels, and readouts, with a Python3 server running on the Orin and a browser frontend.

## Starting point

- The current flashed motor baseline is smooth with yaw PI feedback disabled. Keep that baseline for initial calibration.
- Motion shaping, gyro calibration, motor drivers, and encoder odometry already exist and should be reused.
- A small jerk remained near the end of the last 180-degree turn. Record the command taper and motor response to investigate it without changing several controls at once.
- The IMU is approximately 40 mm behind the chassis center. This does not change its measured yaw rate; acceleration-based work would need to account for the offset.
- Odometry remains encoder-only. Gyro yaw is an independent measurement for calibration and validation.

## Architecture

Browser UI → Python server on Orin → USB serial → separate calibration firmware on MCU.

The calibration firmware is a separate build target, evolved from the motor terminal, rather than a production ROS firmware mode. Share existing drivers and control modules instead of copying their implementations.

The Python server owns the serial port, serves the frontend, records telemetry, and stores profiles and results. The calibration connection must have exclusive ownership of the serial device; the production ROS serial endpoint must release it while this firmware is in use.

The MCU executes bounded tests, motion shaping, and stop logic. Browser and server scheduling must not determine motor timing. Commands should support connection/status discovery, validated parameter updates, manual drive, test start/cancel, calibration, and telemetry. Define the exact protocol during implementation; line-delimited JSON is a reasonable initial option.

Telemetry needs MCU monotonic timestamps and sufficient detail to compare requested body velocity, shaped velocity, measured wheel speeds, raw/filtered gyro, feedback correction, odometry, faults, and test events. Preserve higher-rate samples or suitable summaries for measurement even if browser plots update more slowly. Include current or other hardware telemetry only where supported.

## Guided workflow

Before movement, complete the existing approximately 15-second stationary gyro bias calibration. Make calibration progress and readiness clearly visible. Verify geometry, wheel mapping, and sign conventions before accepting measurements.

### 1. Wheels in the air

- Check each wheel's commanded direction and encoder sign.
- Compare requested, shaped, and measured wheel speeds.
- Establish minimum useful speed, response delay, and smooth acceleration, stopping, and reversal.
- Record a baseline before changing parameters.

### 2. Spins on the floor

Estimate:

`k_icr = encoder-predicted yaw / bias-corrected gyro-measured yaw`

Predict yaw from actual wheel feedback and physical wheel separation, not commanded velocity or an already corrected yaw estimate. Otherwise motor tracking error or an existing correction contaminates the measurement. Keep yaw PI disabled during these trials.

Start with 12 trials: three speeds × two directions × two repetitions. Each trial is explicitly started by the operator. Integrate encoder and gyro yaw over matching intervals; use the steady portion to estimate slip and retain the complete maneuver for acceleration/stopping analysis.

Show individual ratios, median, spread, and direction/speed dependence. Record surface and payload. Significant differences between directions or speeds may mean that a single constant is inadequate; do not hide these differences in an average.

### 3. Braking

- Tune acceleration, jerk, and deceleration against the quiet baseline.
- Measure stopping time, encoder stopping distance, residual rotation, and command/feedback behavior.
- Evaluate normal shaped stopping first; treat electric braking or any additional stopping mode as separate explicit trials.
- Accept operator measurements of actual chassis stopping distance. Encoder distance alone cannot measure translation accurately during wheel slip.

### 4. Odometry validation

- Run bounded straight-distance and turning trials.
- Compare encoder odometry with operator-measured displacement and independent gyro rotation.
- Keep physical geometry, motion feedforward correction, and any odometry calibration parameters explicit and separate. Do not silently apply one fitted coefficient to every path.
- Validate on the intended floor and payload, and retain results for comparison on another surface.

## Manual controls and tuning

Keep a persistent control panel available throughout the workflow:

- Hold-to-drive controls, speed limits, and a prominent stop button.
- Live adjustment of acceleration, jerk, and `k_icr`; yaw PI tuning comes later and starts disabled.
- Calibration status, requested/shaped/measured velocities, gyro data, and motor faults.
- Recording, replay, and comparison of trials.

Manual takeover cancels an automated trial. Parameter changes during a measurement invalidate that trial. Prefer applying validated parameter batches while stationary; establish which parameters can safely change during manual driving.

The MCU enforces a heartbeat timeout, hard motion limits, bounded test duration, and stop conditions for stale sensor data or motor faults. Connection loss must stop motion independently of the browser. A fault must not trigger an automatic return maneuver.

## UI layout

- Top: connection state and guided step progression.
- Main area: current instructions, live plots, and test visualization.
- Persistent side panel: manual controls and tuning values.
- Lower area: trial results, comparisons, and saved recordings.

Reuse the configurator's theme and lightweight browser approach. A 3D viewer is optional, not required for the first useful version.

## Persistence and export

Calibration firmware settings initially live in RAM. The Python server saves named profiles and recordings, with JSON metadata/results and an appropriate tabular export for telemetry.

Save parameter snapshots, geometry, firmware identification, timestamps, surface/payload notes, and trial validity with each recording. Export accepted values as a reviewable firmware configuration block or header. MCU flash persistence is outside the initial scope.

## Implementation order

1. Establish the separate firmware target and server connection: handshake, status, validated parameters, heartbeat, and stop.
2. Deliver a complete manual slice: connect → stationary gyro calibration → drive → tune ramps → record → stop.
3. Add guided wheels-in-the-air checks.
4. Add bounded spin trials, yaw integration, `k_icr` summaries, and trial comparison.
5. Add braking tests and operator-entered physical measurements.
6. Add odometry validation and export of accepted settings.
7. Investigate terminal-turn jerk and, if needed, enable and tune yaw PI as an isolated experiment.

## Verification priorities

Verify timeout and abort behavior, trial bounds, wheel/sign mapping, timestamp alignment, calculation correctness, and profile round trips. Physically validate each new moving test under operator observation before using it as a calibration reference.

Initial success means a reliable manual tuning and recording loop. Calibration success means repeatable trials with visible uncertainty and an explicit, reviewable path from measured results to production settings.

## Connection foundation — initial stage, 2026-09-18

The first implementation step initially established: `calibration_firmware`
provides protocol-v1 handshake, status, atomic RAM parameter validation, a
500 ms MCU heartbeat lease, and repeated electric braking. The standard-library
Python server in `tools/calibration/server.py` owns serial I/O, sends heartbeat,
and exposes status, schema, parameter-update, and stop HTTP endpoints. That initial
connection-only stage could not drive motors or change production settings. The
manual protocol-v2 slice below supersedes it.

All firmware targets build. Host protocol checks and pseudo-terminal/HTTP
integration tests pass. Physical handshake and motor brake acknowledgements
still need verification after the operator flashes `build/calibration_firmware.uf2`.
Run instructions and protocol details are in `tools/calibration/README.md`.

Next: the complete manual slice, including IMU calibration/telemetry, driving
with motion shaping, a browser/operator deadman distinct from server heartbeat,
and measured-at-rest checks before parameter updates.

### Manual UI slice — implemented 2026-09-18

Protocol-v2 firmware now shares the production wheel, shaping, ICR feedforward,
and gyro estimator modules, with yaw PI forced off. It provides automatic and
explicit 15-second gyro calibration, bounded manual drive, smooth release,
independent operator/server watchdogs, fault latching, measured-rest runtime
tuning, offline motor retry while stopped, and encoder-only telemetry.

The Python server serves a responsive browser UI using the configurator theme:
calibration progress, hold-to-drive buttons and W/A/S/D or arrows, always
available Stop/Escape, live body/yaw plots and wheel feedback, atomic tuning,
recording, and JSON downloads of recordings/settings. Browser commands have an
owner, sequence ordering, and a short server intent lease; the MCU operator
watchdog is not renewed by heartbeat. Recorded replies are approximately 20 Hz
while driving, not full-rate IMU acquisition. Storage is in server memory until
download; named persistent profiles are still planned.

Builds, portable protocol/server tests, shared-drive tests, telemetry JSON
serialization, and headless desktop/mobile browser checks pass. Browser checks
use a pseudo-terminal and synthetic hardware telemetry. The new image has not
been flashed or physically driven by the implementation session.

Next: physically validate this manual slice with wheels lifted, then implement
guided wheel checks and bounded spin trials. Keep the baseline PI disabled
while identifying k_icr. Braking and odometry validation follow.

### Initial physical connection debugging — 2026-09-18

The actual protocol-v2 image replied with calibrated gyro, all four motors
fresh/braked (mask 15), and no IMU read failures. However, MCU `%g` formatting
of negative-zero RPM from mirrored wheels produced leading-zero numbers that
are invalid JSON, causing the server to discard every reply and time out.

Fixed serialization to use fixed-point floats and normalized measured zero.
Added an actual Pico SDK formatter regression, alongside shared-drive tests,
and explicit malformed-JSON server diagnostics. The corrected image builds
and produces valid JSON with the SDK formatter on the host. The operator
flashed the corrected image and confirmed that the physical UI works.

### Manual baseline confirmed — 2026-09-18

The operator confirmed the manual UI works on the robot after the serialization
fix. Parameter help now explains both manual speed targets and all five tuning
values, including the response/smoothness tradeoffs, RAM-only lifetime, and
floor-only ICR measurement. Emergency Stop bypasses shaped ramps.

Next implementation: guided wheels-up checks for direction, feedback signs,
minimum useful speed, and acceleration/release/reversal behavior. Preserve manual
control and recording throughout. Then add the floor spin sequence: at least
12 runs (three speeds, both directions, two repeats), with yaw PI disabled.
Compute k_icr from actual encoder-predicted yaw divided by bias-corrected gyro
yaw over matching measurement windows; report repeatability and direction/speed
dependence before applying a suggested value. Braking and odometry tests follow.


### Guided wheels-up checks — implemented 2026-09-18

The UI now offers eight bounded hold-to-run checks using the existing protocol-v2
firmware and operator leases: four polarity checks, a three-speed low-speed sweep,
acceleration/release near 100 RPM, and linear/yaw reversals. Normal completion
releases through shaping, waits for measured rest, then disarms. Early release,
focus loss, Stop, readiness/telemetry loss or parameter changes abort and brake.
Manual operation and tuning remain available between checks. Each run retains
parameters, timestamped browser telemetry, plateau wheel RPM, time to rest, and
operator observations/notes for JSON export. Reports are in page memory only.

Server tests and simulated desktop/mobile browser checks cover early abort,
timed release/completion, disarming, focus-loss interruption and report export.
Physical guided checks remain for the operator. No robot movement was issued
by the implementation session. Restart the Python server and refresh the UI;
no firmware reflash is needed. Next implementation remains the 12-run floor
spin sequence for measured k_icr, followed by braking and odometry validation.


### Approved calibration lab — implemented 2026-09-18

The approved prototype is now the live index page on the Python server (port
8080), with the dark palette, larger/lighter typography, central accordion
steps and two full-height right sidebars: telemetry/wheels, then control and
parameters. Branding, introductory copy and Evidence panel are removed.
Observation buttons use colored borders. The port-8081 concept stays a preview.

Firmware program version 2.1 retains the protocol-v2 handshake and adds full
atomic RAM configuration (19 motion/geometry/motor/filter/PI settings), MCU
measurement summaries and an independent, non-renewable maximum-30-second trial
deadline. PI remains off by default and all guided identification rejects PI.
Changing gyro filtering restarts bias calibration; changing physical geometry
resets calibration encoder pose. Production defaults remain in robot.h.

The server owns eight wheels-up recipes, twelve floor-spin recipes, four normal
release/braking recipes and four straight physical-reference recipes. Each
requires confirmed setup and continuous operator presence; completion disarms.
Manual takeover, abort, faults and timeout do not queue a return maneuver.
Completed and interrupted results, notes and references persist atomically in
calibration-runs/ through a separate writer. Refresh retains results. RAM settings
are not automatically reapplied and interrupted trials never resume.

MCU metrics integrate actual encoder and corrected gyro yaw over matching valid
sample intervals, retain steady-window integrals, tracking RMS, first-target
response, stop distance/time and IMU transient summaries before browser/USB
sampling. Knock scoring is heuristic: residual acceleration after conservative
commanded-linear and 40-mm rotational bounds, threshold 0.8 m/s² and 80-ms event
separation. It can miss suppressed/noisy events and does not certify mechanical
knocks. Actual sensor loop rate remains cooperative, not guaranteed 208 Hz.

Two matching reviewed baselines permit a one-axis −20% jerk candidate, with
response-time guards and candidate rollback advice after repeated comparison.
The floor fit requires all twelve confirmed valid recipes under identical
parameters/surface/payload with ≤15% ratio range. Per-speed/direction statistics
remain exported. Radius fitting requires two forward and two backward physical
references with ≤10% scale range. Candidates are experimental, explicitly loaded
and applied at rest, never silently accepted or flashed to production.

Portable MCU, metrics, server/PTy, analysis, actual-Pico-formatter, shared-drive
and live desktop/mobile UI checks validate the implementation. Physical checks
remain for the operator after flashing build/calibration_firmware.uf2 and
restarting tools/calibration/server.py. No physical motion was issued during
implementation. See tools/calibration/README.md for exact use and limitations.


### 2026-09-18: automatic steps and generated ROS header

Subtests now advance automatically within a step, with measured rest and brake
confirmation between them. Operator presence continues across pauses; Stop or
an interruption cancels the remaining queue. A physical review is made once at
the end. Single-subtest runs remain available. Ten full-turn verification checks
can also run as one sequence.

The chosen deployment workflow is calibration firmware -> UI saves calibration.h
-> build base101_firmware -> flash the ROS image. The ROS target includes all
19 accepted configuration fields from that header at startup. No custom QSPI
storage, reserved flash sectors, or runtime flash-saving commands remain.

# Base101 calibration lab

Separate MCU firmware, a Python3 server on the Orin, and the approved dark browser
UI. Calibration steps occupy the main area; telemetry/wheels and control/tuning
occupy two independent full-height right sidebars. There is no frontend build or
runtime dependency beyond Python3's standard library.

## Build, flash, run

```sh
cmake -S . -B build
cmake --build build --target calibration_firmware -j4
```

Flash **`build/calibration_firmware.uf2`** explicitly. This updated image is
required for measurement summaries, advanced settings, and automatic trials.
It retains protocol-v2 handshake compatibility, advertises `full_config`,
`metrics`, and `bounded_trial`, and has program version 2.1. Old images can still
connect; the new controls stay disabled rather than pretending to apply settings.

Release the CDC device from ROS/zenoh and other serial readers. Restart the server:

```sh
python3 tools/calibration/server.py --port /dev/ttyACM0 --bind 0.0.0.0
```

Open **`http://<orin-ip>:8080/`**. The separate design preview on port 8081 is not
the live application. Omit `--bind` to listen on localhost only. The API uses the
robot's trusted network and does not provide authentication.

## Workflow

Keep the robot stationary through its continuous 15-second gyro bias calibration.
Missing feedback, motion and sensor gaps restart that window. Motor retry, fault
latching and magnetometer exclusion retain the existing calibration behavior.

The four accordion steps are:

1. **Wheels up:** physical forward/backward/turn polarity, low-speed sweep,
   acceleration/release near 100 RPM, and linear/turning reversals. The operator
   confirms physical direction; encoder signs cannot establish it independently.
2. **Floor spins:** twelve trials, three speeds (0.2/0.4/0.6 rad/s), both
   directions, two repeats. Each drives for `pi / requested_rate + 2` seconds,
   then releases normally. These are approximately half-turns, not precision
   180-degree maneuvers. Skid compensation is actual encoder yaw / calibrated
   gyro yaw over matching steady intervals, never requested yaw or a ratio
   multiplied by the coefficient already in use.
3. **Braking:** repeated normal releases from ±0.1 and ±0.2 m/s. MCU summaries
   report stopping time and encoder stopping distance. Wheel slip means encoder
   stopping distance is not necessarily chassis travel. Emergency braking is
   the prominent Stop action, not silently substituted for normal release.
4. **Odometry:** two forward and two backward straight trials. Enter independently
   measured travel for each result. A wheel-radius candidate requires all four
   matching references and ≤10% scale-ratio range. Gyro alone cannot establish
   accurate wheel radius. Physical separation is explicit and encoder odometry
   remains independent of IMU yaw.

For each trial: confirm setup, record surface/payload on the floor, enable
controls at rest, then **click Run entire step**. All subtests run automatically,
with confirmed braking and a short pause between them. The step ends disarmed. Clicking the step button again,
Stop/Escape, focus/page visibility loss, missing operator presence, changed
parameters, stale sensors, restart, faults or serial failure abort the trial.
Manual takeover cancels and stops a trial; enable controls again before driving.
No automatic return maneuver or unattended series follows completion.

Red means physical failure; yellow means tuning is needed; green confirms the
physical check. Sensor measurements are scored independently. Notes and physical
reference distances can be saved after the result. Sliders load either current
MCU values or a candidate; **Apply** is explicit and requires measured rest and
braked motion. Exports contain accepted MCU settings, not unapplied edits.

## Automatic analysis

The MCU integrates encoder and bias-corrected filtered gyro yaw on the **same
valid sample intervals**. Steady rotation requires at least two seconds since
movement start, shaped yaw within 5% of requested yaw, zero linear request, and
fresh sensors. Gaps >50 ms or invalid samples are counted and invalidate a fit.
A floor fit also requires ≥2 seconds steady data, ≥0.3 rad gyro rotation,
consistent signs, and supported ratio 1–5.

IMU summaries are calculated at the actual cooperative-loop sensor sampling
rate, before USB/browser downsampling. The sensor is configured at 208 Hz;
actual acquisition rate depends on I2C and motor transactions. Replies report
sample/gap counts, transient peak/RMS, raw acceleration-change peak, transient
count, tracking RMS, response, stop time/distance and integrated yaw/distance.

Transient detection uses acceleration relative to its stationary baseline,
subtracting conservative bounds for commanded linear acceleration and
rotational acceleration at the known 40 mm lever arm. This avoids assuming the
accelerometer's X/Y mounting orientation. It is **a heuristic**, not a guarantee
that every count is a mechanical knock. The residual threshold is 0.8 m/s² with
80 ms event separation. Sensor noise, chassis tilt and floor bumps can influence
it, and the conservative rotational bound can suppress an event. Do not infer
translation by double-integrating this accelerometer signal.

Tracking compares encoder response to software-shaped motion, including skid
feedforward for yaw. Time to 90% uses encoder speed for linear tests, encoder yaw
with feedforward removed for supported wheels-up turn tests, and gyro yaw on the
floor. Unreached targets are explicit, not zero response time.

Two comparable baseline runs are required before a shaping candidate is proposed.
If transients repeat and response is ≤1.5 seconds, the server proposes 20% less
jerk on the relevant axis, keeping acceleration unchanged. Repeat the candidate
twice; if response worsens by >25% or transients increase, the server recommends
restoring the previous value. This is a bounded experimental search, not a claim
of a globally optimal fit. User observation can be green or yellow for comparison;
physical failures and invalid data are excluded.

Floor summary reports all twelve distinct recipes, median, full range/median,
and per-speed/per-direction medians. Different settings, geometry, surfaces or
payloads are never mixed. A coefficient is proposed only with all twelve
confirmed valid runs and ≤15% total range; wider dependence is exposed rather
than hidden in an average.

## Runtime parameters

All motion tuning groups are exposed and independently validated by server/MCU:

| Parameter | Range | Meaning |
|---|---|---|
| ax / jx | 0.01–5 / 0.01–50 | Linear acceleration / jerk, m/s² / m/s³ |
| aw / jw | 0.01–20 / 0.01–200 | Yaw acceleration / jerk, rad/s² / rad/s³ |
| k_icr | 1–5 | Yaw skid feedforward |
| radius / separation | 0.02–0.1 / 0.15–0.5 m | Physical wheel geometry |
| ramp | integer 1–255 | DDSM motor acceleration byte |
| rpm_max | integer 20–200 RPM | Uniform wheel allocation cap |
| vx_max / wz_max | 0.05–0.40 m/s / 0.1–0.8 rad/s | Calibration body limits |
| lpf | 1–80 Hz | Gyro filtering; changing it restarts bias calibration |
| kp / ki | 0–2 | Yaw PI gains |
| correction_max | 0–1 rad/s | Feedback correction cap |
| deadband | 0–0.05 rad/s | Yaw error deadband |
| correction_accel / correction_jerk | 0.01–2 / 0.01–20 | Correction shaping |
| yaw_feedback | integer 0 or 1 | PI enable; off by default |

Guided identification requires yaw PI off. Manual PI experiments remain explicit.
Geometry updates reset calibration-image encoder pose and rescale measured wheel
velocity consistently; production firmware defaults remain in `robot.h`.
Parameters live in MCU RAM. Trial history is persisted on the server, but settings
are not automatically reapplied after MCU restart. Watchdogs and scoring thresholds
remain fixed. Normal release uses shaping; emergency Stop bypasses it.

## Independent stop layers

Browser manual drive / trial presence expires at 200 ms on the server. Server
heartbeat cannot extend it. MCU drive packets expire at 350 ms; server heartbeat
cannot extend that either. MCU server session expires at 500 ms. USB unmount,
motor/gyro failure and invalid profile timing brake independently.

A server-owned trial additionally starts an MCU `bound` deadline, maximum 30
seconds, that repeated drive/heartbeat packets cannot extend. Normal release has
a two-second electric-brake backstop; the server allows three seconds to receive
measured rest. Slow settings can hit the backstop before a natural settle.
Timeout checks run between bounded bus transactions, not in a dedicated interrupt.

Requests use browser client ownership and monotonically increasing sequence IDs.
Queued stop/calibration requests take priority. Invalid updates are atomic. The
server never silently reconnects after failure: restart it after resolving the
cause. Physical movement validation remains an operator task after flashing.

## Storage, exports and API

Trial summaries, parameter snapshots, observations, notes and downsampled serial
telemetry are written atomically by a separate writer thread under
`calibration-runs/` (override `--data-dir`). Up to 500 recent trials are loaded for
analysis; older files remain on disk. Storage errors are visible in the UI.
Browser refresh retains completed results. MCU restart/connection loss never
resumes a running trial. Results downloaded by the UI contain summaries/plateaus;
the on-disk JSON additionally retains downsampled per-reply telemetry, including
IMU and cumulative measurement summaries, not every raw IMU sample.

| Endpoint | Purpose |
|---|---|
| GET `/api/status` | Connection, hardware, operator, trial and storage state |
| GET `/api/schema` | Bounds and bounded trial recipes |
| GET `/api/trials` | Results, floor fit and per-recipe candidates |
| POST `/api/arm`, `/api/drive`, `/api/release` | Owned manual control |
| POST `/api/trial` | Start confirmed recipe while armed at rest |
| POST `/api/presence` | Refresh operator presence during a held trial |
| POST `/api/stop`, `/api/calibrate` | Brake/disarm or restart bias calibration |
| POST `/api/parameters` | Complete config or legacy five-parameter update |
| POST `/api/review` | Result ID, observation, notes and optional reference_m |
| POST `/api/record`, GET `/api/recording` | Legacy manual recording API |

Same-origin browser requests are enforced. Operator messages include `client`
and increasing integer `seq`. A trial also provides `recipe`,
`setup_confirmed: true`, and floor `surface` / `payload` notes.

Serial protocol retains `hello 2`, `status`, `heartbeat`, `drive vx wz`, `release`,
`stop`, `calibrate` and legacy `set ax jx aw jw k_icr`. New operations:
`measure 0` (wheels-up response), `measure 1` (chassis response), `bound ms`, and
`config` followed by the 19 parameters in the table/schema order. Numeric replies
use fixed-point formatting; actual Pico formatter serialization is regression
checked. Invalid JSON reports byte context. Input lines allow 511 characters;
calibration replies/FIFOs allow 4096 bytes. Production CDC buffers remain 2048.

## Verification

```sh
cc -std=c11 -Wall -Wextra -Werror -I. tests/calibration_protocol_test.c \
  calibration_protocol.c -lm -o /tmp/base101-calibration-protocol-test
/tmp/base101-calibration-protocol-test
cc -std=c11 -Wall -Wextra -Werror -I. tests/calibration_metrics_test.c \
  calibration_metrics.c -lm -o /tmp/base101-calibration-metrics-test
/tmp/base101-calibration-metrics-test
python3 tests/calibration_server_test.py
python3 tests/calibration_trial_server_test.py
python3 tests/calibration_trials_test.py
python3 tests/calibration_serialization_test.py --pico-sdk /path/to/pico-sdk
python3 tests/calibration_ui_test.py
```

The optional UI test needs Playwright Chromium only in the test environment.
All motion integration tests use PTYs or deterministic time/bus stubs; none
opens physical serial. Shared-drive/actual SDK formatter tests cover geometry,
RAM configuration, hardware ramp, polarity, gyro recalibration, faults and JSON.


### Ten-turn yaw verification and step summaries

Odometry includes ten additional 360° verification runs at 0.4 rad/s. Each is
started explicitly, targets integrated gyro yaw with a jerk-limited stop-distance
estimate, and remains under its own MCU deadline. A turn must finish within 10%
of 360° to be accepted; this is not a precision-position controller. Yaw PI may
remain enabled for verification, while identification still requires PI off.
Mark correct turns green. The report accumulates the latest accepted result for
each of the ten turns with identical settings, surface and payload, reporting raw
encoder yaw minus gyro yaw, percentage error and diagnostic encoder/k_icr error.
It does not modify encoder-only odometry or claim gyro is an external ground truth.

Suggestions from each completed calibration step remain visible after settings
change, labeled as previous settings. Loading a candidate changes only suggested
sliders; it does not restore the whole old configuration or apply automatically.
Repeat measurements after applying a candidate to validate the change.


### Automatic substeps and exporting the ROS configuration

Run entire step starts all eight wheels-up checks, twelve floor spins, or four
braking checks from one click. Odometry has a four-run straight-reference sequence
and a separate ten-turn sequence: select a 360° verification subtest, then click
Run all 10 turns. Confirm setup once and keep the page focused while the server
runs the sequence. Stop, focus loss, stale telemetry, faults or lost presence
cancel the remaining tests. There is no restart after interruption.

At the end, one direction/physical review applies to the completed sequence.
Sensor validity remains independent of that review. Run selected subtest only
is available for comparisons and independent physical distance references.

After tuning, Apply settings, stop/disarm, then click **Save calibration.h**.
The server atomically writes the accepted MCU parameters to the repository's
calibration.h (use --header to select another output). Only the ROS target
compiles this header; the calibration target remains independent. Rebuild and
flash the ROS image:

```bash
cmake --build build --target base101_firmware -j4
```

Flash build/base101_firmware.uf2 using picoflash as usual. Settings are compiled
into that image and survive reboot. Saving the header does not reflash the MCU.

Browser trial presence and cached server telemetry each have a 1-second timeout.
The MCU heartbeat/operator watchdogs and sensor freshness limits are unchanged.
Manual drive intent retains its 200 ms server deadline. Timeout messages identify
browser keepalive versus MCU telemetry loss separately.

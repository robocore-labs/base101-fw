# Calibration handoff — 2026-09-18

## Current workflow

1. Flash the calibration firmware and run the Python calibration server.
2. Use the UI to calibrate and apply parameters in RAM.
3. Stop and disarm, then click **Save calibration.h**.
4. Build the ROS firmware, which compiles all 19 parameters from calibration.h:

```bash
cmake --build build --target base101_firmware -j4
```

5. Flash build/base101_firmware.uf2 as usual. Settings are compiled into the
   ROS image and survive reboot.

There is no custom QSPI persistence or reserved flash storage. The calibration
firmware remains independent of the generated ROS configuration header.

## Automatic calibration steps

One click runs each step's subtests automatically: eight wheels-up checks,
twelve floor spins, or four braking checks. The robot brakes and confirms rest
between subtests; the whole step disarms when finished. Physical direction is
reviewed once at the end. Individual subtest runs remain available.

Odometry also has four straight-reference runs and a separate automatic sequence
of ten 360-degree verification turns. Select a verification subtest and click
**Run all 10 turns**. PI may stay enabled for verification; identification requires
PI off. The report compares cumulative raw encoder yaw with gyro yaw, and shows
a separate diagnostic skid-compensated comparison. Physical distance references
still require independent measurement.

Suggestions remain visible after each step. Recommendations measured under
previous settings are labeled accordingly. Loading a candidate changes sliders;
applying it is explicit. Repeat measurements to validate changes.

## Latest fix: browser watchdogs

The old step interruption message combined browser presence expiry and stale
telemetry. The original 200 ms browser presence and 250 ms cached telemetry
windows were too tight for the web UI.

Both trial browser presence and cached host telemetry now have **1-second**
timeouts. The UI uses the same telemetry freshness window. Reasons are separate:

- Browser keepalive expired (1 s).
- MCU telemetry expired (1 s).
- Operator disarmed, MCU readiness/fault changed, or parameters changed.

The MCU's independent watchdogs and sensor freshness limits are unchanged.
Manual drive intent retains its 200 ms server deadline. Stop/Escape, page focus
or visibility loss, stale sensors, faults, and lost presence cancel the sequence;
it never resumes automatically after interruption.

To activate this latest timing change, restart the Python server and refresh
the browser. **No firmware flash is needed for the watchdog timing change.**

```bash
python3 tools/calibration/server.py --port /dev/ttyACM0 --bind 0.0.0.0
```

Open http://<orin-ip>:8080/. Port 8081, if still running, is only the design preview.
The server's --header option can select another generated header output path.

## Verification and next physical check

Firmware builds and live browser integration checks passed for the generated
header and automatic-step workflow. After the timeout adjustment, all ten
watchdog/sequence integration tests passed, including browser jitter tolerance,
expiry between subtests, cancellation, and distinct telemetry timeout reasons.
The tests use simulated hardware and PTYs; they do not move the robot.

Next: restart the server, run an automatic step and the ten-turn verification on
the floor, review results, save calibration.h, then rebuild and flash ROS firmware.

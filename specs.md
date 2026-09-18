# base101-fw motion work

Spec v0.2 · 2026-09-16

Four steps in order. Each is independently testable and independently valuable
— A alone may fix most of the felt problem, and D is useful whether or not C
ever runs.

| Step | What | Code? |
| --- | --- | --- |
| **A** | Basic changes — config only | No |
| **B** | glide library, in-tree | Yes |
| **C** | Calibration sketch | Separate project |
| **D** | Odometry and publishers | Yes |

**Context from the repo**, so the steps below make sense:

- The DDSM210 hardware accel register is already set: `WHEEL_ACCEL_TIME 20`.
- A firmware-side per-wheel ramp already exists: `WHEEL_ACCEL_LIMIT_RAD_S2 6.0`
  at `WHEEL_CONTROL_HZ 50`.
- A stale command triggers the DDSM210 active brake, hard and unramped, after
  `COMMAND_TIMEOUT_MS 500`. 
- The LSM6DSOX runs at 208 Hz ODR. `IMU_HZ 50` is the publish rate.
- The magnetometer is unused for all of this.
- The host publishes `/cmd_vel` and expects `/odom` and `/imu`. Board is time
  synced.

---

## Diagnosis: the jerky motion

Three mechanisms. The first two are config, the third needs glide.

### 1. The stale-command brake

`COMMAND_TIMEOUT_MS` is 500 ms, and a stale command brakes hard rather than
ramping.

If teleop stops publishing on stick release rather than publishing zeros, every
release becomes: half a second coasting at the last commanded speed, then the
active brake all at once. That is the inertia and the hard stop in one event.

Check what the teleop node actually does on release before changing anything
else — it decides whether this is the main story or irrelevant.

### 2. The ramp is far slower than it feels

6 rad/s² on a 36 mm wheel is **0.22 m/s²**.

From `WHEEL_MAX_RPM 200` — roughly 0.75 m/s — a full stop takes **3.5 seconds
and about 1.3 metres**.

So if teleop *does* publish zeros, the complaint inverts: not jerk but lag, the
robot ignoring you for three seconds and drifting most of a metre past where
you let go. Same felt experience, opposite cause.

Worth noting `WHEEL_ACCEL_TIME 20` works out to roughly **52 rad/s²** at the
motor — about nine times faster than the firmware limit. The motor's own ramp
is currently doing nothing. The 6 rad/s² is the entire story.

### 3. Per-wheel ramping distorts turns

Real regardless of the first two, and the only one glide is needed for.

Both ramps are applied per-wheel, after the host has allocated. Straight-line
stops are fine — four wheels start at the same speed and reach zero together.

Stopping out of a turn is not. Left at 10 rad/s, right at 2, both ramping at
6 rad/s²: the right side reaches zero in 0.33 s, the left takes 1.67 s. For
more than a second the robot pivots around a stopped wheel, following a curve
nobody commanded.

**The ratio between wheels is the commanded curvature.** Ramping wheels
independently does not preserve it, and lowering the limit lengthens the
wrong-curvature phase rather than shortening it.

The fix is to ramp in body space and allocate afterwards.

---

# STEP A — Basic changes

Config only. No new code, no new files. Do this first and drive it for a day.

### A.1 Record the baseline first

External shot and camera feed, same route, teleop with hard stops and direction
changes. **Include a turn-to-stop** — that is where mechanism 3 shows, and it
is the before-shot for the whole change.

Once it is smooth this footage cannot be recreated.

### A.2 Find out what teleop does on release

Publishing zeros and silence produce completely different behaviour here.

```
ros2 topic echo /cmd_vel    # release the stick, watch what happens
```

If it goes silent, either make it publish zeros or accept that every release
takes the brake path. If it publishes zeros, mechanism 1 is not your problem
and mechanism 2 is.

### A.3 Raise the ramp limit

`WHEEL_ACCEL_LIMIT_RAD_S2 6.0` → try **20.0**.

That is 0.72 m/s², stopping from full speed in about 1 second and 0.37 m.
Still well under the motor's own 52 rad/s², so the motor ramp continues to do
nothing and the firmware limit stays the authority.

Sweep it: 6, 12, 20, 30. Find where stops stop feeling slow and start feeling
violent. This single constant is the highest-value experiment available before
any code is written.

### A.4 Consider the timeout

If teleop stays silent on release, 500 ms of coasting before the brake is a
long time. Lowering `COMMAND_TIMEOUT_MS` to 200 ms shortens it, at the cost of
being twitchier about link hiccups.

Prefer fixing teleop to publish zeros. The timeout exists for a crashed host,
not for normal stick release, and tuning it to paper over teleop behaviour
makes it worse at its actual job.

### A.5 What A cannot fix

The turn-to-stop curvature artifact. Any ramp rate has it; faster ramps just
make the wrong-curvature phase shorter. If turn-to-stop still looks wrong after
sweeping A.3, that is mechanism 3 and it is what step B is for.

---

# STEP B — glide

### B.1 Form

**In-tree, not a submodule.** `glide.c` / `glide.h` alongside the other files,
same one-file-per-thing shape as the rest of the firmware. Extract it to its
own repo later if it earns that — for now it is part of this robot.

It has no Pico SDK dependency: plain C99, `math.h` and fixed-width ints only,
so it also builds on a host for tests.

### B.2 Scope

Between a velocity command and per-wheel velocity setpoints. Three jobs:

1. **Command shaping** — clamp and rate-limit in body space, so the wheel ratio
   is preserved through every transient.
2. **Yaw rate closure** — correct commanded angular velocity using the gyro.
3. **Kinematic allocation** — body velocity to per-wheel rates, with an
   effective-track correction.

Not an odometry source (that is step D), not an attitude filter, not a motor
controller, not a safety layer. The command watchdog and brake-on-stale stay
where they are.

### B.3 Conventions

REP-103: x forward, y left, z up, right-handed, positive yaw counter-clockwise
from above. SI throughout.

link101 sits centred and axis-aligned in base101, so the IMU transform is
identity. The **sign** still needs verifying rather than assuming — an inverted
gyro turns the yaw loop into positive feedback and the robot spins up to the
correction clamp and stays there.

Each wheel is described by signed lateral offset from the centreline and
radius. Longitudinal position is absent on purpose: for a skid steer with fixed
wheels, a contact point's longitudinal speed depends only on its lateral
offset. Front and rear on the same side genuinely should turn at the same
speed.

### B.4 Pipeline

```
cmd_vel → clamp → jerk-limited ramp → yaw feedforward ─┐
                                                        ├→ allocate → saturate → out
gyro → bias → low-pass → yaw PI correction ────────────┘
                              ↑                              │
                              └──────── windup flag ─────────┘
```

**The ramp is before the loop.** The jerk limit applies to the operator's
command; the corrective term is added after and bounded separately by an
integrator clamp and an output clamp.

Ramping last would slew the correction, making the loop sluggish exactly when
it needs to act. Ramping nowhere would let the PI inject steps that undo the
smoothing.

**Allocation**, for wheel *i* with lateral offset *yᵢ* and radius *rᵢ*:

```
ωᵢ = (vx − wz_corrected × yᵢ × k_icr) / rᵢ
```

`k_icr` is the effective-track coefficient — 1.0 for an ideal differential
drive, typically 1.2–1.5 for skid steer. With the yaw loop active it becomes a
feedforward term the loop corrects around rather than the sole source of
accuracy.

### B.5 API

```c
#define GLIDE_MAX_WHEELS 6

typedef enum { GLIDE_ZERO_COAST, GLIDE_ZERO_HOLD } glide_zero_mode_t;
typedef enum { GLIDE_OK, GLIDE_ERR_CFG, GLIDE_ERR_DT } glide_status_t;

typedef struct {
    float y;        /* lateral offset from centreline, m, +left */
    float r;        /* wheel radius, m */
    float w_max;    /* max wheel rate, rad/s, > 0 */
} glide_wheel_t;

typedef struct {
    glide_wheel_t wheels[GLIDE_MAX_WHEELS];
    uint8_t n_wheels;

    float k_icr;
    float gear_ratio;           /* 1.0 for DDSM210 direct drive */

    float vx_max, wz_max;       /* m/s, rad/s */
    float ax_max, alpha_max;    /* m/s², rad/s² */
    float jx_max, jalpha_max;   /* m/s³, rad/s³; 0 disables */

    float yaw_kp, yaw_ki;
    float yaw_correction_max;   /* rad/s */
    float yaw_deadband;         /* rad/s */
    float yaw_lpf_hz;           /* 0 disables */
    float gyro_bias;            /* rad/s */

    glide_zero_mode_t zero_mode;
} glide_cfg_t;

typedef struct {
    float vx_cmd, wz_cmd;
    float gyro_z;               /* rad/s, body frame, bias not yet removed */
    const float *wheel_fb;      /* rad/s per wheel, NULL if unavailable */
} glide_in_t;

typedef struct { glide_cfg_t cfg; /* internal state */ } glide_t;

glide_status_t glide_init(glide_t *g, const glide_cfg_t *cfg);
glide_status_t glide_set_cfg(glide_t *g, const glide_cfg_t *cfg);
void           glide_reset(glide_t *g);

glide_status_t glide_update(glide_t *g, const glide_in_t *in,
                            float dt, float *out);

float glide_yaw_rate(const glide_t *g);    /* filtered, bias-corrected */
float glide_vx_shaped(const glide_t *g);
float glide_wz_shaped(const glide_t *g);
bool  glide_saturated(const glide_t *g);

void  glide_calibrate_gyro(glide_t *g, float sample);
int   glide_check_yaw_sign(const glide_t *g);   /* +1 ok, -1 inverted, 0 unknown */
```

`glide_yaw_rate()` matters for step D — odometry uses the same filtered
estimate rather than re-filtering the raw gyro.

`glide_init()` returns `GLIDE_ERR_CFG` on zero radius, zero wheels, negative
limits, or all wheels on one side. Fail at init rather than producing quiet
nonsense at 208 Hz.

`dt` is passed per call. Reject `dt <= 0` or `dt > 0.2` with `GLIDE_ERR_DT` and
leave state untouched.

### B.6 Behaviour

**Saturation.** Scale the whole output uniformly so the worst offender lands at
its limit. Yaw preserved, linear velocity sacrificed: follow the commanded
curvature at lower speed rather than the commanded speed along the wrong
curvature. Heading errors compound over a trajectory; speed errors do not.

**Anti-windup.** Freeze the integrator when allocation saturated this tick, or
when wheel feedback shows any wheel's error above `w_max × 0.2` for more than
200 ms. The second catches a stalled or unplugged motor.

**Reset.** Zeroes ramp state, integrator and filtered gyro. Preserves config
including `gyro_bias`, which is a property of the sensor, not the run. Call it
on watchdog trip and e-stop. Rule: if you did not send glide's output to the
motors last tick, reset before you do.

**zero_mode.** `COAST` ramps to zero then stops closing the loop — the default,
because `HOLD` makes the robot fight you when pushed by hand. `HOLD` is right
for a robot that must stay pointed while an arm works, which is a real base101
case; it just should not happen by accident.

### B.7 Where it runs

**On the existing main loop, at the IMU's 208 Hz, unless measurement says
otherwise.**

The LSM6DSOX at 208 Hz ODR with FIFO batching and hardware timestamps means dt
comes from the sensor, not from when the loop got round to reading. Main-loop
jitter stops mattering for sample spacing, and the FIFO holds seconds of gyro
data, so a blocking bus read would have to stall absurdly long to overflow it.

What remains is **latency**: a batched read means the correction acts on data
up to one batch old, which bounds how high `yaw_kp` can go before phase margin
bites. That is a real cost and a much smaller one than sample jitter.

So: single core first. If tuning runs into a `yaw_kp` ceiling that shows up as
sluggish correction, move glide and the IMU read to core 1 then, with a
measured reason. If that happens, core 1 must never call `io_poll()`, never
touch a `serial_hook`-wrapped bus, never call into zenoh, and own i2c1
exclusively — and core 0 needs a heartbeat watchdog on it, because a control
core that dies silently while the robot is moving is the worst failure
available here.

### B.8 robot.h additions

```c
// ===========================================================================
// Geometry
// ===========================================================================
#define WHEEL_RADIUS_M        0.036f
#define WHEEL_LATERAL_M       0.1192f   // centreline to wheel, half the track
#define GEAR_RATIO            1.0f      // DDSM210 is direct drive

// From calibration (step C). Re-run after any change to wheels or payload.
#define ICR_COEFF             1.30f
#define GYRO_BIAS_RAD_S       0.0f
#define GYRO_Z_SIGN           (+1)

// ===========================================================================
// glide
// ===========================================================================
#define GLIDE_HZ              208       // matches the LSM6DSOX ODR
#define GLIDE_VX_MAX          0.75f
#define GLIDE_WZ_MAX          2.0f
#define GLIDE_AX_MAX          0.7f
#define GLIDE_ALPHA_MAX       3.0f
#define GLIDE_JX_MAX          3.5f
#define GLIDE_JALPHA_MAX      15.0f
#define GLIDE_YAW_KP          0.8f
#define GLIDE_YAW_KI          0.5f
#define GLIDE_YAW_CORR_MAX    1.0f
#define GLIDE_YAW_DEADBAND    0.02f
#define GLIDE_YAW_LPF_HZ      20.0f
#define GLIDE_ZERO_MODE       GLIDE_ZERO_COAST

#define TOPIC_CMD_VEL         "cmd_vel"
```

Wheel lateral offsets come from `WHEEL_LATERAL_M` and the existing
`WHEELS[].direction` sign. No new per-wheel table.

`GLIDE_AX_MAX` at 0.7 m/s² is roughly the 20 rad/s² from step A.3, expressed in
body units. Carry across whatever A.3 actually landed on.

### B.9 Integration

`cmd_vel` replaces `base_cmd` — the host is already publishing it, so there is
no transition period and no dual-topic phase.

**Remove the ramp from `wheels_update()`.** Keeping it means double-ramping,
and the per-wheel ramp is exactly the curvature distortion glide exists to fix.

`WHEEL_ACCEL_TIME` stays: it is the motor's own smoothing, below everything,
and at ~52 rad/s² it is far enough above the loop not to fight it.

The command watchdog, `COMMAND_TIMEOUT_MS` and the brake-on-stale behaviour all
stay exactly as they are. Add `glide_reset()` to the watchdog trip path.

### B.10 Host tests

| Test | Asserts |
| --- | --- |
| Straight line | All wheels equal, `k_icr` irrelevant |
| Pure rotation | Wheels equal and opposite, scales with `k_icr` |
| Ramp bound | No tick's velocity delta exceeds `ax_max × dt` |
| Jerk bound | No tick's accel delta exceeds `jx_max × dt` |
| **Curvature preserved** | Step to zero from a turn; wheel ratio constant throughout |
| Saturation ratio | Ratio preserved, worst wheel at `w_max` |
| Yaw convergence | 30% simulated slip; reaches command within 0.5 s |
| Anti-windup | Hold saturation 5 s, release, no overshoot spike |
| Reset | Wind integrator, reset, first output equals cold-start |
| dt guard | 0, negative, 1.0 → `GLIDE_ERR_DT`, state untouched |
| Bias rejection | Inject 0.01 rad/s, calibrate, straight-line symmetric |
| Config validation | Zero radius, zero wheels, all one side → `GLIDE_ERR_CFG` |

**Write the curvature test first.** It is the one that would have caught the
current bug, and it is the regression that matters.

Yaw convergence needs a toy plant — commanded wheel rate produces actual yaw at
some slip factor. Thirty lines, and it makes the loop testable without a robot.

### B.11 Tuning order

Each step depends on the one before.

1. **Gyro bias** from calibration. Verify: zero command 30 s, reported yaw rate
   under 0.005 rad/s.
2. **Yaw sign** must read +1. Everything downstream is nonsense otherwise.
3. **`k_icr`** with `yaw_kp` and `yaw_ki` at zero, so allocation runs open-loop.
4. **Ramp limits**, still open-loop. Carry over from step A.3.
5. **`yaw_kp`** with `yaw_ki` off. Raise from 0.5 until it oscillates in yaw
   during a turn, take about 60%.
6. **`yaw_ki`** until steady-state yaw error during a sustained turn goes away.
   Too high shows as slow hunting after the turn ends.
7. **Second surface**, no retuning. If accuracy collapses, the gains are too low
   and the loop is not doing its job.

### B.12 Acceptance

- Commanded 90° in-place turn within 3°, ten trials, both directions.
- Same within 5° on a second surface, no retuning.
- 3 m straight line ends within 5 cm of centreline.
- **Turn-to-stop holds its curvature.**
- Full-speed-to-zero stop: no visible lurch, no wheel chirp.
- Camera footage through the same stop shows no frame-level smear.

---

# STEP C — Calibration

### C.1 Form

**A separate sketch, not a firmware mode.** Own `pico-sdk` project reusing
`hardware_link101` and `pico_ddsm`. Prints constants to USB CDC for pasting
into `robot.h`.

This matches the repo's philosophy rather than working around it: config mode
and flash persistence were removed deliberately, and `robot.h` is where the
robot's numbers live. A calibration routine that wrote flash would walk that
decision back for nothing.

The product version — button trigger, LED vocabulary, flash A/B slots, geometry
hash — is deferred until base101 ships to people who will not be recompiling.
Everything in C.4 and C.5 still applies to it.

### C.2 What it measures

| Output | Phase | Feeds |
| --- | --- | --- |
| Gyro bias | Stationary | `GYRO_BIAS_RAD_S` |
| Yaw sign | Spin | `GYRO_Z_SIGN` |
| `k_icr` CW and CCW | Spin | `ICR_COEFF` |
| Left/right wheel ratio | Straight | Per-wheel scale, and step D |
| Achievable deceleration | Brake | `GLIDE_AX_MAX` ceiling |
| Brake-vs-coast delta | Brake | Whether the brake command earns its place |
| Per-wheel no-load speed, current, latency | Wheels-up | Diagnostics, `yaw_kp` bound |

**The gyro is both actuator witness and measuring instrument.** Wheel speeds
predict a yaw rate; the gyro measures the real one; the ratio is `k_icr`. No
lidar, no scan matching, no tape on the floor.

**Cannot measure absolute wheel radius** — the gyro gives no length reference,
only the ratio between sides. Absolute radius stays a caliper measurement, and
step D's linear odometry scales directly with it.

### C.3 Space and time

**Every translating phase goes out and comes back**, so excursions do not
accumulate. A drifting robot that never returns to origin is how a calibration
run ends inside a sofa.

**Required clear area: 2.0 m × 1.5 m**, robot centred.

| Phase | Nominal | Hard cap | Returns |
| --- | --- | --- | --- |
| Bias | 0 | 0 | — |
| Wheel check | ±5 cm | 10 cm | Yes, alternating |
| In-place spin | 0 | 30 cm wander | No, bounded by time |
| Straight | 50 cm each way | 70 cm | Yes |
| Brake | 70 cm each way | 90 cm | Yes |

The spin phase cheats — a skid steer rotating in place wanders unpredictably.
Budget for it rather than assuming it away.

**Every moving phase is bounded by distance and time, whichever trips first.**
Distance comes from integrated wheel feedback, the very thing being calibrated
and therefore untrusted. That is acceptable: it is a safety bound, not a
measurement, and 30% error in an untrusted estimate still stops the robot well
inside the margin. Time is the real backstop — commanded speed and distance cap
× 1.5.

Total roughly 90 s. A five-minute calibration gets run once, on the wrong
surface, and never again.

### C.4 Phases

**0. Wheels up** (separate mode, robot propped). Someone has to lift the robot,
so it is not part of the floor run. Gives what floor phases cannot separate:

- Commanded versus actual wheel speed with no load. Without it, an
  under-rotating wheel is ambiguous: slip, or a motor that never reached
  setpoint?
- Per-wheel minimum turning speed — the floor below which the yaw loop cannot
  act, and it varies between motors.
- No-load current per wheel. On-ground current minus this is load, much cleaner
  than raw current. Also catches a dragging bearing.
- Command-to-feedback latency. Loop dead time bounds `yaw_kp`, turning gain
  tuning into arithmetic instead of guesswork.

Natural home for the start-up click test: same setup, same session. Persisting
with wheels free points at something electrical; only on the ground points at
mechanical slack taking up under reversing torque.

**1. Pre-flight.** All motors answer. IMU plausible. Battery holds under a
brief load pulse — a sagging pack produces a calibration that is really a
measurement of voltage droop. Nothing moves.

**2. Gyro bias.** Three seconds stationary, running mean. Compute variance too:
above the noise floor means someone is holding the robot. Abort rather than
baking motion into the bias.

**3. Per-wheel check.** Each wheel alone, slowly, forward then reverse.
Confirms every motor responds and feedback sign matches command sign. Catches a
swapped assignment before the robot tries to spin with two wheels fighting.
Watch the gyro: one wheel driving should yaw in a predictable direction.

**4. In-place spin.** Three yaw rates, both directions, several seconds each.
Ratio of predicted to measured yaw is `k_icr`. Both directions because they
often differ — weight distribution, a dragging bearing. Disagreement over ~10%
is a mechanical finding, not a calibration constant: record both and flag it.
Discard the first second of each run; acceleration transient is not
steady-state slip.

**5. Straight run.** Commanded straight at two speeds, out and back, using the
`k_icr` just measured. The gyro should read zero; whatever it reads is the
left/right asymmetry. Out and back distinguishes real asymmetry, which reverses
sign relative to the body, from floor slope, which does not.

**6. Brake test.** Accelerate to a fixed speed over bounded distance, then stop
two ways: command zero, then the DDSM210 brake. Time and distance to standstill
for both, from wheel feedback. Answers whether the brake command is worth using
and what deceleration is actually achievable — which bounds `GLIDE_AX_MAX`
honestly. Configuring a limit the platform cannot deliver just means silently
overshooting every stop.

### C.5 Abort

| Trigger | Response |
| --- | --- |
| Keypress on the debug console | Brake, unwind, idle |
| Physical e-stop | Brake immediately, no unwind |
| Any motor stops answering | Brake all, fault |
| IMU stale > 50 ms | Brake all, fault |
| Excursion over cap | Brake, unwind, fault |
| Phase timer expired | Brake, end phase |
| Battery below threshold | Brake, fault |
| Total timeout, 3 min | Brake, fault |
| Gyro over ±5 rad/s | Brake, fault — picked up or hit |

**The abort keypress must work during every phase**, polled independently of
phase logic. Even in a throwaway sketch: it drives a 12 kg robot across a floor
with furniture on it.

**Unwind** on graceful abort mid-translation: back toward origin at half speed
under the same caps. Best-effort on an uncalibrated estimate — the job is
getting the robot out of the middle of the floor, not precision. On e-stop or
motor fault, do not unwind: something is wrong with the drivetrain and
commanding more motion is the wrong instinct.

### C.6 Validation

**Repeatability, not single measurements.** Every coefficient from at least
three runs. Median, and record the spread. If spread exceeds threshold, the
phase fails — a value that does not repeat is not a measurement, and the mean
of three inconsistent runs looks exactly as trustworthy as the mean of three
consistent ones.

**Plausibility bounds** — sanity checks against a wiring fault or a wheel in
the air, not tuning parameters.

| Quantity | Accept | Outside means |
| --- | --- | --- |
| `k_icr` | 0.8–2.5 | Geometry wrong, or a wheel not touching |
| Gyro bias | < 0.05 rad/s | IMU fault, or robot moved |
| CW vs CCW `k_icr` | within 15% | Mechanical asymmetry — warn, record both |
| Radius ratio | 0.9–1.1 | Wrong wheel size, or bad slip |
| Deceleration | > 0.2 m/s² | Brakes ineffective or feedback wrong |

**Warn versus fail.** A CW/CCW mismatch is a real finding — record, flag, carry
on. A `k_icr` of 4.0 is a bug and must not reach `robot.h`. A routine that
fails often gets skipped, and a skipped calibration is worse than a flagged one.

### C.7 Output

```
=== base101 calibration complete ===
surface: <operator note>
runs: 3    duration: 94 s

#define ICR_COEFF        1.31f    // CW 1.29, CCW 1.33, spread 3.1%
#define GYRO_BIAS_RAD_S  0.0043f  // spread 0.0002
#define GYRO_Z_SIGN      (+1)

// informational
// left/right wheel ratio  0.998   (within tolerance, no scale applied)
// decel, coast            0.61 m/s²
// decel, brake            1.42 m/s²   → GLIDE_AX_MAX ceiling ~1.2
// WARN: CW/CCW spread 3.1% — within tolerance
```

Paste into `robot.h`, reflash. That is the whole persistence story, and it is
the right one for now.

---

# STEP D — Odometry and publishers

The host expects `/odom` and `/imu`. The board is time synced, so stamps are
real and `/odom` can feed TF without the host re-stamping.

### D.1 The split that matters

**Wheels for linear, gyro for yaw.** Never wheel-derived yaw.

This is the whole point. On a skid steer, yaw from wheel speeds is wrong by
`k_icr` and varies with surface — it is exactly the quantity the calibration
exists to correct and the yaw loop exists to close around. The gyro measures it
directly.

```
vx  = mean(ωᵢ × rᵢ × directionᵢ) over all four wheels
vy  = 0
wz  = glide_yaw_rate()
```

`vy = 0` is the nonholonomic assumption. A skid steer does slip sideways, but
nothing on the robot can observe it, so claiming zero is honest.

Use `glide_yaw_rate()` rather than re-filtering the raw gyro — same bias
correction, same low-pass, one source of truth.

### D.2 Integration

**Integrate at loop rate, publish at 50 Hz.** Integrating only at publish rate
throws away most of the information and accumulates error fast.

```
θ  += wz × dt
x  += vx × cos(θ) × dt
y  += vx × sin(θ) × dt
```

Midpoint on θ — using `θ + wz×dt/2` in the trig — is measurably better during
turns and costs nothing. Worth doing.

Wrap θ to (−π, π] on every update, not on publish.

### D.3 Message

`nav_msgs/Odometry`, `frame_id: odom`, `child_frame_id: base_link`.

Pose carries the integrated x, y and θ as a quaternion about z. Twist carries
`vx`, `vy = 0`, `wz` — in the **child frame**, which is what the message
expects and a common thing to get wrong.

**Covariances encode the asymmetry.** Yaw is measured and good; position is
dead-reckoned and drifts. Do not publish the same number for both.

| Element | Value | Why |
| --- | --- | --- |
| pose x, y | 0.05 | Dead reckoning, scales with radius error |
| pose θ | 0.01 | Gyro-derived, much better |
| pose z, roll, pitch | 1e6 | Not estimated |
| twist vx | 0.01 | Wheel feedback, direct |
| twist vy | 1e6 | Not estimated |
| twist wz | 0.002 | Gyro, direct |

Large values rather than zero for unestimated terms — zero reads as "perfectly
known" to anything consuming this downstream.

These are nominal. If you later fuse on the host, they are the numbers that
decide how much the fusion trusts you, so they are worth revisiting once drift
has actually been measured over a known path.

### D.4 Who publishes TF

**Decide explicitly, and only one of them does it.** Two nodes publishing
`odom → base_link` produces a TF tree that flickers between two answers and
symptoms that look like anything but the real cause.

Firmware publishing it is simplest given the board is time synced. If the host
later runs `robot_localization` fusing `/odom` and `/imu`, the EKF owns that
transform and firmware must stop. Put it behind a `robot.h` flag so the switch
is one constant rather than a code change:

```c
#define PUBLISH_ODOM_TF   true
#define TOPIC_ODOM        "odom"
```

### D.5 Reset

Pose accumulates from boot and drifts without bound — expected, and why `odom`
is not a global frame.

Provide a reset path anyway: a zenoh-triggered service or topic that zeroes x,
y and θ. Useful during calibration, and useful for anyone re-running a test
without power cycling.

Reset pose only. Do not touch the gyro bias, which is a property of the sensor.

### D.6 /imu

Largely unchanged. Keep `orientation_covariance[0] = -1` — the LSM6DSOX does no
fusion and nothing in this work adds any.

One change: publish the **bias-corrected** angular velocity, matching what glide
uses, and say so in the field. A host consumer seeing a different yaw rate in
`/imu` than the one implied by `/odom` has a real inconsistency to chase.

The magnetometer is unused by any of this. Leave `/imu/mag` publishing as-is or
drop it — no interaction either way.

### D.7 Rates

| What | Rate | Why |
| --- | --- | --- |
| Odometry integration | 208 Hz | Loop rate, in the glide tick |
| `/odom` publish | 50 Hz | Matches existing publish rates |
| `/imu` publish | 50 Hz | Unchanged |
| TF broadcast | 50 Hz | With `/odom`, same stamp |

### D.8 Acceptance

- Robot pushed 2 m by hand along a straight line: `/odom` x within 10 cm, y
  under 5 cm.
- Driven 3 m under power: same tolerance. A gap between pushed and driven is
  wheel slip, which is real and worth knowing the size of.
- Commanded 360° in place: `/odom` θ within 5°, and the error does not grow
  across ten consecutive turns. Growth here means residual gyro bias, not
  `k_icr`.
- Stationary for 5 minutes: θ drift under 1°. This is the bias test, and it is
  the one that catches a calibration that was run while someone leaned on the
  robot.
- `ros2 run tf2_tools view_frames` shows exactly one publisher for
  `odom → base_link`.

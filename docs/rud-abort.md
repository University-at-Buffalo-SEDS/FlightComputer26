# Evaluation abort and the break-up (RUD) detector

An *evaluation abort* stops the flight computer's estimator. After it, the
FC makes no automatic deployment: no drogue at apogee, no main at 457 m. The
operator's manual Deploy and Expand still work.

Chutes are critical, so the abort has to work in every flight phase but
must only fire automatically when the readings are clearly haywire, as in a
rapid unscheduled disassembly (RUD): the vehicle breaking up, tumbling,
with readings no intact flight produces. It must never fire on noise,
transients, one bad sensor, a dropout, or a deployment or landing shock.

## 1. What stops automatic deployment

Audit of every path, origin/dev 592785b plus this branch.

| Path | Origin | Trigger and threshold | Persistence / voting | Effect | Phases |
| --- | --- | --- | --- | --- | --- |
| `EvaluationAbort` (FC decree `Evaluation_Abort`, byte 16) | operator only: an explicit GS command. The GS sends it on no automatic path. It is also allowed while the station Abort is latched (`gse.rs` `abort_safe_control`) | the press | none (a deliberate command) | `fc_recovery_request_eval_stop`: the estimator stops (`Eval_Abort_Flag`) | before this branch, ascent only: the descent state machine ran in the distribution thread and still fired the main. Now every phase (section 3) |
| Station Abort / manual Stop (`ABORT`) | operator | the press | none | `manual_stop`: `fc_recovery_holds_sequence`, so `fc_recovery_inhibits_pyro` refuses every automatic pyro; estimator quiesced | every phase (unchanged) |
| Reset to Startup (`BOARD_RECOVERY_REQUEST` op 3) | operator, via recovery transport | the request | none | estimator stopped, state back to Startup | pad / after flight (unchanged) |
| Bench monitor lease | wired probe | lease | pad only | outputs held | pad (unchanged) |
| Break-up detector (new, section 2) | automatic | IMU and barometer both implausible | about 1 s window, two independent sensors | as `EvaluationAbort` | armed Launch to Apogee |

Automatic paths that do not stop deployment, listed because they react to
bad readings:

| Path | Trigger | Persistence | Effect |
| --- | --- | --- | --- |
| Sensor failure count (`process_sensor_report`) | out-of-band samples (`validate_*`: accel outside ±24 g, gyro outside ±2000 deg/s, pressure outside 57.2–110 kPa, altitude outside −30..4572 m). A saturated BMI088 reads at most its full scale, so saturation never counts | a running count, one up per bad sample and one down per good one (`Reset_Failures`, on by default). Only before Apogee (or for a bad barometer without trustworthy GPS); IMU and baro are validated only in ascent and on the pad | ≥ 40: sensor re-initialisation; ≥ 120: on the pad `In_Aborted_State` (await a command); in flight a sensor restart that restarts the estimator, then a ×10 threshold. Never stops deployment |
| GPS delayed/malformed | `GPS_DELAY_MS` 200 ms gaps or bad packets, 40 of either | count | barometer-only Vigilant mode; deployment continues |
| Filter fluctuation | `kf_deviations` > 80 | count | Vigilant mode trusts the raw barometer; deployment continues |
| Watchdog resume rejected | checkpoint invalid at boot | – | holds the sequence at boot (no flight in progress) |

## 2. Break-up detector (`Core/Src/rud_detect.c`)

It is evaluated on every estimator step (one per barometer sample, 25 Hz),
over the last 25 steps (1 s). It is armed from Launch until Apogee while the
ascent filter (so the IMU) runs. It fires once, when both votes hold at the
same step:

| Vote | Sensor | Condition on a step | Needed | Intact flight (IGT datasets, armed) | Margin |
| --- | --- | --- | --- | --- | --- |
| motion | IMU (BMI088: gyroscope and accelerometer share one package, so they count as one sensor) | tumbling, \|gyro x,y\| ≥ 360 deg/s (pitch/yaw; z is the long axis), or lateral load \|accel x,y\| ≥ 49 m/s² (5 g) | 22 of 25 steps (0.88 s) | ≤ 20 deg/s and ≤ 6.1 m/s² (off_nominal's 10 ms IMU spike: one vote) | 18× and 8× on the readings |
| pressure | barometer | \|altitude step\| / dt ≥ 450 m/s | 12 of 25 steps | ≤ 224 m/s (max speed 220 m/s); outliers and steps give single votes (5 at most in the noisiest dataset) | 2× on the rate |

Why it cannot fire on the listed non-events:

- **One sensor alone:** both votes are needed. A saturated or stuck IMU, or
  a wild or frozen barometer, never fires. Once a vote is lost it has to be
  rebuilt over the window.
- **Transients:** the motion vote needs 0.88 s of the last second. A 0.3 s
  saturation gives 8 of the 22 votes needed, and a pressure step or outlier
  gives 2.
- **Deployment and landing shocks:** the detector disarms at Apogee, before
  the drogue fires.
- **Dropouts:** non-finite readings never vote. A missing barometer sample
  is skipped (the rate needs two samples), and a frozen one gives rate 0.
- **Estimator divergence:** not a vote. It derives from the same IMU and
  barometer, and Vigilant mode already handles it.

After Apogee only the operator can abort. Under a parachute the only
automatic action left is the main, and stopping it on sensor trouble would
be the worse failure.

Two simultaneous faults on independent sensors can still fire it: an IMU
stuck at full scale for a whole second while the barometer gives
implausible steps on half the samples. The sweep below does not reach that
combination (closest: 36 %).

## 3. Effect on the estimator

The estimator (evaluation thread) now runs the descent filter and the whole
state machine (docs/task-layout.md step 9). An abort, operator or automatic,
stops it in every phase:

- *Before:* `EvaluationAbort` in descent stopped only the idle ascent
  evaluator, and the descent state machine in the distribution thread still
  fired the main.
- *Now:* it stops the descent state machine too. The automatic detector
  calls the same `fc_recovery_request_eval_stop()`, logs
  `RUD: break-up detected; automatic deployment stopped` (ordered message:
  telemetry and SD), and counts `g_fc_rud_aborts`.

## 4. Evidence: `tools/rud_sweep.py`

The sweep replays every IGT dataset through the host build of the real
estimator, state machine and detector (`tools/igt_replay_host.py
--rud-margins`).

**Must not fire:** 55 cases, no abort, drogue and main in every one. The 5
datasets as given, each also with these faults:

- barometer outliers on 10 % of samples (±15–40 m);
- 60 m barometer bursts at max-Q and in coast;
- barometer frozen 3 s, and a 2 s barometer gap;
- full-scale IMU saturation for 0.3 s, three times;
- IMU stuck at full scale for 3 s;
- saturation together with barometer bursts;
- deployment and landing shocks (IMU at full scale, −110 m pressure pulse);
- GPS lost after liftoff;
- heavy barometer and IMU noise.

**Must fire:** 9 cases, abort before Apogee in all 9. These are synthetic
break-ups at max-Q and in coast (900 deg/s tumble, 120 m/s² rotating
lateral load, ±600 m port pressure swing) and a tumble with random
barometer jumps, on nominal_10k, measured_motor and high_wind. The detector
fires 0.88–0.90 s after the break-up starts (the motion vote's hold time).

**One sensor only:** 6 cases, no abort: the IMU tumble alone, and the
barometer chaos alone.

Result: 70/70 cases as expected. In the must-not-fire cases, the closest
both votes came at the same step is 36 % of firing (IMU saturation 0.3 s
together with barometer bursts).

Reproduce:

    python3 tools/rud_sweep.py <GroundStation26>/test-data/igt --json out.json

Host test of the trigger rule: `tests/test_rud_detect.py`. Estimator loop
with both filters: `tests/test_estimator_loop.py`.

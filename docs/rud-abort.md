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
| Sensor plausibility (sections 5, 7) | an implausible, frozen or silent barometer; an untrusted ascent velocity, a frozen or silent IMU | 0.2-2 s | the filters fall back on the other sensors; deployment continues |
| Dead-reckoned descent (section 6) | no trustworthy altitude in descent | per step | times the main from the last trustworthy altitude |
| Drogue timer (section 7) | no drogue 30 s after Launch | – | fires the drogue |
| Watchdog resume rejected | checkpoint invalid at boot | – | holds the sequence at boot (no flight in progress) |

## 2. Break-up detector (`Core/Src/rud_detect.c`)

It is evaluated on every estimator step (one per barometer sample, 25 Hz),
over the last 25 steps (1 s). It is armed from Launch until Apogee while the
ascent filter (so the IMU) runs. It fires once, when both votes hold at the
same step:

| Vote | Sensor | Condition on a step | Needed | Intact flight (IGT datasets, armed) | Margin |
| --- | --- | --- | --- | --- | --- |
| motion | IMU (BMI088: gyroscope and accelerometer share one package, so they count as one sensor) | tumbling, \|gyro x,y\| ≥ 360 deg/s (pitch/yaw; z is the long axis), or lateral load \|accel x,y\| ≥ 49 m/s² (5 g) | 22 of 25 steps (0.88 s) | 0 votes in four datasets (readings ≤ 20 deg/s, ≤ 6.1 m/s²); 1 vote in off_nominal (see below) |
| pressure | barometer | \|altitude step\| / dt ≥ 450 m/s | 12 of 25 steps | 0 votes in four datasets (≤ 224 m/s; max speed 220 m/s); 5 votes in early_apogee_noise (outliers, a pressure step) |

The per-step thresholds are not the filter, the vote counts are. A single
reading can be far past a threshold in an intact flight:

- off_nominal has a 10 ms IMU spike during the coast (2500 deg/s, 400 m/s²,
  clipped by the sensor). It falls on one estimator step: the detector saw
  1800 deg/s and 212 m/s² and counted one vote of the 22 needed.
- early_apogee_noise has barometer outliers. Single steps reach 1115 m/s,
  for 5 votes of the 12 needed.

`tools/rud_sweep.py` prints these per-dataset raw maxima next to the votes.
So one implausible reading, or a short burst of them, never comes close.
What protects against a false abort is the persistence (0.88 s of
the IMU, half the window of the barometer) and the need for both sensors at
once.

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

**Must not fire:** 55 cases, no abort, and both chutes at sane times
(below). The 5 datasets as given, each also with these faults:

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

**One sensor failing (no abort expected), 24 cases on nominal_10k,
measured_motor, high_wind and off_nominal:**

- an IMU tumble alone, an IMU frozen at rest, a 3 g accelerometer bias;
- barometer chaos alone, a barometer frozen 10 s after apogee, 200 m
  barometer noise from apogee on.

**Must fire:** the 3 break-ups on each of those 4 flights, 12 cases.

"Sane" is checked against each dataset's truth (the replay is open loop):
the drogue within 1 s before to 4 s after the true apogee, and the main at a
true altitude of 457.2 ± 100 m (4 s under the drogue; section 6.3 explains
why altitude and not time).

Result (this section's 91 cases; with sections 6 and 7, 261/261):

- In all 79 no-abort cases the drogue comes 0.8–2.3 s after the true apogee
  and the main within ±0.25 s of the 457.2 m crossing.
- All 12 break-ups abort before Apogee, 0.88–0.90 s after onset.
- The closest any no-abort case came to firing is 36 % (over all 234
  must-not-fire cases, sections 6 and 7 included).

Reproduce:

    python3 tools/rud_sweep.py <GroundStation26>/test-data/igt --json out.json

Host test of the trigger rule: `tests/test_rud_detect.py`. Single-sensor
verdicts: `tests/test_sensor_plausibility.py`. Estimator loop with both
filters: `tests/test_estimator_loop.py`.

## 5. One faulty sensor must not cost the chutes (`Core/Src/sensor_plausibility.c`)

The first sweep found a critical gap that predates this branch: one sensor
failing for the rest of the flight could stop a deployment with no abort.
The same cases fail identically on origin/dev 592785b, and on this branch
before the fix (16 of 24 cases):

| Single-sensor fault | Before (drogue / main vs truth) | After |
| --- | --- | --- |
| IMU tumbling from the coast on | **no apogee, no drogue, no main** (all 4 flights) | drogue +2.2–2.3 s, main +0.1 s |
| barometer chaos from the coast on | **no drogue/main** (2 flights), **drogue +215–230 s, main at the ground** (2) | drogue +1.1–1.5 s, main −0.2–+0.2 s |
| barometer frozen 10 s after apogee | drogue on time, **main 111–127 s late (at the ground)** | main −0.2–+0.2 s |
| 200 m barometer noise from apogee on | drogue +5–19 s, main −6–+3 s | drogue +0.8–1.9 s, main ±0.2 s |
| IMU frozen at rest, 3 g accelerometer bias | already fine (Vigilant mode) | unchanged |

Causes:

- **IMU fault:** the ascent EKF's altitude follows the barometer, but its
  velocity follows the bad IMU (it stayed at +40 m/s after apogee). The
  velocity-based Apogee check never passes, and Vigilant mode trusted that
  velocity.
- **Barometer fault:** the filters take every sample. In ascent a few wild
  samples throw the EKF velocity off by 200 m/s. In descent the descent
  filter's only altitude input is the barometer, so a frozen or wild
  barometer freezes or randomises the descent.

The fix: two verdicts, kept by the estimator on every step:

| Verdict | Condition | Hold / clear | Action |
| --- | --- | --- | --- |
| barometer implausible | its altitude moves faster than 450 m/s on 12 of the last 25 steps, or repeats the bit-identical value for 50 steps (2 s) | clears at ≤ 2 such steps in the window, or when it moves again | ascent: IMU dead reckoning (`ascent_hold`). Descent: GPS altitude above the pad while a fix is fresh (< 2 s) and moving, else the dead-reckoned descent of section 6 (it was the prediction, `descent_hold`). From Landed on the barometer is always taken, as before |
| barometer sample unusable (ascent) | that sample moved faster than 450 m/s; or, while any such step is in the window, it is more than 150 m from the prediction | per sample | that step dead-reckons |
| ascent velocity untrusted | the EKF velocity differs from the trend of its own altitude by more than 40 m/s on 25 consecutive steps (1 s) | 25 agreeing steps | Vigilant's Apogee check runs on the barometer and the altitude trend, and runs even with Vigilant mode off |

Margins on the intact datasets:

- **Velocity verdict:** EKF velocity against its altitude trend stays within
  14 m/s in four datasets. early_apogee_noise reaches 273 m/s on single
  steps, but its longest run above 40 m/s is 13 steps, against the 25
  needed.
- **Barometer verdict:** pressure votes as in section 2. No dataset repeats
  an altitude.
- **IGT replay:**
  - high_wind, measured_motor, nominal_10k and off_nominal replay
    byte-identical.
  - early_apogee_noise keeps every deployment decision and time. Coast is
    reached 40 ms earlier and the estimated maximum velocity is 230.4 →
    229.7 m/s at 15.84 → 15.00 s, because its barometer outliers are no
    longer fed to the ascent filter.

Limits:

- ~~A barometer that fails in descent with no GPS is dead-reckoned from the
  last good descent velocity.~~ The sweep (section 6) showed it did not
  work: a frozen barometer had pulled the descent velocity to zero before
  its verdict, so the prediction never descended (main at the ground, or no
  drogue).
- ~~Two failed sensors are beyond this.~~ Section 7. A break-up is
  section 2.

## 6. A barometer failing in descent without GPS

### 6.1 What existed

In descent the barometer is the only altitude input. When it was
implausible or frozen and no GPS fix was fresh, the filter kept its
prediction at its last velocity (`descent_hold`).

The sweep (6.4) found, on origin/dev, with GPS absent or stale (a held
fix):

| Barometer in descent | origin/dev |
| --- | --- |
| frozen at apogee, or wild (200 m noise) from apogee | **no drogue, no main** (all datasets) |
| frozen or wild 10 s after apogee | main 83-125 s late (at the ground) |

The cause: until the barometer's verdict (2 s frozen, 0.5 s wild) the
filter takes its samples, which pull the velocity to zero or throw it. The
prediction then never descends. A stale GPS was also taken: a fix counted
as fresh while its packets kept arriving, held value or not.

### 6.2 Behaviour

**Dead-reckoned descent** (`deploy_backup.h`). With no trustworthy
altitude (barometer implausible or frozen, and no fresh, moving GPS fix),
the descent predicts from the last trustworthy altitude. That is the last
step whose barometer sample moved at a plausible rate, with no implausible
step in the window and no repeat, or a GPS step. The prediction is built
to stay under the true altitude, so the main, fired when it crosses 457 m,
is early and never late:

| Phase | Prediction |
| --- | --- |
| may still climb | holds, until the earliest apogee from the reference velocity at 2 g (coast: gravity and at most 0.54 g of drag, IGT) |
| before the drogue | free fall without drag (at most 150 m/s) |
| under the drogue | 28.75 m/s: the model's 25.0 m/s (23.9-25.0 apogee to 457 m in the IGT flights) plus 15 % (rate ~ 1/sqrt(Cd rho): Cd 15 %, density 5 %); or a faster measured rate under a settled drogue; slowing from a free fall at no more than 3 g |
| under the main | 7.7 m/s (6.3-6.7 plus 15 %), stopping at the pad, so Landed comes |

From a reference at the drogue the main therefore comes
(h_ref - 457 m) / 28.75 m/s after it. That is early by up to 25 % of the
descent: 15 % from the bias, on a rate that may itself be 15 % slow.

**GPS.** A fix counts only while fresh (2 s) and while its altitude moves
(changed within 2 s). A receiver repeating its last fix is stale.

**A silent barometer** (no sample for 200 ms) gets a step without it every
barometer period, from section 7 on.

### 6.3 Tolerances

The checks are on the true altitude when the main fires, read from the
dataset's own barometer. They are not on its time: the replay is open loop,
and the truth vehicle opens its main at 457 m, so time below that is not
descent time.

- **Drogue:** within -1..+4 s of the true apogee.
- **Main, measured** (barometer or GPS): 457 ± 100 m, 4 s under the drogue.
- **Main, dead-reckoned:** from 357 m up to 457 m plus 25 % of the descent
  from the reference, plus 100 m.
- **Main, barometer offset:** 457 m − offset ± 100 m. The offset is
  unobservable without GPS: a plausible barometer is trusted, so the FC is
  no worse than its barometer.

`tests/test_deploy_backup.py` checks the "never late" property where the
open-loop replay cannot: 180 simulated flights with drag. They cover
references in coast, at apogee and under the drogue; drogues on time and up
to 10 s late; drogue rates 15 % either side of the model; and falling
bodies with little or much drag. The main never comes below 456 m: one
40 ms step of rounding when the true rate is the bound itself.

### 6.4 Sweep

`tools/rud_sweep.py` adds 80 cases, on all five datasets: the barometer
frozen at apogee, 10 s after it and at 830 m; wild (200 m noise) from
apogee and 10 s after it; silent 10 s after it (section 7); offset ±100 m
20 s after it. Each with GPS absent and with GPS stale (a held fix).

All are as expected (261/261 with section 7), and the 79 must-not-fire
cases of sections 4-5 keep their deployments.

| Failure, GPS absent or stale | Drogue after the true apogee | Main, true altitude |
| --- | --- | --- |
| frozen at apogee | +2.1 s | 626-768 m |
| frozen 10 s after apogee | +1.3 s | 586-736 m |
| frozen at 830 m | +1.3 s | 511-515 m |
| wild from apogee | +0.7-0.9 s | 642-784 m |
| wild or silent 10 s after apogee | +1.3 s | 588-737 m |
| offset +100 m (reads high) | +1.3 s | 355-357 m (the barometer's) |
| offset −100 m | +1.3 s | 555 m (the barometer's) |

The sweep now also warns when the Apogee state (the switch to the descent
filter) comes more than 2 s before the true apogee. On origin/dev the
existing `baro_freeze_3s` case has it 5.4-13.4 s early on every dataset,
and early_apogee_noise has it with IMU saturation. The drogue still waits
for the descent filter, and GPS saved it there; without GPS it would not
have. Section 7 covers both.

Reproduce: `python3 tools/rud_sweep.py <GroundStation26>/test-data/igt`.

## 7. Silent sensors, two sensors failing, and the drogue timer

### 7.1 What existed

There was no time-based backup:

- **Vigilant mode** (`vigilant_watchdog`, on by default) is a second Apogee
  check on the barometer and the altitude trend, not a timer. It needs a
  working barometer.
- **No apogee timeout.** `MOTOR_BURN_TIME` (3.5 s) only gates Coast.
- **The estimator waited forever** for its samples (`TX_WAIT_FOREVER`). Any
  of these stalled the whole state machine:
  - a silent IMU in ascent (no data-ready);
  - a silent barometer in ascent, or one reading outside the validation
    band (rejected in `for_ascent_update`, never offered);
  - a silent barometer in descent (GPS stages a step only outside Vigilant
    mode).

  The hardware watchdog would then restart the board every 16 s, and the
  restart resumes into the same wait. The host replay could not show this:
  it always supplied samples.
- **A barometer frozen in ascent** fed its repeats to the filter until its
  2 s verdict, pulling the velocity to zero: a false Apogee 5-13 s early (the
  existing `baro_freeze_3s` case, section 6.4). Without GPS the dead-reckoned
  descent (section 6) would then fire the drogue at speed.
- **A wild barometer in ascent** had its samples within 150 m of the
  prediction used until its verdict (0.5 s). With GPS stale, that put a
  drogue 3.7 s early.

### 7.2 Behaviour

**Silent sensors.** The estimator now wakes at least every barometer period
(40 ms, `EST_STEP_MS`). A sensor that delivers nothing for 200 ms
(`EST_SILENT_MS`) is silent:

- A silent IMU in ascent: the steps run on the barometer alone, with the
  velocity from its trend. The filter velocity is untrusted, so Vigilant's
  Apogee check decides.
- A silent barometer in ascent: the staged prediction is updated without it
  (IMU dead reckoning), once per period. Each missing sample votes, so 12
  make the barometer implausible.
- A silent barometer in descent: a step without it every period (GPS, else
  section 6's dead reckoning).

`estimator_pass` takes these steps for the thread and the IGT host replay
alike. The replay can now make a sensor silent (`IGT_SILENT`).

**Barometer frozen in ascent.** In ascent a sample that repeats the last one
bit for bit, while the filter moves at 5 m/s or more, is not used (0.2 m a
step, 150 times the BMP390's resolution; at rest a quantised one may
repeat). This removes the false Apogee.

**Re-seat at the barometer verdict.** When a barometer turns implausible in
ascent, the filter re-seats on its last clean step carried forward by the
IMU alone. That undoes whatever its samples did before the verdict. A
barometer that never reaches the verdict (bursts, outliers) is untouched.

Two alternatives were tried and rejected:

- A tighter 50 m gate fixed the wild case, but during an IMU transient it
  locked a good barometer out, giving a false Apogee 8 s early in three
  datasets.
- Anchoring the gate on the barometer failed the same way: it carried the
  transient's velocity.

**IMU verdict.** The IMU is untrusted while frozen (50 bit-identical pairs),
silent, or while the velocity verdict is bad. The velocity verdict needs the
barometer: while that is implausible it keeps its last value, so an IMU
found bad before the barometer failed stays untrusted.

**Nothing trustworthy for Apogee** (barometer implausible and IMU
untrusted): no sensor Apogee; the drogue timer fires it. On the IMU alone,
Apogee is not believed before the last clean barometer state could have
reached it (at 2 g).

**Drogue timer** (`deploy_backup.h`):

- The drogue fires `BACKUP_DROGUE_MS` = 30.0 s after Launch if nothing has
  fired it.
- The flight model (GS `tools/flight_profile`, the 10,000 ft flight, the
  longest planned) reaches apogee 25.9 s after the FC detects Launch.
- The margin, 4.1 s: the model's apogee time is uncertain by about 1 s (Cd
  10 %, impulse 5 %), and the sensor path itself fires 0.8-2.3 s after
  apogee. The latest sensor-path drogue in the whole sweep is 28.16 s after
  Launch.
- While a trustworthy barometer still climbs, the timer waits up to 10 s
  more.
- It is armed only once a measured altitude has reached 30 m
  (`FLYING_ALTITUDE`). A false Launch detection on the pad, after a misfire
  with people approaching, must never fire a chute there. The arming is
  kept across a watchdog restart.
- It runs on every pass, including the wakes without a sample.
- `LaunchClock` is a flight timer, so it survives a watchdog restart.
- `g_fc_backup_drogues` counts its drogues.

**Main timer.** This is section 6's dead-reckoned descent: from a reference
at the drogue, (h_ref - 457 m) / 28.75 m/s after it, early by up to 25 %.

- An IMU-only reference (barometer out in ascent) is capped by the no-drag
  ballistic flight from the last clean barometer step: in coast nothing
  climbs higher or faster.
- When the drogue timer fires, nothing found Apogee, so the reference goes
  back to the last clean barometer step. That is a lower bound while
  climbing, and the prediction then falls freely until the drogue.

### 7.3 Required behaviour and tolerances

As section 6.3, with two more kinds:

- **Drogue by the timer:** at 30.0 s after Launch.
- **Main with no trustworthy reference:** not under 357 m, and after the
  drogue.

Saturated IMU with a wild barometer is the break-up signature (section 2).
By design it aborts before Apogee, with no automatic chutes; the operator's
Deploy/Expand remain.

`tests/test_deploy_backup.py` also covers late drogues. The timer comes
2.9 s after the 10k flight's usual drogue and 10.1 s after the measured
motor's. A real flight falls freely until then, which the open-loop replay
cannot show; the main is still never below 456 m.

### 7.4 Sweep and matrix

`tools/rud_sweep.py` adds 85 cases: 17 failures (two single silent
sensors, the rest two or three sensors), each on all five datasets. Onset is
in coast (22 s) or 10 s after apogee.

Result: **261/261 as expected**:

- The drogue timer fires in none of the 79 must-not-fire cases.
- Those cases keep their deployments: drogue 0.8-2.3 s after apogee, main
  within ±0.25 s of 457 m.

| Failure (onset) | Drogue after the true apogee | Main, true altitude | How |
| --- | --- | --- | --- |
| IMU silent in ascent | +1.2-1.3 s | 454-455 m | barometer alone in ascent, then as usual |
| baro silent in ascent, GPS present | +0.6-1.6 s | 451-461 m | Apogee from the IMU; main on GPS |
| IMU + baro in ascent (silent/silent, frozen/frozen, frozen/wild, silent/wild), GPS present | +4.1 s (10k) .. +11.3 s (measured motor) | 444-461 m | drogue timer; main on GPS |
| IMU saturated + baro wild in ascent | – | – | break-up abort 0.88 s after onset (section 2) |
| IMU + GPS in ascent (silent/absent, frozen/stale) | +1.2-1.4 s | 454-455 m | barometer |
| baro + GPS in ascent (silent/absent, frozen/absent, wild/stale) | −0.6..+0.7 s | 668-913 m | Apogee from the IMU; main dead-reckoned |
| IMU + baro in descent (the IMU is off after Apogee) | +1.3 s | 444-461 m | GPS |
| IMU + GPS in descent | +1.3 s | 454-455 m | barometer |
| baro + GPS in descent (silent/absent, frozen/stale) | +1.3 s | 541-736 m | dead-reckoned |
| all three in ascent (silent, or frozen + absent) | +4.1..+11.3 s | 1273-2244 m | drogue timer; main early from the last clean altitude |

Timer drogue times are against the truth vehicle, whose drogue opened 1.2 s
after apogee. On the measured-motor flight (1.5 km), with all three
failed, both chutes open together, at the timer.

### 7.5 Limits

- **The drogue timer assumes the planned flight.** On the measured motor
  (1,512 m, apogee 18.7 s after Launch) it fires 11.3 s after apogee, after
  about 11 s of free fall (roughly 80-90 m/s). A flight much lower than
  1.2 km could pass 457 m before it. Set `BACKUP_DROGUE_MS` per flight from
  the motor that flies (deploy_backup.h).
- **Pre-existing early Apogee state.** On early_apogee_noise, 0.3 s of IMU
  saturation together with the dataset's false-descent pressure step gives
  an Apogee state 7.7 s early. It is the same on origin/dev. The drogue is
  still on time (+1.3 s): the descent filter waits for the fall on the
  barometer. The sweep reports it as a warning.
- **Barometer offsets** are unobservable without GPS. With GPS present, a
  plausible barometer is still not checked against it (a possible
  follow-up).
- **IMU-only after the barometer fails.** An IMU that also fails without
  being frozen or silent (a bias) cannot be verified. The ballistic cap and
  the timer's reset to the clean barometer step limit the damage: with all
  three failed, both chutes come out early.

Reproduce: `python3 tools/rud_sweep.py <GroundStation26>/test-data/igt`;
`tests/test_deploy_backup.py`, `tests/test_estimator_loop.py`,
`tests/test_sensor_plausibility.py`.

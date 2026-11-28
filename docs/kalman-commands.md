# Operator Kalman filter commands

Three FLIGHT_COMMAND decrees let the operator control the FC's estimator (the
ascent EKF / descent filter and the flight state machine it drives):

| Command | fc_msg | Compat byte | GroundStation |
| --- | --- | --- | --- |
| Kalman: stop | `Kalman_Stop` (`Actionable_Decrees + 22`) | 34 | `KalmanStop`, every mode with an FC |
| Kalman: reset | `Kalman_Reset` (`Actionable_Decrees + 23`) | 35 | `KalmanReset`, every mode with an FC |
| Kalman: start (HITL) | `Kalman_Start` (`Actionable_Decrees + 24`) | 36 | `KalmanStart`, HITL only |

They exist to recover from a false launch detection, where the FC is
evaluating a flight while the vehicle is still on the pad, and to let HITL
start flight mode by hand. None of them reverts the FC or the rocket to
Startup, publishes FLIGHT_STATE, or touches fill, valves, outputs or the
station's global state. They are ignored when sent internally, and refused
like every other command while the sequence is held (manual Stop, monitor
session, commissioning) or a reset to Startup is running.

## Real flights are never stoppable

Stop and Reset are refused while the current flight was admitted by a
station Launch (`fc_recovery_flight_is_admitted()`). Only a network
`Launch_Signal` admits a flight (`fc_recovery_admit_new_flight(false)`), and a
validated watchdog restore of such a flight keeps it admitted; so does a
sensor restart within it. An evaluation the FC started with no station
Launch before it (an internal Launch, or Kalman: start) is never admitted. The check is made again,
under the IRQ guard, when the stop is queued (`fc_recovery_request_kf_stop`).
The GroundStation also refuses both once the station's own Launch was sent.

## Kalman: stop

When the estimator is running (or starting, or paused) and no station Launch
admitted the flight, the recovery owner pauses it at its barrier and stops
its thread (as for a sensor restart, but with no reset, no failure and no
epoch change). A pending sensor restart is cancelled so it cannot start the
evaluator again. Once the thread is quiescent,
`fc_recovery_control_kf_stopped` undoes the evaluation side:

- `Launch_Requested` (and `Graceful_Reset`, `Defer_Baro_Fallback`) cleared,
  `Using_Ascent_KF` set again;
- the FC's own flight state goes back to the pad state the Launch started
  from (Postinit or Armed) if the evaluation had moved it on. Nothing is
  published: the station's global state is untouched;
- the estimator's flight memory is cleared (`estimator_cancel_evaluation`
  and the sensor-failure monitor, as at the reset to Startup): state
  history, extremes, plausibility verdicts, break-up detector, loop state and
  the backup drogue timer, which is disarmed;
- both watchdog flight checkpoints are invalidated, so a later watchdog
  reset cannot resume the stopped evaluation, without the operator Stop seal
  (the next flight can still actuate).

The estimator then stays stopped (`Eval_Abort_Flag`, phase Stopped), so no
automatic deployment can come from it. Kept: deployment history bits, IMU
biases, the barometer reference, and the manual Deploy/Expand and Abort
paths, which do not change. A stop while nothing runs reports that and does
nothing. In a ground-test image the replay is left to the IGT controls.

## Kalman: reset

Clears the estimator's flight memory (the same cancel as above), clears the
stopped flag and sets the estimator phase to Idle, so the next start begins
from scratch. It is refused while the estimator is running ("stop it
first"): the memory belongs to the evaluator thread while it runs, so the
reset never races it. It does not change the FC's flight state.

## Kalman: start

Starts the estimator the way an internal Launch does (`enter_launch(true)`):
from Postinit or Armed, with nothing already running, never admitting a
station flight (so Kalman: stop can end it). It is refused after a reset to
Startup or a Stop (until the station Launch starts the next flight), while
the sequence is held, and outside Postinit/Armed. The FC has no notion of
the station mode: the GroundStation sends it only in HITL and refuses it in
every other mode.

Tests: `tests/test_admitted_flight.py` (`kf_*` cases against the actual
recovery owner and control hooks) and `tests/test_kalman_commands.py`.

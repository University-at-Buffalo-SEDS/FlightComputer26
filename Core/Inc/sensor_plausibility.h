/* Core/Inc/sensor_plausibility.h */

#ifndef FC_SENSOR_PLAUSIBILITY_H
#define FC_SENSOR_PLAUSIBILITY_H

#include <stdbool.h>
#include <stdint.h>

/* One faulty sensor must not cost the chutes (docs/rud-abort.md section 5).
 * The estimator keeps two verdicts, on every step, in every phase:
 *
 *   barometer  implausible once its altitude moves faster than
 *              RUD_PRESSURE_RATE_MPS (450 m/s) on RUD_PRESSURE_MIN (12) of the
 *              last RUD_WINDOW (25) steps, the break-up detector's pressure
 *              vote; plausible again once that count is back to
 *              PLAUS_BARO_CLEAR_MAX (2) or fewer. An intact flight stays under
 *              about 225 m/s and gives at most 5 such steps.
 *   velocity   the ascent filter's velocity is untrusted once it differs from
 *              the trend of its own altitude by more than PLAUS_VEL_TOLER_MPS
 *              (40 m/s) on PLAUS_VEL_STEPS (25) consecutive steps (1 s), and
 *              trusted again after as many agreeing steps. A bad IMU (stuck,
 *              biased, tumbling) drives the velocity while the barometer
 *              keeps the altitude: an intact flight stays within 14 m/s, and
 *              its worst run above 40 m/s is 13 steps.
 *
 * In ascent a barometer sample that itself moved faster than 450 m/s since
 * the last one (an outlier, a broken port) is not used either, nor, while
 * any such step lies in the window, one more than PLAUS_BARO_GATE_M (150 m)
 * from the filter's prediction. A clean barometer is never gated, so it
 * keeps the altitude when the IMU is the faulty sensor.
 *
 * A barometer that repeats the identical altitude for PLAUS_BARO_STUCK_STEPS
 * steps (2 s; a live BMP390 never does in flight) is frozen and counts as
 * implausible until it moves again.
 *
 * With an implausible barometer the filters dead-reckon instead of taking it
 * (ascent: IMU only; descent: GPS altitude while a fix is fresh, else the
 * prediction). With an untrusted ascent velocity the Vigilant check runs on
 * the barometer even when Vigilant mode is off. */

#define PLAUS_BARO_CLEAR_MAX 2U
#define PLAUS_BARO_GATE_M    150.0f
#define PLAUS_BARO_STUCK_STEPS 50U   /* 2 s of bit-identical altitudes */
#define PLAUS_VEL_TOLER_MPS  40.0f
#define PLAUS_VEL_STEPS      25U

typedef struct {
  uint32_t baro_votes;
  uint8_t baro_steps;
  bool have_baro, baro_bad, baro_stuck;
  uint8_t stuck_run;
  float last_alt;
  uint32_t last_t;
  uint8_t vel_bad_run, vel_good_run;
  bool vel_bad;
} plaus_state;

void plaus_reset(plaus_state *p);
/* One barometer sample per estimator step; true if the filter may use it. */
bool plaus_baro(plaus_state *p, uint32_t t_ms, float alt);
bool plaus_baro_ok(const plaus_state *p);
/* Any implausible step in the window. */
bool plaus_baro_suspect(const plaus_state *p);
/* The last sample moved at a plausible rate, with no implausible step in
 * the window and no repeat: the filter output is a trustworthy altitude. */
bool plaus_baro_clean(const plaus_state *p);
/* The ascent filter's velocity against its altitude trend, per step. */
void plaus_velocity(plaus_state *p, float ekf_vel, float trend_vel);
bool plaus_velocity_ok(const plaus_state *p);

#endif /* FC_SENSOR_PLAUSIBILITY_H */

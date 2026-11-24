/* Core/Inc/rud_detect.h */

#ifndef FC_RUD_DETECT_H
#define FC_RUD_DETECT_H

#include <stdbool.h>
#include <stdint.h>

/* Automatic evaluation abort on a break-up of the vehicle (RUD), docs/
 * rud-abort.md. It stops every automatic deployment, so it fires only on
 * readings no intact flight produces, held over time and confirmed by an
 * independent sensor; never on one sensor alone, a transient, a dropout or a
 * deployment or landing shock. Armed from Launch until Apogee only.
 *
 * Evaluated on every estimator step (one per barometer sample, 25 Hz), over
 * the last RUD_WINDOW steps (about 1 s):
 *   motion   the IMU (BMI088: gyroscope and accelerometer share a package,
 *            so they count as one sensor) reads tumbling, |gyro x,y| >= 360
 *            deg/s (pitch/yaw; z is the long axis), or a lateral load,
 *            |accel x,y| >= 49 m/s^2 (5 g), on at least RUD_MOTION_MIN of
 *            the steps. An intact flight stays near 21 deg/s and 7 m/s^2.
 *   pressure the barometer altitude moves faster than 450 m/s between
 *            consecutive samples on at least RUD_PRESSURE_MIN of the steps.
 *            An intact flight stays under about 225 m/s; a single outlier
 *            or a pressure step makes two such steps.
 * It fires when both hold at once. Either alone never fires. */

#define RUD_TUMBLE_DPS        360.0f
#define RUD_LATERAL_MPS2      49.0f
#define RUD_PRESSURE_RATE_MPS 450.0f
#define RUD_WINDOW            25U   /* 1 s at 25 Hz */
#define RUD_MOTION_MIN        22U   /* 0.88 s of it */
#define RUD_PRESSURE_MIN      12U   /* about half of it */

typedef struct {
  uint32_t t_ms;
  bool armed;           /* Launch, Ascent or Coast */
  float gyro[3];        /* deg/s */
  float accel[3];       /* m/s^2 */
  float baro_alt;       /* m above the pad */
} rud_input;

typedef struct {
  uint32_t motion, pressure; /* one bit per step, newest in bit 0 */
  uint8_t steps;             /* steps seen since arming, up to RUD_WINDOW */
  bool have_baro;
  float last_alt;
  uint32_t last_t;
  bool fired;
} rud_state;

void rud_reset(rud_state *s);
/* One estimator step. True exactly once, on the step that fires. */
bool rud_step(rud_state *s, const rud_input *in);

#endif /* FC_RUD_DETECT_H */

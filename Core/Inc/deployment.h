/*
 * Deployment logic header and confirguration file.
 */

#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

/* Local configuration */

#define DEPL_BUF_SIZE 4

#define MIN_SAMP_ASCENT   6
#define MIN_SAMP_BURNOUT  6
#define MIN_SAMP_DESCENT  6
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   12

#define RECOVERY_INTERVAL 4
#define PRE_RECOV_RETRIES 30
#define PRE_ABORT_RETRIES 40

#define CONSECUTIVE_CONFIRMS
/* Delay in ThreadX ticks */
#define LAUNCH_CONFIRM_DELAY 25
#define APOGEE_CONFIRM_DELAY 75

/* If deployment aborts, delay (in ticks) between
 * engine shutdown and CO2/REEF deployments */
#define ABORT_CO2_DELAY   150
#define ABORT_REEF_DELAY  150

/* Measurement thresholds.
 * Units: altitude ALT (m), velocity VEL (m/s)
 * vertical acceleration VAX (m/s^2) */

#define SANITY_MAX_ALT 4800.0f
#define SANITY_MAX_VEL 200.0f
#define SANITY_MAX_VAX (GRAVITY_SI * 12.0f)

#define SANITY_MIN_ALT -10.0f
#define SANITY_MIN_VEL -6.0f
#define SANITY_MIN_VAX -4.0f

#define LAUNCH_MIN_VEL  8.0f
#define LAUNCH_MIN_VAX  5.0f

#define BURNOUT_MIN_VEL 12.0f
#define BURNOUT_MAX_VAX 1.5f

#define APOGEE_MAX_VEL  4.0f

#define REEF_TARGET_ALT 457.2f

#define ALT_TOLER  2.0f
#define VEL_TOLER  1.5f
#define VAX_TOLER  1.0f

/* FC '26 GPIO port maps */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6

/* Service definitions */

#define GRAVITY_SI 9.80665f

#define DEPL_CODE_MASK (DEPL_BUF_SIZE + 1)

/* Type definitions */

/*
 * Service enum used to report inference result.
 * DEPL_CODE_MASK is used to adjust bad data reporting
 * range based on a reasonable local buffer size.
 */
typedef enum {
  /* Data validation error codes (additive, critical) */
  DEPL_BAD_ALT    = -(DEPL_CODE_MASK)*(DEPL_CODE_MASK),
  DEPL_BAD_VEL    = -(DEPL_CODE_MASK),
  DEPL_BAD_VAX    = -1,

  /* Generic OK and a reference point */
  DEPL_OK         = 0,

  /* Filter had no data (non-critical) */
  DEPL_NO_INPUT   = 1,
  
  /* Recovery error codes (additive, critical) */
  DEPL_BAD_ACCEL  = 2,
  DEPL_BAD_GYRO   = 3,
  DEPL_BAD_BARO   = 4,

  /* Unconfirmed state transitions (non-critical) */
  DEPL_N_LAUNCH   = 16,
  DEPL_N_BURNOUT  = 17,
  DEPL_N_DESCENT  = 18,
  DEPL_N_REEF     = 19,
  DEPL_N_LANDED   = 20,

  /* Assert unreachable */
  DEPL_DOOM       = 127
} inference_e;

/*
 * The backbone of state machine and error handling.
 */
typedef enum {
  INIT,
  IDLE,
  LAUNCH,
  ASCENT,
  BURNOUT,
  APOGEE,
  DESCENT,
  REEF,
  LANDED,
  RECOVERY,
  ABORTED,
} state_e;

/*
 * Minimum required stats to have some variety
 * of checks we use to make a decision. Calculated
 * for current buffer, useful for two iterations.
 */
typedef struct {
  float min_alt;
  float max_alt;
  float avg_vel;
  float avg_vax;
} stats_t;

/*
 * This struct unifies rocket state, the
 * amount of successive detections of last
 * concerned state (one explicit int),
 * a toolkit of various indices (see below),
 * and an error recovery toolkit.
 */
typedef struct {
  state_e state;
  union {
    uint_fast8_t ascent;
    uint_fast8_t burnout;
    uint_fast8_t descent;
    uint_fast8_t landing;
    uint_fast8_t idle;
  } samp;
  struct {
    uint_fast8_t ex;  /* Position in external filter ring   */
    uint_fast8_t sc;  /* # used elements in "current" buf   */
    uint_fast8_t sp;  /* # used elements in "previous" buf  */
    uint_fast8_t a;   /* Index of "current" buffer (0 or 1) */
  } i;
  struct {
    state_e state;
    inference_e inf;
    uint_fast8_t ret;
  } rec;
} rocket_t;

#endif // DEPLOYMENT_H
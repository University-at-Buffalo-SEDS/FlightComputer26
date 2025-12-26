/*
 * Deployment logic header and confirguration file.
 */

#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

/* Local configuration */

#define MAX_SAMPLE 4

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

#define SN_MAX_ALT 4800.0f
#define SN_MAX_VEL 200.0f
#define SN_MAX_VAX (GRAVITY_SI * 12.0f)

#define SN_MIN_ALT -10.0f
#define SN_MIN_VEL -6.0f
#define SN_MIN_VAX -4.0f

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

/* Data validation and statistics  */

#define DATA_MASK (MAX_SAMPLE + 1)

#define VALID_ALT 0x01u
#define VALID_VEL (1u << 2)
#define VALID_VAX (1u << 4)

/* Base offset for reporting bad data */
#define DATA_OFFSET -10

/* Type definitions */

/*
 * Service enum used to report inference result.
 * DEPL_CODE_MASK is used to adjust bad data reporting
 * range based on a reasonable local buffer size.
 */
typedef enum {
  /* Bad data codes (additive, critical)
   * Ranges: [-134, -11] (error), [11, 134] (warning) 
   * when used with DATA_OFFSET */
  DATA_BAD_ALT    = -(DATA_MASK)*(DATA_MASK),
  DATA_BAD_VEL    = -(DATA_MASK),
  DATA_BAD_VAX    = -1,

  /* Recovery error codes (additive, critical) */
  DEPL_BAD_ACCEL  = -4,
  DEPL_BAD_GYRO   = -3,
  DEPL_BAD_BARO   = -2,

  /* Generic OK and a reference point */
  DEPL_OK         = 0,

  /* Unconfirmed state transitions (non-critical) */
  DEPL_N_LAUNCH   = 1,
  DEPL_N_BURNOUT  = 2,
  DEPL_N_DESCENT  = 3,
  DEPL_N_REEF     = 4,
  DEPL_N_LANDED   = 5,

  /* UKF ring has no new entries (non-critical) */
  DATA_NONE       = 8,

  /* For abortion and unreachable statements */
  DEPL_DOOM       = 9,
} inference_e;

_Static_assert(sizeof(inference_e) > sizeof(int8_t), "enum capacity");

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
    inference_e warn;
    uint_fast8_t ret;
    uint_fast8_t lock;
  } rec;
} rocket_t;

/* Public helpers */

/*
 * Public helper. Invoked from thread context.
 */
state_e get_rocket_state();

/*
 * Triggers forced parachute firing and expansion with
 * compile-time specified intervals. USE WITH CAUTION.
 * Invoked from thread context.
 */
void force_abort_deployment();

#endif // DEPLOYMENT_H
/*
 * Deployment logic header and confirguration file.
 */

#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <stddef.h>
#include <stdint.h>

#ifdef PL_HOST
#include "../../tests/platform.h"
#else
#include "platform.h"
#endif

#include "ukf.h"


/* ------ Local configuration ------ */

#define DATA_CAP 4

#define MIN_SAMP_ASCENT   6
#define MIN_SAMP_BURNOUT  6
#define MIN_SAMP_DESCENT  6
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   12

#define RECOVERY_INTERVAL 4
#define PRE_RECOV_RETRIES 20
#define PRE_ABORT_RETRIES 20

#define CONSECUTIVE_CONFIRMS
/* Delay in ThreadX ticks */
#define LAUNCH_CONFIRM_DELAY 25
#define APOGEE_CONFIRM_DELAY 75

/* Measurement thresholds.
 * Units: altitude ALT (m), velocity VEL (m/s)
 * vertical acceleration VAX (m/s^2) */

#define MAX_ALT 4800.0f
#define MAX_VEL 200.0f
#define MAX_VAX (GRAVITY_SI * 12.0f)

#define MIN_ALT -10.0f
#define MIN_VEL -6.0f
#define MIN_VAX -4.0f

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

#define DATA_MASK (DATA_CAP + 1)

#define VALID_ALT 0x01u
#define VALID_VEL (1u << 2)
#define VALID_VAX (1u << 4)

#define VALID_STATS (VALID_ALT | VALID_VEL | VALID_VAX)

/* Base offset for reporting bad data */
#define DATA_OFFSET -10

/* Cycle and abortion defines */

#define MAX_CYCLES (DEPLOYMENT_THREAD_RETRIES \
                    + DEPLOYMENT_THREAD_INPUT)

#define AUTO_ABORT    0x01u
#define MANUAL_ABORT  (1u << 2)
#define PYRO_FIRED    (1u << 4)

#define SEDS_ARE_COOL 1


/* ------ Type definitions ------ */

/// Service enum used to report inference result.
/// DATA_MASK is used to adjust bad data reporting
/// range based on a reasonable local buffer size.
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

  /* For unreachable statements */
  DEPL_DOOM       = 9,
} inference_e;

/* Do not allow inference_e to be < 16 bits due to increment */
_Static_assert(sizeof(inference_e) > sizeof(int8_t), "enum capacity");

/// In-flight rocket states only since used internally.
typedef enum {
  IDLE,
  LAUNCH,
  ASCENT,
  BURNOUT,
  APOGEE,
  DESCENT,
  REEF,
  LANDED,
} state_e;

/// Commands for manual control.
typedef enum {
  ABORT,
  FIRE_PYRO,
  FIRE_REEF,
  SHUTDOWN
} command_e;

/// Minimum required stats to have some variety of checks
/// we use to make a decision. Useful for two iterations.
typedef struct {
  float min_alt;
  float max_alt;
  float avg_vel;
  float avg_vax;
} stats_t;

/// Local struct used when fetching data from UKF ring.
/// Conveniently passed as argument.
typedef struct {
  inference_e st;
  uint_fast8_t mask;
} fetch_t;

/// Flight statistics struct.
/// These variables are used most often and
/// should to loaded to D-Cache together.
typedef struct {
  state_e state;
  uint_fast8_t a;
  uint_fast8_t samples;
} flight_t;

/// Critical communication struct.
typedef struct {
  command_e cmd;      /* Last command in manual mode       */
  inference_e warn;   /* Stash for non-critical code       */
  uint_fast8_t retry; /* Per cycle, reset on success       */
  uint_fast8_t abort; /* Mask for auto and manual triggers */
} monitor_t;


/* ------ Public API ------ */

/// Does not cover states before rocket is ready to launch.
state_e current_flight_state();

/// Send enum-specified command in manual mode.
/// First command should be 'ABORT' to enter manual mode.
void deployment_send_command(command_e command);

#endif // DEPLOYMENT_H
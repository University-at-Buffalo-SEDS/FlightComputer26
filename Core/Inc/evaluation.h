/*
 * Data evaluation header and API.
 */

#ifndef EVAL_H
#define EVAL_H

#include <stdint.h>

#include "dma.h"

#ifdef PL_HOST
#include "../../tests/platform.h"
#else
#include "platform.h"
#endif


/* ------ Local configuration ------ */

#define MIN_SAMP_ASCENT   6
#define MIN_SAMP_BURNOUT  6
#define MIN_SAMP_DESCENT  6
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   12

/* Delay in ThreadX ticks */
#define LAUNCH_CONFIRM_DELAY 25
#define APOGEE_CONFIRM_DELAY 75

/* Measurement thresholds.
 * Units: altitude ALT (m), angular velocity ANG (deg/s)
 * vertical acceleration VAX (m/s^2) */

#define MAX_ALT 4800.0f
#define MAX_ANG 2000.0f
#define MAX_VAX (GRAVITY_SI * 12.0f)

#define MIN_ALT -10.0f
#define MIN_ANG -MAX_ANG
#define MIN_VAX -(GRAVITY_SI * 12.0f)

#define LAUNCH_MIN_VEL  8.0f
#define LAUNCH_MIN_VAX  5.0f

#define BURNOUT_MIN_VEL 12.0f
#define BURNOUT_MAX_VAX 1.5f

#define APOGEE_MAX_VEL  4.0f

#define REEF_TARGET_ALT 457.2f

#define ALT_TOLER  2.0f
#define VEL_TOLER  1.5f
#define VAX_TOLER  1.0f


/* ------ Data validation ------  */

#define VALID_ALT 0x01u
#define VALID_VEL (1u << 2)
#define VALID_VAX (1u << 4)

#define VALID_DATA (VALID_ALT | VALID_VEL | VALID_VAX)


/* ------ UKF data defs ------ */

#define GRAVITY_SI 9.80665f
#define TOLERANCE 1e-3f
#define TLOWER_1 (1.0f - TOLERANCE)
#define TUPPER_1 (1.0f + TOLERANCE)

#define RING_SIZE 8
#define RING_MASK (RING_SIZE - 1)

#define FSEC(ms) ((float)(ms) * 0.001f)

#define NR_ITERATIONS 1


/* ------ UKF Containers ------ */

union bithack {
  float f;
  uint32_t d;
};

struct quaternion {
  float q1, q2, q3, q4;
};

struct state_vec {
  struct coords p, v, a, w;
  struct quaternion qv;
};


/* ------ Data evaluation containers ------ */

/// In-flight rocket states only since used internally.
enum state {
  IDLE,
  LAUNCH,
  ASCENT,
  BURNOUT,
  APOGEE,
  DESCENT,
  REEF,
  LANDED,
};

/// Averaged representation of a single
/// UKF evaluation that is easier to decide on.
struct stats {
  float min_alt, max_alt, avg_vel, avg_vax;
};

/// Replica of several global config paramters
/// but as a locally accessed bitfield
struct op_mode {
  unsigned safe_expand_reef : 1;
  unsigned pyro_req_confirm : 1;
  unsigned consecutive_samp : 1;
};


/* ------ Public API ------ */

/// Enqueues raw data set for processing by UKF.
void evaluation_put(const struct measurement *buf);

/// Aggregates sensor compensation functions.
/// Run before reporting and publishing data.
void compensate(struct measurement *buf);


#endif // EVAL_H
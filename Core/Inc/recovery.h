/*
 * Recovery enum definition.
 * Values are passed through TX queue.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#include "predict.h"

/// Used to adjust bad data reporting
/// range based on a reasonable buffer size.
#define DATA_MASK (DATA_CAP + 1)

/// Command or warning/error code for recovery.
typedef enum {
  /* Bad sensor measurement codes (critical) */
  RAW_BAD_ALT   = -5,
  RAW_BAD_ANG_Z = -4,
  RAW_BAD_ANG_Y = -3,
  RAW_BAD_ANG_X = -2,
  RAW_BAD_VAX   = -1,

  /* Reference point */
  CMD_NONE = 0,
  
  /* Commands */
  FIRE_PYRO = 1,
  FIRE_REEF = 2,
  RECOVER   = 3,
  
  /* Unconfirmed state transitions (non-critical) */
  NOT_LAUNCH  = 11,
  NOT_BURNOUT = 12,
  NOT_DESCENT = 13,
  NOT_REEF    = 14,
  NOT_LANDED  = 15,
} cmd_e;

// Not needed if we validate one sample per UKF iteration
// #define typeeq(a, b) __builtin_types_compatible_p(a, b)

// /// Since we stack error codes, require int for cmd_e.
// _Static_assert(typeeq(typeof(cmd_e), typeof(int)), "");

#endif // RECOVERY_H
/*
 * Public API for the parachute deployment logic and thread.
 */

#pragma once
#include <stdint.h>

/* Threshold configuration */

#define MIN_BURNOUT 4
#define MIN_DESCENT 4
#define MIN_REEF    6
#define MIN_LANDED  4

/* Type definitions */

typedef enum {
  IDLE,
  LAUNCH,
  ASCENT,
  BURNOUT,
  APOGEE,
  DESCENT,
  REEF,
  LANDED
} state_e;

// Needs clock
typedef union {
  uint32_t launch_ms;
  uint32_t burnout_ms;
  uint32_t apogee_ms;
  uint32_t pyro_ms;
  uint32_t reef_ms;
  uint32_t landed_ms;
} time_ex;

typedef union {
  uint32_t burnout;
  uint32_t descent;
  uint32_t reef;
  uint32_t landed;
} samples_ex;

typedef struct {
  state_e state;
  time_ex time_of;
  samples_ex samples_of;
  uint32_t ring_index;
  uint32_t apogee_height;
} rocket_t;
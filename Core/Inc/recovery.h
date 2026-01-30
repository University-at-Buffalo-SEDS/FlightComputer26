/*
 * Recovery enum definition and config.
 * Values are passed through TX queue.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"

/// Header declaration of the recovery queue
extern TX_QUEUE shared;

/// Run time configuration mask
extern atomic_uint_least32_t config;


/* ------ User configuration ------ */

#define FAILS_TO_REINIT 10
#define FAILS_TO_ABORT  40

#define ACCUMULATE_FAILURES 0


/* ------ Endpoint identifiers: FC, GND ------ */

/// 2s complement awareness: as negative codes
/// can only originate from the FC, checking
/// against the incoming message MSB is reliable.
#define FC_MASK (1u << 31)

/// Mark general commands as coming from FC.
/// Use this for enum command values > 0.
#define FC_MSG(message) (message | FC_MASK)


/* ------ Endpoint timeouts ------ */

/* Mirrors Ground Station timeouts */
#define FC_TIMEOUT_MS 4000
#define GND_TIMEOUT_MS 4000

/* Expiration of timer that invokes timeout checks 
 * (1 tick = 10 ms) */
#define TX_TIMER_TICKS   100
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)


/* ------ Run time configuration flags ------ */

#if defined(__GNUC__) || __STDC_VERSION__ >= 202311L
/*
 * In the version of C we are using, this is a GNU extension.
 */
enum g_conf : uint_least32_t {

#else
enum g_conf {

#endif

  CHECKS_COMPLETE = 0, // Do not modify

  /* Options (currently max 32) */
  FORCE_ALT_CHECKS = 1u,
  ACCUMULATE_FAILS = 1u << 1,
  DISTRIB_EMERGENT = 1u << 2,
  SAFE_EXPAND_REEF = 1u << 3,
  REINIT_ATTEMPTED = 1u << 4,
  CONSECUTIVE_SAMP = 1u << 5,
};


/* ------ Recovery commands ------ */

/// Command or warning/error code for recovery.
/// Guideline for including your custom messages:
/// 
/// 1. Mark your section with a variant of appropriate
/// label. Put that variant as the next-largest power
/// of 2. Then add your variants by incrementing that mark.
///
/// 2. In either FC or GND sections of recovery.c decode(),
/// add a check against your section mark as a top 'if'
/// statement (so it executes before other checks).
///
/// 3. Inside this check, call your function to handle
/// your added variants.
/// 
/// You can send a command to recovery like this
/// (assume firing pyro from the ground station):
///
/// enum command cmd = FIRE_PYRO;
/// tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
///
/// Wait option depends on whether you want to drop
/// the value if queue is full (unlikely). 
///
/// NOTE: if you are sending a message from FC,
/// mask it with FC_MSG(_variant_). Otherwise - UB!

#if defined(__GNUC__) || __STDC_VERSION__ >= 202311L
enum command : uint32_t {

#else
enum command {

#endif

  /* Raw data codes (additive) */
  RAW_DATA = 0,

  RAW_BAD_ALT   = 1u,
  RAW_BAD_ANG_Z = (1u << 1),
  RAW_BAD_ANG_Y = (1u << 2),
  RAW_BAD_ANG_X = (1u << 3),
  RAW_BAD_VAX   = (1u << 4),  

  /* Data evaluation codes */
  DATA_EVALUATION = (1u << 5),

  NOT_LAUNCH  = DATA_EVALUATION + 1,
  NOT_BURNOUT = DATA_EVALUATION + 2,
  NOT_DESCENT = DATA_EVALUATION + 3,
  NOT_REEF    = DATA_EVALUATION + 4,
  NOT_LANDED  = DATA_EVALUATION + 5,
  
  /* Actionanle commands */
  ACTION = (1u << 6),

  FIRE_PYRO = ACTION + 1,
  FIRE_REEF = ACTION + 2,
  RECOVER   = ACTION + 3,

  /* ... */

  /* Synchronization event
   * The same code for both FC and GND
   * because masking is in effect */
  SYNC = (1u << 30),
};


/* For non-GNU C < 23, prevent UB at compile-time */

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L
/*
 * Positive 32-bit value required (TX queue and recovery semantics).
 */
#define typeeq(a, b) __builtin_types_compatible_p(a, b)

_Static_assert(typeeq(typeof(enum command), typeof(uint32_t)), "");
_Static_assert(typeeq(typeof(enum g_conf),  typeof(uint32_t)), "");

#endif


/* ------ User run time configuration ------ */

/// Run time config applied on boot.
/// Call this function once (before creating tasks).
/// Users are welcome to edit this function.
void user_runtime_config();


#endif // RECOVERY_H
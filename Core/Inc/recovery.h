/*
 * Recovery configuration and API.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#include "platform.h"

/// Header declaration of the recovery queue
extern TX_QUEUE shared;

/// Run time configuration mask
extern atomic_uint_fast32_t config;


/* ------ Thresholds for bad/delayed/outdated data reports  ------ */

#define TO_REINIT 10
#define TO_ABORT  40

#define MAX_RESTARTS 3

#define GPS_DELAY_MS 125
#define MAX_GPS_DELAYS 16
#define GPS_TIME_DRIFT_MS 40
#define GPS_MAX_MALFORMED 15

/* ------ Endpoint identifiers: FC, GND ------ */

/// 2s complement awareness: as negative codes
/// can only originate from the FC, checking
/// against the incoming message MSB is reliable.
#define FC_MASK (1u << 31)

/// Mark general commands as coming from FC.
/// Use this for enum command values > 0.
#define FC_MSG(message) (message | FC_MASK)


/* ------ Endpoint timeouts ------ */

#define FC_TIMEOUT_MS 4000
#define RF_TIMEOUT_MS 2000
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
enum g_conf : uint32_t {

#else
enum g_conf {

#endif // GNU C + C23

  CHECKS_COMPLETE = 0, /* Reserved */

  /* Evaluation options */
  FORCE_ALT_CHECKS = 1u,
  CONSECUTIVE_SAMP = 1u << 1,
  SAFE_EXPAND_REEF = 1u << 2,
  REINIT_ATTEMPTED = 1u << 3,
  PYRO_REQ_CONFIRM = 1u << 4,

  RENORM_QUATERN_1 = 1u << 5, /* Default */
  RENORM_QUATERN_2 = 1u << 6,
  RENORM_QUATERN_4 = 1u << 7,
  RENORM_QUATERN_8 = 1u << 8,
  ABORT_EVALUATION = 1u << 9,
  EVAL_PREEMPT_OFF = 1u << 10,  
  FORCE_ASCENT_UKF = 1u << 11,

  /* Distribution options */
  DISTRIB_EMERGENT = 1u << 12,
  ENTER_DIST_CYCLE = 1u << 13,

  /* Recovery options */
  RESET_FAILURES = 1u << 14,
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
/// (assume trying to recover sensors from the FC):
///
/// enum command cmd = FC_MSG(FIRE_PYRO);
/// tx_queue_send(&shared, &cmd, TX_NO_WAIT);
///
/// Wait option depends on whether you want to drop
/// the value if the queue is full (latter is unlikely). 
///
/// NOTE: if you are sending a message from FC,
/// mask it with FC_MSG(_variant_). Otherwise - UB!

#if defined(__GNUC__) || __STDC_VERSION__ >= 202311L
enum command : uint32_t {

#else
enum command {

#endif // GNU C + C23

  /* Raw data codes (additive) */
  RAW_DATA = 0,

  RAW_BAD_ALT   = 1u,
  RAW_BAD_ANG_X = (1u << 1),
  RAW_BAD_ANG_Y = (1u << 2),
  RAW_BAD_ANG_Z = (1u << 3),
  RAW_BAD_ACC_X = (1u << 4),  
  RAW_BAD_ACC_Y = (1u << 5),
  RAW_BAD_ACC_Z = (1u << 6),
  RAW_BAD_GPS_X = (1u << 7),
  RAW_BAD_GPS_Y = (1u << 8),
  RAW_BAD_GPS_Z = (1u << 9),

  /* Data evaluation codes */
  DATA_EVALUATION = (1u << 10),

  NOT_LAUNCH  = DATA_EVALUATION + 1,
  NOT_BURNOUT = DATA_EVALUATION + 2,
  NOT_DESCENT = DATA_EVALUATION + 3,
  NOT_REEF    = DATA_EVALUATION + 4,
  NOT_LANDED  = DATA_EVALUATION + 5,
  
  /* Actionanle commands */
  ACTION = (1u << 11),

  FIRE_PYRO   = ACTION + 1,
  FIRE_REEF   = ACTION + 2,
  RECOVER     = ACTION + 3, 
  START       = ACTION + 4,
  EVAL_RELAX  = ACTION + 5,
  EVAL_FOCUS  = ACTION + 6,
  EVAL_ABORT  = ACTION + 7,
  ALT_CHECKS  = ACTION + 8,
  ACCUM_FAILS = ACTION + 9,
  USE_ASCENT  = ACTION + 10,

  /* Run time Bounds for abort */
  AUTO_ABORT_BOUNDS = (1u << 12),

  ABORT_10 = AUTO_ABORT_BOUNDS + 10,
  ABORT_20 = AUTO_ABORT_BOUNDS + 25,
  ABORT_50 = AUTO_ABORT_BOUNDS + 50,

  /* Run time Bounds for reinit */
  AUTO_REINIT_BOUNDS = (1u << 13),

  REINIT_5  = AUTO_REINIT_BOUNDS + 5,
  REINIT_12 = AUTO_REINIT_BOUNDS + 12,
  REINIT_20 = AUTO_REINIT_BOUNDS + 20,

  /* Run time KF config options */
  KF_OP_MODE = (1u << 14),

  CMD_RENORM_QUATERN_1 = KF_OP_MODE + 1,
  CMD_RENORM_QUATERN_2 = KF_OP_MODE + 2,
  CMD_RENORM_QUATERN_4 = KF_OP_MODE + 4,
  CMD_RENORM_QUATERN_8 = KF_OP_MODE + 8,

  /* GPS data delivery codes */
  GPS_DELIVERY = (1u << 15),

  GPS_DELAY     = GPS_DELIVERY + 1,
  GPS_MALFORMED = GPS_DELIVERY + 2,

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

#endif // !GNU C * < C23


/* ------ User default configuration ------ */

/// Run time config options applied on boot.
/// Users are welcome to edit the defaults here.
#define DEFAULT_OPTIONS ( (fu32) (0               \
                          | CONSECUTIVE_SAMP      \
                          | RENORM_QUATERN_1      \
                          | EVAL_PREEMPT_OFF      \
                        ) )


#endif // RECOVERY_H
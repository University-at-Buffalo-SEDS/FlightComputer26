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

#define TO_REINIT 20
#define TO_ABORT  40
#define RENORM_STEP 0

#define MAX_RESTARTS 3

#define GPS_DELAY_MS 125
#define MAX_GPS_DELAYS 16
#define GPS_TIME_DRIFT_MS 40
#define GPS_MAX_MALFORMED 15


/* ------ Endpoint timeouts ------ */

#define FC_TIMEOUT_MS 4000
#define GND_TIMEOUT_MS 4000

/* Expiration of timer that invokes timeout checks  */
#define TX_TIMER_TICKS   100
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)



/* ------ Universal Flight Computer message ------ */

/* This message format is used by the Flight Computer
 * for internal and external communication.
 *
 * The general format can be summarized as follows:
 *
 * The MSB represents an internal FC message. If it
 * is unset, the message came from external source.
 * The next bit represents Ground Station heartbeat.
 *
 * The next set bit (can be any) represents message
 * category, such as an Action (command) or sensor
 * measurement code. Run time configuration category
 * has its own subcategories, which overlap with other
 * categories. This is acceptable because configuration
 * category is matched first and then branched away.
 *
 * After category mask is removed, the subcategory
 * variants represent directly useful values except
 * for when literal value is not important (in this
 * case values are 1..N). The lowest category is
 * additive, and therefore follows exponential pattern.
 */

#if defined(__GNUC__) || __STDC_VERSION__ >= 202311L
enum message : uint32_t {

#else
enum message {

#endif // GNU C + C23

  Sensor_Measm_Code = 0,

  Bad_Altitude   = 1u,
  Bad_Attitude_X = (1u << 1),
  Bad_Attitude_Y = (1u << 2),
  Bad_Attitude_Z = (1u << 3),
  Bad_Accel_X    = (1u << 4),  
  Bad_Accel_Y    = (1u << 5),
  Bad_Accel_Z    = (1u << 6),
  Bad_Lattitude  = (1u << 7),
  Bad_Longtitude = (1u << 8),
  Bad_Sea_Level  = (1u << 9),

  Spurious_Confirmations = (1u << 10),

  Not_Launch  = Spurious_Confirmations + 1,
  Not_Burnout = Spurious_Confirmations + 2,
  Not_Descent = Spurious_Confirmations + 3,
  Not_Reefing = Spurious_Confirmations + 4,
  Not_Landed  = Spurious_Confirmations + 5,
  
  Actionable_Decrees = (1u << 11),

  Deploy_Parachute = Actionable_Decrees + 1,
  Expand_Parachute = Actionable_Decrees + 2,
  Reinit_Sensors   = Actionable_Decrees + 3, 
  Launch_Signal    = Actionable_Decrees + 4,
  Evaluation_Relax = Actionable_Decrees + 5,
  Evaluation_Focus = Actionable_Decrees + 6,
  Evaluation_Abort = Actionable_Decrees + 7,
  Reinit_Barometer = Actionable_Decrees + 8,

  GPS_Packet_Code = (1u << 12),

  GPS_Delayed   = GPS_Packet_Code + 1,
  GPS_Malformed = GPS_Packet_Code + 2,

  /* ... */

  Runtime_Configuration = (1u << 29),

  Monitor_Altitude    = Runtime_Configuration | 1u,
  GPS_Available       = Runtime_Configuration | (1u << 1),
  Consecutive_Samples = Runtime_Configuration | (1u << 2),
  Parachute_Deployed  = Runtime_Configuration | (1u << 3),
  Reinit_Attempted    = Runtime_Configuration | (1u << 4),
  Confirm_Altitude    = Runtime_Configuration | (1u << 5),
  Eval_Focus_Flag     = Runtime_Configuration | (1u << 6),
  Eval_Abort_Flag     = Runtime_Configuration | (1u << 7),
  Reset_Failures      = Runtime_Configuration | (1u << 8),
  Launch_Triggered    = Runtime_Configuration | (1u << 8),
  Validate_Measms     = Runtime_Configuration | (1u << 9),
  Using_Ascent_KF     = Runtime_Configuration | (1u << 11),
  In_Aborted_State    = Runtime_Configuration | (1u << 12),
  Lost_GroundStation  = Runtime_Configuration | (1u << 13),

  Revoke_Option = Runtime_Configuration | (1u << 20),

  KF_Operation_Mode = Runtime_Configuration | (1u << 28),

  Renormalize_Quat_1 = KF_Operation_Mode + 0,
  Renormalize_Quat_2 = KF_Operation_Mode + 1,
  Renormalize_Quat_4 = KF_Operation_Mode + 3,
  Renormalize_Quat_8 = KF_Operation_Mode + 7,

  Abortion_Thresholds = Runtime_Configuration | (1u << 27),

  Abort_After_15 = Abortion_Thresholds + 15,
  Abort_After_40 = Abortion_Thresholds + 40,
  Abort_After_70 = Abortion_Thresholds + 70,

  Reinit_Thresholds = Runtime_Configuration | (1u << 26),

  Reinit_After_12 = Reinit_Thresholds + 12,
  Reinit_After_26 = Reinit_Thresholds + 26,
  Reinit_After_44 = Reinit_Thresholds + 44,

  /* ... */

  GroundStation_Heartbeat = (1u << 30),
  FC_Identifier = (1u << 31),

  Invalid_Message = UINT32_MAX
};


/* For non-GNU C < 23, prevent UB at compile-time */

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L

#define typeeq(a, b) __builtin_types_compatible_p(a, b)

_Static_assert(typeeq(typeof(enum command), typeof(uint32_t)), "");
_Static_assert(typeeq(typeof(enum g_conf),  typeof(uint32_t)), "");

#endif // !GNU C * < C23


/* ------ Endpoint identifiers: FC ------ */

#define fc_mask(message)    ((message) | FC_Identifier)
#define fc_unmask(message)  ((message) & ~FC_Identifier)


/* ------ Statically unflag option value ------ */

/* When sending commands TO decode_message() */
#define revoke(opt) ((opt) | Revoke_Option)

/* When manipulating config OUTSIDE decode_message() */
#define option(opt) ((opt) & ~Runtime_Configuration)


/* ------ User default configuration ------ */

/// Run time config options applied on boot.
/// Users are welcome to edit the defaults here.
#define DEFAULT_OPTIONS ( (fu32) (0                   \
                          | Consecutive_Samples       \
                          | Renormalize_Quat_1        \
                          | Eval_Focus_Flag           \
                          | Reset_Failures            \
                          | Validate_Measms           \
                          | GPS_Available             \
                          | Using_Ascent_KF           \
                        ) )


/* ------ On-board relative timer implementation ------ */

enum fc_timer {
  AscentKF,
  DescentKF,
  HeartbeatFC,
  HeartbeatRF,
  HeartbeatGND,
  IntervalGPS,

  Time_Users
};

/// Last recorded time for each UKF timer user.
/// Defined in recovery.c to avoid multiple linkage.
/// u32 wrap is not handled (flight assumed < 49 days :D).
extern volatile fu32 local_time[Time_Users];


/// Report time elapsed since last call to either 
/// timer_fetch_update or timer_update,
/// and set local time to current HAL tick (ms).
static inline fu32 timer_exchange(enum fc_timer u)
{
  fu32 prev = local_time[u];
  local_time[u] = now_ms();
  return local_time[u] - prev;
}


/// Set local time to current HAL tick (ms).
static inline void timer_update(enum fc_timer u)
{
  local_time[u] = now_ms();
}


/// Report time elapsed since last call to either 
/// timer_fetch_update or timer_update.
static inline fu32 timer_fetch(enum fc_timer u)
{
  return now_ms() - local_time[u];
}


#endif // RECOVERY_H
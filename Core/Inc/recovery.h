/*
 * Recovery configuration and API.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#include "platform.h"
#include "evaluation.h"

extern TX_QUEUE shared;
extern atomic_uint_fast32_t config;


/* ------ Thresholds for data reports  ------ */

#define TO_REINIT 20
#define TO_ABORT  40
#define RENORM_STEP 0

#define GPS_DELAY_MS 125
#define GPS_TIME_DRIFT_MS 40

#define GPS_MAX_DELAYS 40
#define GPS_MAX_MALFORMED 30

#define GPS_SUS_DELAYS (GPS_MAX_DELAYS / 2)
#define GPS_SUS_MALFORMED (GPS_MAX_MALFORMED / 2)

#define MAX_SERVICE_THRESHOLD 0xFFu

/* ------ Thresholds for data reports  ------ */


/* ------ Sensor reinitialization ------ */

#define MAX_REINIT_ATTEMPTS 3

enum sensor_mask : fu8 {
  Init_Baro = (1u << 0),
  Init_Gyro = (1u << 1),
  Init_Accl = (1u << 2),

  Shut_Baro = (1u << 4),
  Shut_Gyro = (1u << 5),
  Shut_Accl = (1u << 6),

  Init_All = (Init_Baro | Init_Gyro | Init_Accl),
  Shut_All = (Shut_Baro | Shut_Gyro | Shut_Accl)
};

#define reinit(fn, ctr, sens)             \
  do {                                    \
    fu8 k = 0;                            \
    for (; k < MAX_REINIT_ATTEMPTS ||     \
           (fn) != HAL_OK; ++k)           \
           ;                              \
    if (k >= MAX_REINIT_ATTEMPTS)         \
    {                                     \
      (ctr) += (sens);                    \
    }                                     \
  } while (0)

/* ------ Sensor reinitialization ------ */


/* ------ TX Timer interrupt definitions ------ */

/* Time is in ThreadX ticks. 1 tick = 1 ms */

#define FC_TIMEOUT 3000
#define GND_TIMEOUT 3000

/* Expiration of timer that invokes timeout checks  */
#define TX_TIMER_TICKS   500
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)

#define CO2_ASSERT_INTERVAL  500
#define REEF_ASSERT_INTERVAL 500

/* ------ TX Timer interrupt definitions ------ */


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
enum message : fu32 {
  Sensor_Measm_Code = 0,

  Bad_Altitude   = 1u,
  Bad_Pressure   = (1u << 1),
  Bad_Attitude_X = (1u << 2),
  Bad_Attitude_Y = (1u << 3),
  Bad_Attitude_Z = (1u << 4),
  Bad_Accel_X    = (1u << 5),  
  Bad_Accel_Y    = (1u << 6),
  Bad_Accel_Z    = (1u << 7),

  /* Range reserved for possibly more sensors */

  Spurious_Confirmations = (1u << 16),

  Not_Launch  = Spurious_Confirmations + 1,
  Not_Burnout = Spurious_Confirmations + 2,
  Not_Descent = Spurious_Confirmations + 3,
  Not_Reefing = Spurious_Confirmations + 4,
  Not_Landed  = Spurious_Confirmations + 5,
  
  Actionable_Decrees = (1u << 17),

  Deploy_Parachute = Actionable_Decrees + 1,
  Expand_Parachute = Actionable_Decrees + 2,
  Reinit_Sensors   = Actionable_Decrees + 3, 
  Launch_Signal    = Actionable_Decrees + 4,
  Evaluation_Relax = Actionable_Decrees + 5,
  Evaluation_Focus = Actionable_Decrees + 6,
  Evaluation_Abort = Actionable_Decrees + 7,
  Reinit_Barometer = Actionable_Decrees + 8,
  Reinit_IMU       = Actionable_Decrees + 9,
  Disable_IMU      = Actionable_Decrees + 10,
  Advance_State    = Actionable_Decrees + 11,
  Rewind_State     = Actionable_Decrees + 12,

  GPS_Data_Code = (1u << 18),

  Bad_Lattitude  = GPS_Data_Code | 1u,
  Bad_Longtitude = GPS_Data_Code | (1u << 1),
  Bad_Sea_Level  = GPS_Data_Code | (1u << 2),
  GPS_Delayed    = GPS_Data_Code | (1u << 3),
  GPS_Malformed  = GPS_Data_Code | (1u << 4),

  /* This is the only type of FC message that is stored
   * in global config. Other types are consumed immediately. */
  Runtime_Configuration = (1u << 29),

  /* User flags */
  Monitor_Altitude    = Runtime_Configuration | 1u,
  Consecutive_Samples = Runtime_Configuration | (1u << 1),
  Eval_Focus_Flag     = Runtime_Configuration | (1u << 2),
  Eval_Abort_Flag     = Runtime_Configuration | (1u << 3),
  Reset_Failures      = Runtime_Configuration | (1u << 4),
  Validate_Measms     = Runtime_Configuration | (1u << 5),

  User_Option_Bound   = Runtime_Configuration | (1u << 6), 

  /* Internal flags */
  Launch_Triggered    = Runtime_Configuration | (1u << 7),
  Parachute_Deployed  = Runtime_Configuration | (1u << 8),
  Parachute_Expanded  = Runtime_Configuration | (1u << 9),
  CO2_Asserted        = Runtime_Configuration | (1u << 10),
  REEF_Asserted       = Runtime_Configuration | (1u << 11),
  GPS_Available       = Runtime_Configuration | (1u << 12),
  Lost_GroundStation  = Runtime_Configuration | (1u << 13),
  Init_Failure_Record = Runtime_Configuration | (1u << 14),
  In_Aborted_State    = Runtime_Configuration | (1u << 15),
  Confirm_Altitude    = Runtime_Configuration | (1u << 16),
  Using_Ascent_KF     = Runtime_Configuration | (1u << 17),
  Defer_Baro_Fallback = Runtime_Configuration | (1u << 18),

  Abortion_Thresholds = Runtime_Configuration | (1u << 20),

  Abort_After_40  = Abortion_Thresholds + 40,
  Abort_After_100 = Abortion_Thresholds + 100,
  Abort_After_250 = Abortion_Thresholds + 250,

  Reinit_Thresholds = Runtime_Configuration | (1u << 21),

  Reinit_After_15 = Reinit_Thresholds + 15,
  Reinit_After_30 = Reinit_Thresholds + 30,
  Reinit_After_50 = Reinit_Thresholds + 50,

  Revoke_Option = Runtime_Configuration | (1u << 28),

  /* ... */

  GroundStation_Heartbeat = (1u << 30),
  FlightComputer_Mask = (1u << 31),

  Invalid_Message = UINT_FAST32_MAX
};

struct description {
  enum message val;
  const char *name;
};

/* ------ Universal Flight Computer message ------ */


/* ------ Helper macros ------ */

/* Length of task identifier (for sync logging) */
#define mlen(len) (len + sizeof(id))

/* Endpoint identifier: FC */
#define fc_mask(message)    ((message) | FlightComputer_Mask)
#define fc_unmask(message)  ((message) & ~FlightComputer_Mask)

/* When sending commands TO decode_message() */
#define revoke(opt) ((opt) | Revoke_Option)

/* When manipulating config OUTSIDE decode_message() */
#define option(opt) ((opt) & ~Runtime_Configuration)

/* To filter out inadequate thresholds */
#define threshold(raw) ((raw) & MAX_SERVICE_THRESHOLD)

#define namecount(arr) (sizeof(arr) / sizeof(struct description))

#define MAX_CONFIG_REPORT_SIZE 128

/* ------ Helper macros ------ */


/* ------ User default configuration ------ */

/* Run time config options applied on boot.
 * Users are welcome to edit the defaults here. */
#define DEFAULT_OPTIONS ( (enum message) (0           \
                          | Consecutive_Samples       \
                          | Eval_Focus_Flag           \
                          | Reset_Failures            \
                          | Validate_Measms           \
                          | Using_Ascent_KF           \
                        ) )

/* ------ User default configuration ------ */


/* ------ On-board relative timer implementation ------ */

enum fc_timer : fu8 {
  AscentKF,
  DescentKF,
  HeartbeatRF,
  HeartbeatGND,
  IntervalGPS,
  IntervalBaro,
  AssertCO2,
  AssertREEF,

  Time_Users
};

/* Last recorded time for each UKF timer user.
 * Defined in recovery.c to avoid multiple linkage.
 * u32 wrap is not handled (flight assumed < 49 days). */
extern volatile fu32 local_time[Time_Users];

/*
 * Report time elapsed since last call to either 
 * timer_fetch_update or timer_update,
 * and set local time to current HAL tick (ms).
 */
static inline fu32 timer_exchange(enum fc_timer u)
{
  fu32 prev = local_time[u];
  local_time[u] = now_ms();
  return local_time[u] - prev;
}

/*
 * Set local time to current HAL tick (ms).
 */
static inline void timer_update(enum fc_timer u)
{
  local_time[u] = now_ms();
}

/*
 * Report time elapsed since last call to either 
 * timer_exchange or timer_update.
 */
static inline fu32 timer_fetch(enum fc_timer u)
{
  return now_ms() - local_time[u];
}

/* ------ On-board relative timer implementation ------ */


/* ------ Parachute deployment routines ------ */

/*
 * Performs safe deployment routine.
 * Returns true on successful deployment and false otherwise.
 */
static inline bool release_parachute(void)
{
  if (flight < Ascent)
  {
    log_err("SE blocked deployment, state %u", flight);
    return false;
  }

  co2_high();

  timer_update(AssertCO2);
  fetch_or(&config, option(Parachute_Deployed | CO2_Asserted), Rel);

  return true;
}

/*
 * Performs safe expansion routine.
 * Returns true on successful deployment and false otherwise.
 */
static inline bool expand_parachute(void)
{
  if (flight < Ascent)
  {
    log_err("ND blocked expansion, state %u", flight);
    return false;
  }
  
  if (!(load(&config, Acq) & option(Parachute_Deployed)))
  {
    log_err("ND blocked expansion: no deployment");
    return false;
  }

  reef_high();

  timer_update(AssertREEF);
  fetch_or(&config, option(Parachute_Expanded | REEF_Asserted), Rel);

  return true;
}

/* ------ Parachute deployment routines ------ */


#endif // RECOVERY_H
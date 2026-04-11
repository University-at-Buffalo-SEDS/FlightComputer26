/*
 * Flight Computer data structures.
 */

#ifndef FC_DATA_STRUCTURES
#define FC_DATA_STRUCTURES

#include "fctypes.h"


/* Kalman filter */

typedef struct serial coords {
	float x, y, z;
} f_xyz;

typedef struct serial quaternion {
  float q0, rho1, rho2, rho3;
} quat;

typedef struct serial barometer {
	float alt, prs, tmp;
} baro;

typedef struct serial measurement {
	f_xyz gyro;
	union {
		f_xyz accl, gps;
	} mode;
	baro baro;
} measm;

typedef struct serial ascent_tail {
  f_xyz theta;
	float acz;
	f_xyz att;
} ekf_sv;

typedef struct serial descent_tail {
	float lat, lon;
} dkf_sv;

typedef struct serial state_vector {
	float alt, vel;
  union {
    ekf_sv asc;
    dkf_sv dsc;
  } tail;
} kf_svec;

typedef union newton_raphson_bithack {
  float f;
  uint32_t d;
} f32u;

typedef struct serial euler_angles {
  float phi, theta, psi;
} eul;


/* DMA */

typedef enum sensor_id : fu8 {
  Sensor_Baro = 0,
  Sensor_Gyro = 1,
  Sensor_Accl = 2,

  Sensors
} sens;

typedef struct gpio_lookup_table {
	const GPIO_TypeDef *port[Sensors];
  const uint16_t pin[Sensors];
  const uint16_t drdy[Sensors];
  const fu8 offset[Sensors];
} gpio_map;

typedef struct callback_selector {
  volatile fu8 next;
  volatile fu8 valid;
} dmasel;

typedef struct dma_flags {
  atomic_uint_fast16_t drdy;
  atomic_uint_fast16_t relv;
} fdma;


/* Evaluation */

typedef enum flight_state : fu8 {
  Suspended,
  Postinit,
  Awaiting,
  Launch,
  Ascent,
  Burnout,
  Apogee,
  Descent,
  Reefing,
  Landed,

  Flight_States
} state;

typedef struct state_metadata {
  float dt;
  fu32 idx;
  fu16 samp;
} sv_meta;


/* Recovery */

typedef enum relative_timer : fu8 {
  AscentKF,
  DescentKF,
  HeartbeatRF,
  HeartbeatGND,
  AssertCO2,
  AssertREEF,
  FillSequence,
  Auxiliary,

  Time_Users
} timer;

typedef enum flight_message : fu32 {
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

  Postinit_Signal  = Actionable_Decrees + 1,
  Launch_Signal    = Actionable_Decrees + 2,
  Rollback_Signal  = Actionable_Decrees + 3,
  Deploy_Parachute = Actionable_Decrees + 4,
  Expand_Parachute = Actionable_Decrees + 5,
  Reinit_Sensors   = Actionable_Decrees + 6,
  Reinit_Barometer = Actionable_Decrees + 7,
  Reinit_IMU       = Actionable_Decrees + 8,
  Disable_IMU      = Actionable_Decrees + 9,
  Evaluation_Relax = Actionable_Decrees + 10,
  Evaluation_Focus = Actionable_Decrees + 11,
  Evaluation_Abort = Actionable_Decrees + 12,
  Advance_State    = Actionable_Decrees + 13,
  Rewind_State     = Actionable_Decrees + 14,

  GPS_Data_Code = (1u << 18),

  Bad_Lattitude  = GPS_Data_Code | 1u,
  Bad_Longtitude = GPS_Data_Code | (1u << 1),
  Bad_Sea_Level  = GPS_Data_Code | (1u << 2),
  GPS_Delayed    = GPS_Data_Code | (1u << 3),
  GPS_Malformed  = GPS_Data_Code | (1u << 4),

  /* This is the only type of FC message that is stored
   * in global config. Other types are consumed immediately. */
  Runtime_Configuration = (1u << 29),

  Revoke_Option = Runtime_Configuration | (1u << 28),

  /* User flags */
  Monitor_Altitude    = Runtime_Configuration | 1u,
  Consecutive_Samples = Runtime_Configuration | (1u << 1),
  Eval_Focus_Flag     = Runtime_Configuration | (1u << 2),
  Eval_Abort_Flag     = Runtime_Configuration | (1u << 3),
  Reset_Failures      = Runtime_Configuration | (1u << 4),
  Validate_Measms     = Runtime_Configuration | (1u << 5),

  User_Option_Bound   = Runtime_Configuration | (1u << 6), 

  /* Internal flags */
  Postinit_Requested  = Runtime_Configuration | (1u << 7),
  Confirm_Postinit    = Runtime_Configuration | (1u << 8),
  Launch_Requested    = Runtime_Configuration | (1u << 9),
  Confirm_Launch      = Runtime_Configuration | (1u << 10),
  Rollback_Requested  = Runtime_Configuration | (1u << 11),
  Confirm_Rollback    = Runtime_Configuration | (1u << 12),
  Parachute_Deployed  = Runtime_Configuration | (1u << 13),
  Parachute_Expanded  = Runtime_Configuration | (1u << 14),
  CO2_Asserted        = Runtime_Configuration | (1u << 15),
  REEF_Asserted       = Runtime_Configuration | (1u << 16),
  GPS_Available       = Runtime_Configuration | (1u << 17),
  Lost_GroundStation  = Runtime_Configuration | (1u << 18),
  Init_Failure_Record = Runtime_Configuration | (1u << 19),
  In_Aborted_State    = Runtime_Configuration | (1u << 20),
  Graceful_Reset      = Runtime_Configuration | (1u << 21),
  Confirm_Altitude    = Runtime_Configuration | (1u << 22),
  Using_Ascent_KF     = Runtime_Configuration | (1u << 23),
  Defer_Baro_Fallback = Runtime_Configuration | (1u << 24),

  Abortion_Thresholds = Runtime_Configuration | (1u << 25),

  Abort_After_40  = Abortion_Thresholds + 40,
  Abort_After_100 = Abortion_Thresholds + 100,
  Abort_After_250 = Abortion_Thresholds + 250,

  Reinit_Thresholds = Runtime_Configuration | (1u << 26),

  Reinit_After_15 = Reinit_Thresholds + 15,
  Reinit_After_30 = Reinit_Thresholds + 30,
  Reinit_After_50 = Reinit_Thresholds + 50,

  /* More options go here */

  GroundStation_Heartbeat = (1u << 30),
  FlightComputer_Mask = (1u << 31),

  Invalid_Message = UINT_FAST32_MAX
} fc_msg;

typedef struct config_description_map {
  const fc_msg val;
  const char *name;
} conf_dict;

typedef struct system_monitor {
  fu16 to_abort, to_reinit;
  fu16 gps_delay, gps_malform;
  volatile fu16 failures;
} sysmon;

typedef enum sensor_init_mask : fu8 {
  Init_Baro = (1u << 0),
  Init_Gyro = (1u << 1),
  Init_Accl = (1u << 2),

  Shut_Baro = (1u << 4),
  Shut_Gyro = (1u << 5),
  Shut_Accl = (1u << 6),

  Init_All = (Init_Baro | Init_Gyro | Init_Accl),
  Shut_All = (Shut_Baro | Shut_Gyro | Shut_Accl)
} sens_init;


#endif /* FC_DATA_STRUCTURES */
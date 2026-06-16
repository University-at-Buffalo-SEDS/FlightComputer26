/* Core/Inc/fcstructs.h */

#ifndef FC_DATA_STRUCTURES
#define FC_DATA_STRUCTURES

#include "platform.h"
#include "fctypes.h"
#include "fccommon.h"


/* Internal API */

typedef enum relative_timer : fu8 {
  AscentKF,
  DescentKF,
  HeartbeatRF,
  HeartbeatGND,
  AssertCO2,
  AssertREEF,
  GPSWatchdog,
  Auxiliary,
  PostinitCmd,
  LaunchCmd,
  BaroLocal,
  BaroRemote,
  IMULocal,
  IMURemote,
  KFLocal,
  KFRemote,
  SDFlush,
  MessageRemote,
  MessageLocal,

  Time_Users
} timer;

typedef struct wakeyield_spinlock {
  atomic_uint_fast8_t lock, waiters;
} spinlock;


/* Logging & reporting */

typedef enum led_kind : fu8 {
  Green,
  Blue,

  Leds
} led;

typedef struct led_gpio {
  GPIO_TypeDef *port;
  uint16_t pin;
} led_gpio;

typedef enum logger_bound_mask : fu8 {
  Log_User_Bound = 0x0,
  Log_Rate_Bound = 0x1,
  Log_Local_Bound = 0x2,
  Log_Shutdown = 0x4,
} log_mask;

typedef struct telemetry_log_rates {
  fu32 sd, gnd;
} log_rates;

typedef struct rated_logger_lookup {
  fu8 tim_sd, tim_gnd, size;
  fu16 kind_sd, kind_gnd;
} log_lookup;

#ifdef SD_AVAILABLE

typedef struct sd_buffer_metadata {
  TX_SEMAPHORE full;
  fu32 off[2];
  spinlock lock;
  volatile bool cur, free;
} sd_meta;

#endif


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

typedef struct serial gps_reversed {
  float sea, lon, lat;
} kf_gps;

typedef struct serial measurement {
  kf_gps gps;
	baro baro;
  f_xyz accl;
  f_xyz gyro;
} measm;

typedef struct serial ascent_bias {
  float az, gx, gy, gz;
} ekf_bias;

typedef struct serial state_vector {
  kf_gps gps;
	float alt, vel;
  ekf_bias bias;
} kf_svec;

typedef struct serial euler_angles {
  float phi, theta, psi;
} eul;

typedef struct kf_matrix_objects {
  matrix mxp, mxq, mxa, mxr, mxh;
} kf_matrix;


/* DMA */

typedef enum sensor_id : fu8 {
  Sensor_Baro = 0,
  Sensor_Gyro = 1,
  Sensor_Accl = 2,

  Sensors
} sens;

typedef struct cm_align gpio_lookup_table {
	const GPIO_TypeDef *port[Sensors];
  const uint16_t pin[Sensors];
  const uint16_t drdy[Sensors];
  const fu8 offset[Sensors];
} gpio_map;

typedef struct cm_align callback_selector {
  volatile fu8 next;
  volatile fu8 valid;
} dmasel;

typedef struct cm_align dma_flags {
  atomic_uint_fast16_t drdy;
  atomic_uint_fast16_t relv;
} fdma;

typedef enum mems_device_id : fu8 {
  IMU,
  Baro,

  MEMS_Devices
} devid;


/* Evaluation */

typedef enum flight_state : fu8 {
  Startup,
  Postinit,
  Armed,
  Launch,
  Ascent,
  Coast,
  Apogee,
  Descent,
  Reefing,
  Landed,
  Recovery,

  Flight_States
} state;

typedef enum global_state : uint8_t {
  G_Startup,
  G_Idle,
  G_PreFill,
  G_FillTest,
  G_NitrogenFill,
  G_NitrousFill,
  G_Armed,
  G_Launch,
  G_Ascent,
  G_Coast,
  G_Apogee,
  G_Descent,
  G_Reefing,
  G_Landed,
  G_Recovery,
  G_Aborted,

  Global_States
} gnd_state;

typedef struct cm_align state_metadata {
  atomic_uint_fast8_t flight;
  uint8_t global_state;
  fu16 idx;
  fi16 confidence;
  fi16 kf_deviations;
} sv_meta;

typedef struct rf_distribution_block {
  kf_gps rail;
  f_xyz coords_buf;
  spinlock rflock;
  bool updated;
} rf_receiver;

typedef struct flight_statistics {
  float max_alt, max_vel;
} stats;


/* Recovery */

typedef enum flight_message : fu32 {
  Sensor_Measm_Code = (1u << 16),

  Bad_Altitude   = Sensor_Measm_Code | 1u,
  Bad_Pressure   = Sensor_Measm_Code | (1u << 1),
  Bad_Attitude_X = Sensor_Measm_Code | (1u << 2),
  Bad_Attitude_Y = Sensor_Measm_Code | (1u << 3),
  Bad_Attitude_Z = Sensor_Measm_Code | (1u << 4),
  Bad_Accel_X    = Sensor_Measm_Code | (1u << 5),  
  Bad_Accel_Y    = Sensor_Measm_Code | (1u << 6),
  Bad_Accel_Z    = Sensor_Measm_Code | (1u << 7),
  
  Actionable_Decrees = (1u << 17),

  Postinit_Signal  = Actionable_Decrees + 1,
  Launch_Signal    = Actionable_Decrees + 2,

  Deploy_Parachute = Actionable_Decrees + 3,
  Expand_Parachute = Actionable_Decrees + 4,
  Reinit_Sensors   = Actionable_Decrees + 5,
  Reinit_Barometer = Actionable_Decrees + 6,
  Reinit_IMU       = Actionable_Decrees + 7,
  Disable_IMU      = Actionable_Decrees + 8,
  Evaluation_Relax = Actionable_Decrees + 9,
  Evaluation_Focus = Actionable_Decrees + 10,
  Evaluation_Abort = Actionable_Decrees + 11,
  Advance_State    = Actionable_Decrees + 12,
  Rewind_State     = Actionable_Decrees + 13,

  Log_Rate_Limit   = Actionable_Decrees + 14,
  Log_Restrict     = Actionable_Decrees + 15,
  Log_Terminate    = Actionable_Decrees + 16,

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

  /* User */
  Vigilant_Mode       = Runtime_Configuration | 1u,
  Eval_Successive     = Runtime_Configuration | (1u << 1),
  Eval_Focused        = Runtime_Configuration | (1u << 2),
  Eval_Abort_Flag     = Runtime_Configuration | (1u << 3),
  Reset_Failures      = Runtime_Configuration | (1u << 4),
  Measm_Reports       = Runtime_Configuration | (1u << 5),
  Velocity_Checks     = Runtime_Configuration | (1u << 6),

  User_Option_Bound   = Runtime_Configuration | (1u << 7), 

  /* Internal */
  Postinit_Requested  = Runtime_Configuration | (1u << 8),
  Launch_Requested    = Runtime_Configuration | (1u << 9),
  Parachute_Deployed  = Runtime_Configuration | (1u << 10),
  Parachute_Expanded  = Runtime_Configuration | (1u << 11),
  CO2_Asserted        = Runtime_Configuration | (1u << 12),
  REEF_Asserted       = Runtime_Configuration | (1u << 13),
  GPS_Available       = Runtime_Configuration | (1u << 14),
  Lost_GroundStation  = Runtime_Configuration | (1u << 15),
  Init_Failure_Record = Runtime_Configuration | (1u << 16),
  In_Aborted_State    = Runtime_Configuration | (1u << 17),
  Graceful_Reset      = Runtime_Configuration | (1u << 18),
  Manual_Biases       = Runtime_Configuration | (1u << 19),
  Ascent_KF_Staged    = Runtime_Configuration | (1u << 20),
  Using_Ascent_KF     = Runtime_Configuration | (1u << 21),
  Defer_Baro_Fallback = Runtime_Configuration | (1u << 22),
  SD_Pipeline_Reset   = Runtime_Configuration | (1u << 23),

  Abortion_Thresholds = Runtime_Configuration | (1u << 26),
  Reinit_Thresholds   = Runtime_Configuration | (1u << 27),

  FlightComputer_Mask = (1u << 31),
  Invalid_Message = UINT_FAST32_MAX
} fc_msg;

typedef struct config_description_map {
  const fc_msg val;
  const char *name;
} conf_dict;

typedef struct cm_align system_monitor {
  fu16 to_abort, to_reinit;
  fu16 gps_delayed, gps_malform;
  fu16 failures;
  fu16 triggers;
} sysmon;

typedef enum sensor_init_mask : fu8 {
  Baro_Mask = (1u << 0),
  Gyro_Mask = (1u << 1),
  Accl_Mask = (1u << 2),

  Shut_Baro = (1u << 4),
  Shut_Gyro = (1u << 5),
  Shut_Accl = (1u << 6),

  Wild_Mask = (Baro_Mask | Gyro_Mask | Accl_Mask),
  Shut_Mask = (Shut_Baro | Shut_Gyro | Shut_Accl)
} sens_init;

typedef enum remote_cmd_compat : uint8_t
{
  Compat_Postinit_Signal,
  Compat_Launch_Signal,

  Compat_Vigilant_Mode,
  Revoke_Vigilant_Mode,
  Compat_Eval_Successive,
  Revoke_Eval_Successive,
  Compat_Reset_Failures,
  Revoke_Reset_Failures,
  Compat_Measm_Reports,
  Revoke_Measm_Reports,
  Compat_Velocity_Checks,
  Revoke_Velocity_Checks,

  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,
  Compat_Reinit_Sensors,
  Compat_Reinit_Barometer,
  Compat_Reinit_IMU,
  Compat_Disable_IMU,
  Compat_Advance_State,
  Compat_Rewind_State,

  Compat_Abort_After_40,
  Compat_Abort_After_100,
  Compat_Abort_After_250,
  Compat_Reinit_After_15,
  Compat_Reinit_After_30,
  Compat_Reinit_After_50,

  Compat_Messages
} compat;


#endif /* FC_DATA_STRUCTURES */
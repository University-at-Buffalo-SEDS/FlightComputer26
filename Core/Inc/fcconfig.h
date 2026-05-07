/* Core/Inc/fcconfig.h */

#ifndef FC_USER_CONFIG
#define FC_USER_CONFIG

#ifdef SIMULATION_CONFIG

#include "simulation.h"

#else


/* IREC 2026 */

#define GRAVITY_SI 9.80665f

#define LAUNCH_SITE_LAT 31.9436879f
#define LAUNCH_SITE_LON -102.214251f
#define LAUNCH_SITE_SEA	872.0f

#define IGNITION_COMMAND 14


/* Logging */

#define MSP_STACK_MARGIN	32768
#define TELEMETRY_HEAP 		65536

#define LOG_RATE_SD	 			50
#define LOG_RATE_GND 			1000
#define LOG_RATE_SD_LTD  	150
#define LOG_RATE_GND_LTD	2000

#define SD_BUFFER_SIZE		32768
#define SD_FLUSH_INTERVAL 30000
#define SD_MAX_MSG_SIZE		256


/* Deployment GPIO (sensors' GPIO is in main.h) */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6


/* Ring data structures capacity */

#define GPS_RING_SIZE 4
#define STATE_HISTORY 8


/* Evaluation */

#define SPURIOUS_THRESHOLD	2
#define SPURIOUS_PENALTY	 	1

#define MIN_SAMP_LAUNCH		2
#define MIN_SAMP_ASCENT   3
#define MIN_SAMP_COAST  	5
#define MIN_SAMP_APOGEE		4
#define MIN_SAMP_DESCENT  4
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   36

#define LAUNCH_MIN_VEL  	24.0f
#define ASCENT_MIN_VEL		18.0f
#define COAST_MIN_VEL 		12.0f
#define APOGEE_MAX_VEL 		3.0f
#define DESCENT_MIN_VEL		4.0f
#define REEF_TARGET_ALT 	457.2f
#define FLYING_ALTITUDE		100.0f

#define LAUNCH_CONFIRM_DELAY 30
#define APOGEE_CONFIRM_DELAY 80
#define RECOVERY_ANNOUNCE_DELAY 5000
#define URGENT_DEPLOYMENT_DELAY 750

#define KF_FLUC_THRES 	 80
#define VIGILANT_MAX_ALT 3657.6f
#define VIGILANT_MIN_ALT -25.0f
#define VIGILANT_MIN_APG 762.0f
#define HYBRID_VEL_SPIKE -110.0f
#define SVHIST_ALT_TREND 8.0f
#define VIGILANT_MIN_VEL 7.5f

/* Altitude         ALT     meters  
 * Pressure         PRS     pascals
 * Attitude         DPS     deg/sec
 * Acceleration     ACC     m/s^2
 * Lattitude        LAT     degrees
 * Longtitude       LON     degrees
 * Alt. above sea   SEA     meters    */

#define MAX_ALT 4572.0f
#define MAX_PRS 110000.0f
#define MAX_DPS 2000.0f
#define MAX_ACC (GRAVITY_SI * 24.0f)
#define MAX_LAT (LAUNCH_SITE_LAT + 1.0f)
#define MAX_LON (LAUNCH_SITE_LON + 1.0f)
#define MAX_SEA (LAUNCH_SITE_SEA + MAX_ALT)

#define MIN_ALT -30.0f
#define MIN_PRS 57200.0f
#define MIN_DPS -MAX_DPS
#define MIN_ACC -MAX_ACC
#define MIN_LAT (LAUNCH_SITE_LAT - 1.0f)
#define MIN_LON (LAUNCH_SITE_LON - 1.0f)
#define MIN_SEA (LAUNCH_SITE_SEA - 100.0f)

#define ALT_TOLER 2.0f
#define VEL_TOLER 1.5f
#define GPS_TOLER 1.41f
#define GPS_RAIL_TOLER 0.05f


/* Kalman */

#define DKF_GPS_TRUST	1.2f
#define DKF_BARO_TRUST 0.05f

#define EKF_BARO_VARIANCE 0.1f
#define EKF_ACCL_RAIL_DIV (GRAVITY_SI * 2.0f)

#define EKF_BIAS_GYRO_X 0
#define EKF_BIAS_GYRO_Y 0
#define EKF_BIAS_GYRO_Z 0
#define EKF_BIAS_ACCL_Z 0


/* Recovery */

#define TO_REINIT 40
#define TO_ABORT  120

#define SENSOR_REINIT_ATTEMPTS 3

#define GPS_DELAY_MS 		200
#define GPS_MAX_DELAYS	40
#define GPS_MAX_MALFORM 40
#define GPS_SUS_DELAYS	(GPS_MAX_DELAYS / 2)
#define GPS_SUS_MALFORM (GPS_MAX_MALFORM / 2)

#define FC_TIMEOUT	3000
#define GND_TIMEOUT 3000

#define CONFIRMATION_TIMEOUT 5000

#define POSTINIT_DURATION 5000
#define POSTINIT_INTERVAL 100

#define TX_TIMER_TICKS   450
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)

#define CO2_ASSERT_INTERVAL  500
#define REEF_ASSERT_INTERVAL 500

#define USER_OPTIONS ( (fc_msg) (0                   			\
                      | option(Eval_Focus_Flag)           \
                      | option(Reset_Failures)            \
                      | option(Measm_Reports)    					\
                      ) )

#define LED_BLOCKING_CYCLES 10000000
#define LED_BLINKS_ON_CO2	 5
#define LED_BLINKS_ON_REEF 20


#endif /* SIMULATION_CONFIG */
#endif /* FC_USER_CONFIG */
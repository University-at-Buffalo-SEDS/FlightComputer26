/* Core/Test/simulation.h */

#ifndef SIMULATION_H
#define SIMULATION_H


/* IREC 2026 */

#define GRAVITY_SI 9.80665f

#define LAUNCH_SITE_LAT 43.003556f
#define LAUNCH_SITE_LON -78.768556f

#define IGNITION_COMMAND 14


/* Deployment GPIO (sensors' GPIO is in main.h) */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6


/* Ring data structures capacity */

#define GPS_RING_SIZE 4
#define STATE_HISTORY 8


/* Evaluation */

#define SPURIOUS_THRESHOLD 1

#define MIN_SAMP_ASCENT   3
#define MIN_SAMP_BURNOUT  5
#define MIN_SAMP_DESCENT  4
#define MIN_SAMP_REEF     2
#define MIN_SAMP_LANDED   6

#define LAUNCH_CONFIRM_DELAY 30
#define APOGEE_CONFIRM_DELAY 80
#define LANDED_GPS_INTERVAL  1000

#define URGENT_DEPLOYMENT_DELAY 200

/* Altitude         ALT     meters  
 * Pressure         PRS     pascals
 * Attitude         DPS     deg/sec
 * Acceleration     ACC     m/s^2
 * Lattitude        LAT     degrees
 * Longtitude       LON     degrees
 * Alt. above sea   SEA     meters    */

#define MAX_ALT 500.0f
#define MAX_PRS 125000.0f
#define MAX_DPS 2000.0f
#define MAX_ACC (GRAVITY_SI * 12.0f)
#define MAX_LAT 90.0f
#define MAX_LON 180.0f
#define MAX_SEA 9999.9f

#define MIN_ALT -20.0f
#define MIN_PRS 30000.0f
#define MIN_DPS -MAX_DPS
#define MIN_ACC -MAX_ACC
#define MIN_LAT -MAX_LAT
#define MIN_LON -MAX_LON
#define MIN_SEA -999.9f

#define LAUNCH_MIN_VEL  1.0f
#define LAUNCH_MIN_VAX  2.0f

#define BURNOUT_MIN_VEL 2.0f
#define BURNOUT_MAX_VAX -1.0f

#define APOGEE_MAX_VEL 0.5f

#define REEF_TARGET_ALT 10.0f

#define ALT_TOLER 0.5f
#define VEL_TOLER 0.5f
#define VAX_TOLER 0.5f
#define GPS_TOLER 1.0f

#define GPS_RAIL_TOLER 0.02f

#define VIGILANT_MAX_ALT 400.0f
#define VIGILANT_MIN_ALT 0.5f


/* Kalman */

#define DKF_GPS_TRUST	1.2f
#define DKF_BARO_TRUST 0.05f

#define EKF_BARO_VARIANCE 2.0f
#define EKF_ACCL_RAIL_DIV (GRAVITY_SI * 1.5f)

#define EKF_BIAS_GYRO_X 0
#define EKF_BIAS_GYRO_Y 0
#define EKF_BIAS_GYRO_Z 0
#define EKF_BIAS_ACCL_Z 0


/* Recovery */

#define TO_REINIT 40
#define TO_ABORT  80

#define SENSOR_REINIT_ATTEMPTS 2

#define GPS_DELAY_MS 200

#define GPS_MAX_DELAYS	40
#define GPS_MAX_MALFORM 30

#define GPS_SUS_DELAYS	(GPS_MAX_DELAYS / 2)
#define GPS_SUS_MALFORM (GPS_MAX_MALFORM / 2)

#define FC_TIMEOUT	3000
#define GND_TIMEOUT 3000

#define POSTINIT_DURATION 5000
#define POSTINIT_INTERVAL 150
#define IGNITER_SEQ_TIMER 1200

#define CONFIRMATION_TIMEOUT 5000

#define TX_TIMER_TICKS   500
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)

#define CO2_ASSERT_INTERVAL  500
#define REEF_ASSERT_INTERVAL 500

#define USER_OPTIONS ( (fc_msg) (0                   			\
											| option(Consecutive_Samples)       \
											| option(Eval_Focus_Flag)           \
											| option(Reset_Failures)            \
											| option(Measm_Reports)    					\
                      ) )

#define LED_BLOCKING_CYCLES 80000000
#define LED_BLINKS_ON_CO2	 5
#define LED_BLINKS_ON_REEF 20


#endif /* SIMULATION_H */
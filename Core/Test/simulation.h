/* Core/Test/simulation.h */

#ifndef SIMULATION_H
#define SIMULATION_H


/* IREC 2026 */

#define GRAVITY_SI 9.80665f

#define LAUNCH_SITE_LAT 43.003556f
#define LAUNCH_SITE_LON -78.768556f
#define LAUNCH_SITE_SEA	182.0f

#define IGNITION_COMMAND 14

#define ALLOC_MAX 	8192
#define ALLOC_ALIGN	4
#define ALLOC_MAGIC (greedy_hdr *)(0xFEE1DEAD)


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

#define MAX_ALT 250.0f
#define MAX_PRS 110000.0f
#define MAX_DPS 360.0f
#define MAX_ACC (GRAVITY_SI * 4.0f)
#define MAX_LAT (LAUNCH_SITE_LAT + 1.0f)
#define MAX_LON (LAUNCH_SITE_LON + 1.0f)
#define MAX_SEA (LAUNCH_SITE_SEA + MAX_ALT)

#define MIN_ALT -5.0f
#define MIN_PRS 80000.0f
#define MIN_DPS -MAX_DPS
#define MIN_ACC -MAX_ACC
#define MIN_LAT (LAUNCH_SITE_LAT - 1.0f)
#define MIN_LON (LAUNCH_SITE_LON - 1.0f)
#define MIN_SEA (LAUNCH_SITE_SEA - 100.0f)

#define LAUNCH_MIN_VEL  1.0f
#define BURNOUT_MIN_VEL 2.0f
#define APOGEE_MAX_VEL 	0.5f
#define REEF_TARGET_ALT 10.0f

#define ALT_TOLER 0.5f
#define VEL_TOLER 0.5f
#define GPS_TOLER 1.0f
#define GPS_RAIL_TOLER 0.05f

#define VIGILANT_MAX_ALT 400.0f
#define VIGILANT_MIN_ALT 2.0f

#define STABILIZATION_PAD 60
#define STABILIZATION_STEPS 20


/* Kalman */

#define DKF_GPS_TRUST	1.2f
#define DKF_BARO_TRUST 0.05f

#define EKF_BARO_VARIANCE 0.1f
#define EKF_ACCL_RAIL_DIV (GRAVITY_SI * 1.5f)

#define EKF_BIAS_GYRO_X 0
#define EKF_BIAS_GYRO_Y 0
#define EKF_BIAS_GYRO_Z 0
#define EKF_BIAS_ACCL_Z 0


/* Recovery */

#define TO_REINIT 20
#define TO_ABORT  60

#define SENSOR_REINIT_ATTEMPTS 2

#define GPS_DELAY_MS 		125
#define GPS_MAX_DELAYS	20
#define GPS_MAX_MALFORM 16
#define GPS_SUS_DELAYS	(GPS_MAX_DELAYS / 2)
#define GPS_SUS_MALFORM (GPS_MAX_MALFORM / 2)

#define FC_TIMEOUT	2000
#define GND_TIMEOUT 2000

#define CONFIRMATION_TIMEOUT 5000

#define POSTINIT_DURATION 5000
#define POSTINIT_INTERVAL 100

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

#define LED_BLOCKING_CYCLES 10000000
#define LED_BLINKS_ON_CO2	 5
#define LED_BLINKS_ON_REEF 20


#endif /* SIMULATION_H */
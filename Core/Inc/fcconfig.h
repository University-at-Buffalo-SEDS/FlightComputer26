/*
 * Flight Computer user configuration file.
 * All time constants are in milliseconds.
 */

#ifndef FC_USER_CONFIG
#define FC_USER_CONFIG


/* Constants */

#define GRAVITY_SI 9.80665f


/* Deployment GPIO 
 * (sensors' GPIO is in main.h) */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6


/* Ring data structures capacity */

#define GPS_RING_SIZE 4
#define STATE_HISTORY 4


/* Evaluation */

#define MIN_SAMP_ASCENT   3
#define MIN_SAMP_BURNOUT  3
#define MIN_SAMP_DESCENT  2
#define MIN_SAMP_REEF     1
#define MIN_SAMP_LANDED   6

#define LAUNCH_CONFIRM_DELAY 30
#define APOGEE_CONFIRM_DELAY 80
#define LANDED_GPS_INTERVAL  500

#define URGENT_DEPLOYMENT_DELAY 500

/* Measurement thresholds:
 * Altitude         ALT     meters  
 * Pressure         PRS     pascals
 * Attitude         DPS     deg/sec
 * Acceleration     ACC     m/s^2
 * Lattitude        LAT     degrees
 * Longtitude       LON     degrees
 * Alt. above sea   SEA     meters    */

#define MAX_ALT 4800.0f
#define MAX_PRS 125000.0f
#define MAX_DPS 2000.0f
#define MAX_ACC (GRAVITY_SI * 24.0f)
#define MAX_LAT 90.0f
#define MAX_LON 180.0f
#define MAX_SEA 9999.9f

#define MIN_ALT -10.0f
#define MIN_PRS 30000.0f
#define MIN_DPS -MAX_DPS
#define MIN_ACC -MAX_ACC
#define MIN_LAT -MAX_LAT
#define MIN_LON -MAX_LON
#define MIN_SEA -999.9f

#define LAUNCH_MIN_VEL  8.0f
#define LAUNCH_MIN_VAX  10.0f

#define BURNOUT_MIN_VEL 12.0f
#define BURNOUT_MAX_VAX -3.0f

#define APOGEE_MAX_VEL -3.0f

#define REEF_TARGET_ALT 457.2f

#define ALT_TOLER 2.0f
#define VEL_TOLER 1.5f
#define VAX_TOLER 1.0f
#define GPS_TOLER 1.41f

#define GPS_RAIL_TOLER 0.05f

/* Midland, TX */
#define LAUNCH_SITE_LAT 32.000507f
#define LAUNCH_SITE_LON -102.077408f


/* Recovery */

#define TO_REINIT 20
#define TO_ABORT  40

#define SENSOR_REINIT_ATTEMPTS 3

#define GPS_DELAY_MS 125

#define GPS_MAX_DELAYS	40
#define GPS_MAX_MALFORM 30

#define GPS_SUS_DELAYS	(GPS_MAX_DELAYS / 2)
#define GPS_SUS_MALFORM (GPS_MAX_MALFORM / 2)

#define FC_TIMEOUT	3000
#define GND_TIMEOUT 3000

/* Expiration of timer that invokes timeout checks  */
#define TX_TIMER_TICKS   500
#define TX_TIMER_INITIAL (TX_TIMER_TICKS * 2)

#define CO2_ASSERT_INTERVAL  500
#define REEF_ASSERT_INTERVAL 500

#define USER_OPTIONS ( (fc_msg) (0                   			\
											| option(Consecutive_Samples)       \
											| option(Eval_Focus_Flag)           \
											| option(Reset_Failures)            \
											| option(Validate_Measms)           \
                      ) )


#endif /* FC_USER_CONFIG */
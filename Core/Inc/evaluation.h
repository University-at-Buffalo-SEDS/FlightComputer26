/*
 * Data evaluation configuration and API.
 */

#ifndef EVALUATION_H
#define EVALUATION_H

#include "dma.h"
#include "platform.h"


/* ------ Local configuration ------ */

#define MIN_SAMP_ASCENT   3
#define MIN_SAMP_BURNOUT  3
#define MIN_SAMP_DESCENT  3
#define MIN_SAMP_REEF     2
#define MIN_SAMP_LANDED   3

/* Delay in ThreadX ticks */
#define LAUNCH_CONFIRM_DELAY 25
#define APOGEE_CONFIRM_DELAY 75

/* Measurement thresholds:
 * Altitude         ALT     meters  
 * Pressure         PRS     pascals
 * Attitude         DPS     deg/sec
 * Acceleration     ACC     m/s^2
 * Lattitude        LAT     degrees
 * Longtitude       LON     degrees
 * Alt. above sea   SEA     meters    */

#define GRAVITY_SI 9.80665f

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

#define LAUNCH_SITE_LAT 32.000507f
#define LAUNCH_SITE_LON -102.077408f

#define LAUNCH_MIN_VEL  8.0f
#define LAUNCH_MIN_VAX  10.0f

#define BURNOUT_MIN_VEL 12.0f
#define BURNOUT_MAX_VAX -3.0f

#define APOGEE_MAX_VEL -3.0f

#define REEF_TARGET_ALT 457.2f

#define ALT_TOLER 2.0f
#define VEL_TOLER 1.5f
#define VAX_TOLER 1.0f
#define GPS_TOLER 1.0f  

#define lat_within_launch_site(k)                         \
  (fabsf((float)(k) - LAUNCH_SITE_LAT) <= GPS_TOLER)

#define lon_within_launch_site(k)                         \
  (fabsf((float)(k) - LAUNCH_SITE_LON) <= GPS_TOLER)


/* ------ Put/fetch algorithm definitions ------ */

#define RING_SIZE 4
#define RING_MASK (RING_SIZE - 1)

#define CLEAR_IDX ((fu16)UINT_FAST8_MAX << 8)


/* ------ Data containers ------ */

struct serial quaternion {
  float q1, q2, q3, q4;
};

struct serial state_vec {
  struct coords p, v, a;
  struct quaternion qv;
  struct coords w;
};

/// Piece of measm_z accepted by Descent filter
struct serial descent {
  union {
    struct coords accl;
    struct coords gps;
  } axis;

  float alt;
};

/// Full measurement excluding baro temperature and pressure.
struct serial measm_z {
  struct coords gyro;
  struct descent d;
};


/// In-flight rocket states only since used internally.
enum state {
  Suspended,
  Idle,
  Launch,
  Ascent,
  Burnout,
  Apogee,
  Descent,
  Reefing,
  Landed,
};


/* ------ Public API ------ */

/// Enqueues raw data set for processing by KF.
void evaluation_put(const struct measurement *buf);


#endif // EVALUATION_H
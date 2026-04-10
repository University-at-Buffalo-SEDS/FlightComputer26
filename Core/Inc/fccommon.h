/*
 * Flight Computer common constants and macros.
 */

#ifndef FC_COMMON
#define FC_COMMON

#include "platform.h"
#include "fcstructs.h"
#include "fcconfig.h"


/* DMA constants */

#define SENSOR_BUF_SIZE 8

#define DMA_TIMEOUT_MS 50

#define BARO_MASK BARO_EXTI_Pin
#define GYRO_MASK (GYRO_EXTI_1_Pin | GYRO_EXTI_2_Pin)
#define ACCL_MASK (ACCL_EXTI_1_Pin | ACCL_EXTI_2_Pin)


/* Evaluation constants */

#define FLOAT_LOG_PRECISION 8
#define MAX_TRANSITION_SIZE 56

#define MAX_REPORT_SIZE 															\
	(MAX_TRANSITION_SIZE + FLOAT_LOG_PRECISION)

#define GPS_RING_SIZE_MASK (GPS_RING_SIZE - 1)
#define STATE_HISTORY_MASK (STATE_HISTORY - 1)

#define CLEAR_IDX ((fu16)UINT_FAST8_MAX << 8)

#define CAN_EVALUATE UINT_FAST32_MAX

#define IMU_ID (Sensor_Gyro | Sensor_Accl)
#define ASCENT_PREDICT_DONE (fu8)(1u << 7)


/* Kalman filter constants */

#define TOLERANCE 1e-3f

#define NR_ITERATIONS 2

#define ASC_STAT 16
#define ASC_MEAS 7
#define DESC_STAT 6
#define DESC_MEAS 4

# if ASC_STAT > DESC_STAT 
#   define L ASC_STAT
# else
#   define L DESC_STAT
# endif

# if ASC_MEAS > DESC_MEAS 
#   define M ASC_MEAS
# else
#   define M DESC_MEAS
# endif

#define DESC_MASK (DESC_STAT - 1)
#define APEX_A    (DESC_MASK - 2)

#define SIGMA_GYRO  	0.1f
#define SIGMA_GYRO_Z	1e-6f
#define SIGMA_ACC   	0.1f
#define SIGMA_ALT   	10.0f

#define ALPHA         1.0f
#define BETA          2.0f
#define KAPPA         (1.5f * L)

#define W_DIM         (int)(2*L + 1)
#define W_l           (W_DIM - 1)

#define W_0_a         (float)((ALPHA*ALPHA*KAPPA - L) \
                          		 / (ALPHA*ALPHA*KAPPA))
#define W_l_a         (float)(1.0f / (2.0f*ALPHA*ALPHA*KAPPA))
#define W_0_c         (float)(W_0_a + 1.0f - ALPHA*ALPHA + BETA)
#define W_l_c         W_l_a


/* Recovery constants  */

#define MAX_THRESHOLD 0xFFu
#define FC_MSG_Q_SIZE 8

#define MAX_CONFIG_REPORT_SIZE 128
#define INVALID_MESSAGE_STATUS 0xFFu

#define FC_DEFAULTS ( (fc_msg) (0                   	  \
                    | option(Using_Ascent_KF)           \
                    ) )


/* Common macros */

#define mlen(len) (len + sizeof(id))

#define fc_mask(message)    ((message) | FlightComputer_Mask)
#define fc_unmask(message)  ((message) & ~FlightComputer_Mask)

#define option(opt) ((opt) & ~Runtime_Configuration)
#define revoke(opt) ((opt) | Revoke_Option)

#define threshold(raw) ((raw) & MAX_THRESHOLD)

#define namecount(arr) (sizeof(arr) / sizeof(conf_dict))

#define sigma_low(k)  ((float)k - TOLERANCE)
#define sigma_high(k) ((float)k + TOLERANCE)

#define fsec(ms) ((float)(ms) * 0.001f)

#define within(expr, bound)                               \
  (fabsf((float)(expr)) <= (bound))

#define proxim_lat(k)                                     \
  within((k) - LAUNCH_SITE_LAT, GPS_TOLER)

#define proxim_lon(k)                                     \
  within((k) - LAUNCH_SITE_LON, GPS_TOLER)

/* Go 'k' state vectors back, 0 to get the current vector
 */
#define svec(k) (sv[(((sm.idx) - (k)) & STATE_HISTORY_MASK)])

#define try_init_sensor(_fn, _ctr, _sn)   \
  do {                                    \
    fu8 k = 0;                            \
    for (; k < SENSOR_REINIT_ATTEMPTS &&  \
           (_fn) != HAL_OK; ++k)          \
           ;                              \
    if (k >= SENSOR_REINIT_ATTEMPTS)      \
    {                                     \
      (_ctr) += (_sn);                    \
    }                                     \
  } while (0)



#endif /* FC_COMMON */
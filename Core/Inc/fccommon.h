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
#define DKF_TOLER 1e-2f

#define NR_ITERATIONS 2

#define EKF_STATE 9
#define EKF_MEASM 7
#define DKF_STATE 4
#define DKF_MEASM 3

#define DKF_STATE_SQ (DKF_STATE * DKF_STATE)
#define DKF_MEASM_SQ (DKF_MEASM * DKF_MEASM)
#define EKF_STATE_SQ (EKF_STATE * EKF_STATE)
#define EKF_MEASM_SQ (EKF_MEASM * EKF_MEASM)

#define DKF_ST_ME (DKF_STATE * DKF_MEASM)
#define EKF_ST_ME (EKF_STATE * EKF_STATE)

#define DKF_PREDICT_BYTES fbyte(DKF_STATE_SQ * 3)
#define DKF_UPDATE_BYTES  fbyte(DKF_MEASM_SQ * 2                \
                                 + DKF_ST_ME * 3)

#define EKF_PREDICT_BYTES fbyte(EKF_STATE_SQ * 000000000)
#define EKF_UPDATE_BYTES  fbyte(EKF_STATE_SQ * 000000000)

#define L maxd(EKF_STATE, DKF_STATE)
#define M maxd(EKF_MEASM, DKF_MEASM)

/* TX reserves 2 pointers per block for metadata.
 * A KF function allocates 1 block and releases it.
 * No fragmentation, 4-alignment => ~ O(1) allocation.
 */
#define KFP_OVERHEAD (2 * sizeof(size_t))

#define KF_POOL_USED maxq(DKF_PREDICT_BYTES, DKF_UPDATE_BYTES, \
                          EKF_PREDICT_BYTES, EKF_UPDATE_BYTES)

#define KF_POOL_SIZE (KF_POOL_USED + KFP_OVERHEAD)


/* Recovery constants */

#define MAX_THRESHOLD 0x3FFu
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

#define maxd(a, b) ((a) > (b) ? (a) : (b))
#define maxq(a, b, c, d) maxd(maxd(a, b), maxd(c, d))

#define sigma_low(k)  ((float)k - TOLERANCE)
#define sigma_high(k) ((float)k + TOLERANCE)

#define fsec(ms) ((float)(ms) * 0.001f)
#define fbyte(f) ((f) * sizeof(float))

#define ekf_view(vec) (float *)((void *)(vec) + sizeof(kf_gps))
#define dkf_view(vec) (float *)((void *)(vec) + sizeof(float))

#define within(expr, bound)                               \
  (fabsf((float)(expr)) <= (bound))

#define proxim_lat(k)                                     \
  within((k) - LAUNCH_SITE_LAT, GPS_TOLER)

#define proxim_lon(k)                                     \
  within((k) - LAUNCH_SITE_LON, GPS_TOLER)

/* Go 'k' state vectors back, 0 to get the current vector.
 */
#define svec(k) (sv[(((sm.idx) - (k)) & STATE_HISTORY_MASK)])

#define check_rollback_request(k)         \
  do {                                    \
    if ((k) & option(Rollback_Requested)) \
    {                                     \
      return;                             \
    }                                     \
  } while (0)

#define satur_add(_n, _i, _th)            \
  do {                                    \
    if ((_n) + (_i) <= (_th))             \
    {                                     \
      (_n) += (_i);                       \
    }                                     \
  } while (0)

#define satur_sub(_n, _i, _th)            \
  do {                                    \
    if ((_n) - (_i) >= (_th))             \
    {                                     \
      (_n) -= (_i);                       \
    }                                     \
  } while (0)

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
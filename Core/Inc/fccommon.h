/*
 * Flight Computer common constants and macros.
 */

#ifndef FC_COMMON
#define FC_COMMON

#include "fctypes.h"
#include "fcconfig.h"
#include "testing.h"


/* DMA constants */

#define SENSOR_BUF_SIZE 8

#define DMA_TIMEOUT_MS 10

#define DMA_BARO_MASK BARO_EXTI_Pin
#define DMA_GYRO_MASK (GYRO_EXTI_1_Pin | GYRO_EXTI_2_Pin)
#define DMA_ACCL_MASK (ACCL_EXTI_1_Pin | ACCL_EXTI_2_Pin)

#define DMA_BARO_OFFSET 0x2u
#define DMA_GYRO_OFFSET 0x1u
#define DMA_ACCL_OFFSET 0x2u


/* Evaluation constants */

#define FLOAT_LOG_PRECISION 8
#define MAX_TRANSITION_SIZE 56

#define MAX_REPORT_SIZE 															\
	(MAX_TRANSITION_SIZE + FLOAT_LOG_PRECISION)

#define GPS_RING_SIZE_MASK (GPS_RING_SIZE - 1)
#define STATE_HISTORY_MASK (STATE_HISTORY - 1)

#define CLEAR_IDX ((fu16)UINT_FAST8_MAX << 8)

#define EVALUATION_STAGED (fu8)(1u << 7)

#define AZ_RAIL_THRES (GRAVITY_SI + EKF_ACCL_RAIL_DIV)


/* Kalman filter constants */

#define TOLERANCE 1e-3f
#define DKF_TOLER 1e-2f

#define NR_ITERATIONS 2

#define DKF_STATE 4
#define DKF_MEASM 3

#define EKF_STATE 6
#define EKF_MEASM 6
#define EKF_OMEGA 4

#define DKF_STATE_SQ (DKF_STATE * DKF_STATE)
#define DKF_MEASM_SQ (DKF_MEASM * DKF_MEASM)

#define EKF_STATE_SQ (EKF_STATE * EKF_STATE)
#define EKF_MEASM_SQ (EKF_MEASM * EKF_MEASM)
#define EKF_OMEGA_SQ (EKF_OMEGA * EKF_OMEGA)

#define DKF_ST_ME (DKF_STATE * DKF_MEASM)
#define EKF_ST_ME (EKF_STATE * EKF_STATE)

#define DKF_PREDICT_BYTES fbyte(DKF_STATE_SQ * 3)
#define DKF_UPDATE_BYTES  fbyte(DKF_MEASM_SQ * 2                \
                                 + DKF_ST_ME * 3)

#define EKF_PREDICT_BYTES fbyte(EKF_STATE_SQ * 3)
#define EKF_UPDATE_BYTES  fbyte(EKF_STATE_SQ * 3                \
                                 + EKF_STATE * 3)

#define MAX_STATE maxd(EKF_STATE, DKF_STATE)
#define MAX_MEASM maxd(EKF_MEASM, DKF_MEASM)

#define LARGEST_POOL maxq(DKF_PREDICT_BYTES, DKF_UPDATE_BYTES,  \
                          EKF_PREDICT_BYTES, EKF_UPDATE_BYTES)

#ifdef PARALLEL_PREDICT_UPDATE

#define KFP_OVERHEAD (4 * sizeof(size_t))

#define KF_POOL_USED maxd(DKF_PREDICT_BYTES + DKF_UPDATE_BYTES, \
                          EKF_PREDICT_BYTES + EKF_UPDATE_BYTES)

#else

#define KFP_OVERHEAD 0
#define KF_POOL_USED LARGEST_POOL

#endif /* PARALLEL_PREDICT_UPDATE */

#define KF_POOL_SIZE (KF_POOL_USED + KFP_OVERHEAD)


/* Recovery constants */

#define MAX_THRESHOLD 0x3FFu
#define FC_MSG_Q_SIZE 8

#define CONFIRM_LAUNCH 0x01
#define CONFIRM_POSTINIT 0x02

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

#define fsec(ms) ((float)(ms) * 0.001f)
#define fbyte(f) ((f) * sizeof(float))

#define ekf_view(vec) (float *)((void *)(vec) + sizeof(kf_gps))
#define dkf_view(vec) (float *)((void *)(vec) + sizeof(float))

#define deg(rad) ((rad) * 180.0f / PI)
#define rad(deg) ((deg) * PI / 180.0f)

#define vnorm2(x, y, res) fvsqrt((x)*(x) + (y)*(y), (res))
#define inorm4(w, x, y, z) invsqrtf((w)*(w) + (x)*(x) + \
                                    (y)*(y) + (z)*(z))

#define within(expr, bound)                               \
  (fabsf((float)(expr)) <= (bound))

#define proxim_lat(k)                                     \
  within((k) - LAUNCH_SITE_LAT, GPS_TOLER)

#define proxim_lon(k)                                     \
  within((k) - LAUNCH_SITE_LON, GPS_TOLER)

/* Go 'k' state vectors back, 0 to get the current vector.
 */
#define svec(k) (sv[(((sm.idx) - (k)) & STATE_HISTORY_MASK)])

#define check_rollback_request(k)             \
  do {                                        \
    if ((k) & option(Rollback_Requested))     \
    {                                         \
      return;                                 \
    }                                         \
  } while (0)

#define satur_incr(_n, _th)                   \
  ((_n) + 1 <= (_th) ? ++(_n) : (_th))

#define satur_decr(_n, _th)                   \
  ((_n) - 1 >= (_th) ? --(_n) : (_th))

#define try_init_sensor(_fn, _ctr, _sn)       \
  do {                                        \
    fu8 k = 0;                                \
    for (; k < SENSOR_REINIT_ATTEMPTS &&      \
           (_fn) != HAL_OK; ++k)              \
           ;                                  \
    if (k >= SENSOR_REINIT_ATTEMPTS)          \
    {                                         \
      (_ctr) += (_sn);                        \
    }                                         \
  } while (0)

#define kf_clear_shared_buffers()                   \
  do {                                              \
    memset(&imedsv, 0, sizeof imedsv);              \
    memset(kf.P_stacov, 0, sizeof kf.P_stacov);     \
    memset(kf.Q_procno, 0, sizeof kf.Q_procno);     \
    memset(kf.A_genpur, 0, sizeof kf.A_genpur);     \
    memset(kf.R_measno, 0, sizeof kf.R_measno);     \
    memset(kf.H_measjc, 0, sizeof kf.H_measjc);     \
  } while (0)

#define mx_scale(src, scl, dst)                     \
  math_call(matrix_scl, src, scl, dst)
#define mx_chol_l(src, dst)                         \
  math_call(chol_lotri, src, dst)
#define mx_inverse(src, dst)                        \
  math_call(matrix_inv, src, dst)
#define mx_transpose(src, dst)                      \
  math_call(mtranspose, src, dst)
#define mx_mul(srca, srcb, dst)                     \
  math_call(matrix_mul, srca, srcb, dst)
#define mx_add(srca, srcb, dst)                     \
  math_call(matrix_add, srca, srcb, dst)
#define mx_sub(srca, srcb, dst)                     \
  math_call(matrix_sub, srca, srcb, dst)


#endif /* FC_COMMON */
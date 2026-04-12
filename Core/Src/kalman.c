/*
 * Kalman Filters
 *
 * This file includes implementations of:
 *   - Extended (ascent) Kalman filter,
 *	 - Regular (descent) Kalman filter,
 *   - Covariance initialization functions,
 *	 - Domain-specifc math functions.
 *
 * The entirety of logic in this file is executed within
 * the context of either Distribution or Evaluation task,
 * which invoke parts of one of two filters based on the
 * flight state and global run time configuration.
 *
 * Both KF implementations share the same buffers, the size
 * of which equals to the largest demanded size among the
 * two filters. When the filters are switched, the buffers
 * are cleared and set to the default values expected by
 * the newly selected filter.
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"


TX_BYTE_POOL kfpool;

static float P[L][L] = {0};
static float Q[L][L] = {0};
static float A[L][L] = {0};
static float R[M][M] = {0};
static float H[M][L] = {0};

static matrix mxp = {0, 0, &P[0][0]};
static matrix mxq = {0, 0, &Q[0][0]};
static matrix mxa = {0, 0, &A[0][0]};
static matrix mxr = {0, 0, &R[0][0]};
static matrix mxh = {0, 0, &H[0][0]};


/*
 * KF-specific wrapper around TX heap manager.
 */
static float *_kfalloc(size_t size, bool user)
{
  assert(size != 0 && (size & (sizeof(float) - 1)) == 0);

  UINT st;
  void *ptr;

  st = tx_byte_allocate(&kfpool, &ptr, size, TX_NO_WAIT);

  if (st == TX_NO_MEMORY && user)
  {
    tx_byte_pool_delete(&kfpool);
    tx_byte_pool_create(&kfpool, "EX P", kfpool_buf, KF_POOL_SIZE);
    
    _kfalloc(size, false);
  }

  return st == TX_SUCCESS ? ptr : kfpool_buf;
}

static inline float *kfalloc(size_t size)
{
  return _kfalloc(size, true);
}

static inline void kffree(float *ptr)
{
  tx_byte_release((void *)ptr);
}


/*
 * Inverse square root from Quake 3 Arena.
 */
static inline constexpr float invsqrtf(float x)
{
  f32u k = {.f = x};

  k.d = 0x5f3759df - (k.d >> 1);

  for (fu8 i = 0; i < NR_ITERATIONS; ++i)
  {
    k.f *= 1.5f - 0.5f * x * k.f * k.f;
  }

  return k.f;
}

/*
 * Converts euler angles to quaternions.
 */
static inline void euler_to_quat(const eul *ang)
{

}

/*
 * Converts sensor averages to euler angles.
 */
void accel_to_quaternion(const f_xyz *accl)
{
  
}

/*
 * Transforms state vector into sensor measurement. (??)
 */
static inline void
kf_measm(kf_svec *restrict vec, measm *restrict out)
{

}


/*
 * Sets descent filter values in shared buffers.
 */
void descent_initialize(void)
{
  memset(P, 0, sizeof P);
  memset(Q, 0, sizeof Q);
  memset(A, 0, sizeof A);
  memset(R, 0, sizeof R);
  memset(H, 0, sizeof H);

  for (fu8 i = 0; i < DKF_MEASM; ++i)
  {
    A[i][i] = H[i][i] = 1.0f;
  }

  A[DKF_STATE - 1][DKF_STATE - 1] = 1.0f;
  Q[0][0] = Q[1][1] = TOLERANCE;
  Q[2][2] = Q[3][3] = DKF_TOLER;
  R[0][0] = R[1][1] = DKF_GPS_TRUST;
  R[2][2] = DKF_BARO_TRUST;

  mxp.numRows = mxp.numCols = DKF_STATE;
  mxq.numRows = mxq.numCols = DKF_STATE;
  mxa.numCols = mxa.numRows = DKF_STATE;
  mxr.numCols = mxr.numRows = DKF_MEASM;
  mxh.numRows = DKF_MEASM;
  mxh.numCols = DKF_STATE;

  irq_off(Gyro_EXTI_1);
/*irq_off(Gyro_EXTI_2);   not used for IREC 2026 */
  irq_off(Accl_EXTI_1);
/*irq_off(Accl_EXTI_2);   not used for IREC 2026 */

  fc_msg toggle = Using_Ascent_KF;

  if (load(&g_conf, Acq) & option(Defer_Baro_Fallback))
  {
    toggle |= Defer_Baro_Fallback;
    fetch_or(&g_conf, option(Monitor_Altitude | Validate_Measms), Rlx);

    fc_msg cmd = fc_mask(Reinit_Barometer);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }

  timer_update(DescentKF);
  fetch_and(&g_conf, ~option(toggle), Rel);
}

/*
 * Note: overwrites previous state vector in place.
 * Prerequisites: None.
 */
void descent_predict(const float dt)
{
  matrix presv = {DKF_STATE, 1, dkf_view(&svec(1))};

  float *mk = kfalloc(DKF_PREDICT_BYTES);

  memcpy(mk, dkf_view(&svec(1)), DKF_STATE);

  matrix mxat = {DKF_STATE, 1, mk};
  matrix mxap = {DKF_STATE, DKF_STATE, mxat.pData + DKF_STATE_SQ};
  matrix mxfi = {DKF_STATE, DKF_STATE, mxap.pData + DKF_STATE_SQ};

	matrix_mul(&mxa, &mxat, &presv);
  
  mxat.numCols = DKF_STATE;
  mtranspose(&mxa, &mxat);

  matrix_mul(&mxa, &presv, &mxap);
  matrix_mul(&mxap, &mxat, &mxfi);
  matrix_add(&mxfi, &mxq, &mxp);

  kffree(mk);
}

/*
 * This function partitions allocated buffer into blocks,
 * then reuses them, merging when recessary. If names and
 * transitions (->) don't make sense, refer to MATLAB code.
 * Prerequisites: Barometer or GPS.
 */
void descent_update(const float dt)
{
  sweetbench_start(4, 50, true);

  matrix cursv = {DKF_STATE, 1, dkf_view(&svec(0))};
  matrix presv = {DKF_STATE, 1, dkf_view(&svec(1))};
  matrix measm = {DKF_MEASM, 1, (float *)&meas.gps};

  float *mk = kfalloc(DKF_UPDATE_BYTES);

  matrix mxht   = {DKF_STATE, DKF_MEASM, mk};
  matrix mxhp   = {DKF_MEASM, DKF_STATE, mk + DKF_ST_ME};
  matrix mxhpht = {DKF_MEASM, DKF_MEASM, mxhp.pData + DKF_ST_ME};
  matrix mxs    = {DKF_MEASM, DKF_MEASM, mxhpht.pData + DKF_MEASM_SQ};
  matrix mxpht  = {DKF_MEASM, DKF_STATE, mxs.pData + DKF_MEASM_SQ};

  mtranspose(&mxh, &mxht);
  matrix_mul(&mxh, &mxp, &mxhp);
  matrix_mul(&mxhp, &mxht, &mxhpht);
  matrix_add(&mxhpht, &mxr, &mxs);
  matrix_inv(&mxs, &mxhpht);
  matrix_mul(&mxp, &mxht, &mxpht);
  matrix_mul(&mxpht, &mxhpht, &mxht);

  /* mxht -> "mxk"; mxs -> "mxhx"; mxhpht -> "mxzhx";
   * mxhp -> "mxkzhx"
   */

  mxs.numCols = mxhp.numCols = 1;
  mxhp.numRows = DKF_STATE;

  matrix_mul(&mxh, &presv, &mxs);
  matrix_sub(&measm, &mxs, &mxhpht);
  matrix_mul(&mxht, &mxhpht, &mxhp);
  matrix_add(&presv, &mxhp, &cursv);

  /* Merge mxhp + mxhpht; mxs + mxpht.
   * mxhp -> "mxkh" -> "mxp_f"; mxs -> "mxkhp"
   */

  mxhp.numRows = mxhp.numCols = DKF_STATE;
  mxs.numRows = mxs.numCols = DKF_STATE;

  matrix_mul(&mxht, &mxh, &mxhp);
  matrix_mul(&mxhp, &mxp, &mxs);
  matrix_sub(&mxp, &mxs, &mxhp);

  memcpy(P, mxhp.pData, DKF_STATE_SQ);

  kffree(mk);

  sweetbench_catch(4);
}


/*
 * Sets ascent filter values in shared buffers.
 */
void ascent_initialize(void) 
{
  memset(P, 0, sizeof P);
  memset(Q, 0, sizeof Q);
  memset(A, 0, sizeof A);
  memset(R, 0, sizeof R);
  memset(H, 0, sizeof H);

  // TODO

  mxp.numRows = mxp.numCols = EKF_STATE;
  mxq.numRows = mxq.numCols = EKF_STATE;
  mxa.numCols = mxa.numRows = EKF_STATE;
  mxr.numCols = mxr.numRows = EKF_MEASM;
  mxh.numRows = EKF_MEASM;
  mxh.numCols = EKF_STATE;

  if (!(load(&g_conf, Acq) & option(Using_Ascent_KF)))
  {
    /* Switched to Ascent mid-flight, for some reason, but OK.
     */
    fc_msg cmd = fc_mask(Reinit_IMU);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }

  timer_update(AscentKF);
  fetch_or(&g_conf, option(Using_Ascent_KF), Rel);
}

/*
 * Predict step of the Ascent filter.
 * Prerequisites: IMU.
 */
void ascent_predict(const float dt)
{

}

/*
 * Update step of the Ascent filter.
 * Prerequisites: Barometer.
 */
void ascent_update(const float dt)
{
  sweetbench_start(3, 50, true);

  sweetbench_catch(3);
}
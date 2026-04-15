/*
 * Kalman Filters
 *
 * Regular (descent) KF
 * Extended error state (ascent) KF
 * Initialization functions
 * Domain-specifc math helpers
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


#ifdef PARALLEL_PREDICT_UPDATE
TX_BYTE_POOL kfpool;
#endif

static quat qv = {0};

static float P_stacov[MAX_STATE][MAX_STATE] = {0};
static float Q_procno[MAX_STATE][MAX_STATE] = {0};
static float A_genpur[MAX_STATE][MAX_STATE] = {0};
static float R_measno[MAX_MEASM][MAX_MEASM] = {0};
static float H_measjc[MAX_MEASM][MAX_STATE] = {0};

static matrix g_mxp = {0, 0, &P_stacov[0][0]};
static matrix g_mxq = {0, 0, &Q_procno[0][0]};
static matrix g_mxa = {0, 0, &A_genpur[0][0]};
static matrix g_mxr = {0, 0, &R_measno[0][0]};
static matrix g_mxh = {0, 0, &H_measjc[0][0]};


/*
 * Fault-tolerant (fault-friendly) KF-specific wrapper
 * around TX heap manager.
 */
static inline float *kfalloc(size_t size)
{
#ifdef PARALLEL_PREDICT_UPDATE
  if (size == 0 || size > LARGEST_POOL)
  {
    size = LARGEST_POOL;
  }

  UINT st, old_pt;
  void *ptr;

  request: do
  {
    st = tx_byte_allocate(&kfpool, &ptr, size, TX_WAIT_FOREVER);
  }
  while (st == TX_WAIT_ABORTED);

  if (st != TX_SUCCESS)
  {
    TX_THREAD *curr = tx_thread_identify();

    if (!curr)
    {
      return NULL; /* Friendliness has its limits */
    }

    tx_thread_preemption_change(curr, 1, &old_pt);
    tx_thread_relinquish();

    goto request;
  }

  return (float *)ptr;

#else
  return (float *)kfpool_buf;

#endif /* PARALLEL_PREDICT_UPDATE */
}

static inline void kffree(float *ptr)
{
#ifdef PARALLEL_PREDICT_UPDATE
  tx_byte_release((void *)ptr);

#else
  return;

#endif /* PARALLEL_PREDICT_UPDATE */
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
 * Converts euler angles to quaternions, stored globally.
 */
static inline void euler_to_quat(eul *ang)
{
  ang->phi *= 0.5f;
  ang->theta *= 0.5f;
  ang->psi *= 0.5f;

  float cosphi = fcos(ang->phi);
  float sinphi = fsin(ang->phi);
  float costhe = fcos(ang->theta);
  float sinthe = fsin(ang->theta);
  float cospsi = fcos(ang->psi);
  float sinpsi = fsin(ang->psi);

  float costhepsi = costhe * cospsi;
  float sinthepsi = sinthe * sinpsi;
  float sinthecospsi = sinthe * cospsi;
  float costhesinpsi = costhe * sinpsi;

  qv.q0 = cosphi * costhepsi + sinphi * sinthepsi;
  qv.rho1 = sinphi * costhepsi - cosphi * sinthepsi;
  qv.rho2 = cosphi * sinthecospsi + sinphi * costhesinpsi;
  qv.rho3 = cosphi + costhesinpsi - sinphi * sinthecospsi;

  float norm = inorm4(qv.q0, qv.rho1, qv.rho2, qv.rho3);

  qv.q0 *= norm;
  qv.rho1 *= norm;
  qv.rho2 *= norm;
  qv.rho3 *= norm;

  // TODO log
}

/*
 * ang.psi used to temporarily hold the norm.
 */
void accel_to_quaternion(const f_xyz *accl)
{
  eul ang;

  fatan2(accl->y, accl->z, &ang.phi);
  vnorm2(accl->y, accl->z, &ang.psi);
  fatan2(-accl->x, ang.psi, &ang.theta);
  ang.psi = 0;

  // TODO log in deg

  euler_to_quat(&ang);
}


/*
 * In DKF, "A_genpur" will serve A (state transition).
 */
void descent_initialize(void)
{
  kf_clear_shared_buffers();

  for (fu8 i = 0; i < DKF_MEASM; ++i)
  {
    H_measjc[i][i] = 1.0f;
  }
  for (fu8 i = 0; i < DKF_STATE; ++i)
  {
    A_genpur[i][i] = 1.0f;
  }

  Q_procno[0][0] = Q_procno[1][1] = TOLERANCE;
  Q_procno[2][2] = Q_procno[3][3] = DKF_TOLER;
  R_measno[0][0] = R_measno[1][1] = DKF_GPS_TRUST;
  R_measno[2][2] = DKF_BARO_TRUST;

  g_mxp.numRows = g_mxp.numCols = DKF_STATE;
  g_mxq.numRows = g_mxq.numCols = DKF_STATE;
  g_mxa.numCols = g_mxa.numRows = DKF_STATE;
  g_mxr.numCols = g_mxr.numRows = DKF_MEASM;
  g_mxh.numRows = DKF_MEASM;
  g_mxh.numCols = DKF_STATE;

  irq_off(Gyro_EXTI_1);
/*irq_off(Gyro_EXTI_2);   not used for IREC 2026 */
  irq_off(Accl_EXTI_1);
/*irq_off(Accl_EXTI_2);   not used for IREC 2026 */

  fu32 conf = load(&g_conf, Acq);
  fc_msg toggle = Using_Ascent_KF;

  if (conf & option(Defer_Baro_Fallback))
  {
    toggle |= Defer_Baro_Fallback;
    conf = Monitor_Altitude | Validate_Measms;

    fetch_or(&g_conf, option(conf), Rlx);

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
  A_genpur[DKF_STATE -  1][DKF_MEASM - 1] = dt;

  matrix presv = {DKF_STATE, 1, dkf_view(&svec(1))};

  float *mk = kfalloc(DKF_PREDICT_BYTES);

  memcpy(mk, dkf_view(&svec(1)), DKF_STATE);

  matrix mxat = {DKF_STATE, 1, mk};
  matrix mxap = {DKF_STATE, DKF_STATE, mxat.pData + DKF_STATE_SQ};
  matrix mxfi = {DKF_STATE, DKF_STATE, mxap.pData + DKF_STATE_SQ};

	matrix_mul(&g_mxa, &mxat, &presv);
  
  mxat.numCols = DKF_STATE;
  mtranspose(&g_mxa, &mxat);

  matrix_mul(&g_mxa, &presv, &mxap);
  matrix_mul(&mxap, &mxat, &mxfi);
  matrix_add(&mxfi, &g_mxq, &g_mxp);

  kffree(mk);
}

/*
 * Note: allocated buffer is partitioned into blocks,
 * which are coalesced when necessary to hold bigger data.
 * Prerequisites: Barometer or GPS.
 */
void descent_update(void)
{
  sweetbench_start(4, 50, true);

  matrix cursv = {DKF_STATE, 1, dkf_view(&svec(0))};
  matrix presv = {DKF_STATE, 1, dkf_view(&svec(1))};
  matrix measm = {DKF_MEASM, 1, (float *)&meas.gps};

  float *mk = kfalloc(DKF_UPDATE_BYTES);

  matrix mxht   = {DKF_STATE, DKF_MEASM, mk};
  matrix mxhp   = {DKF_MEASM, DKF_STATE, mxht.pData + DKF_ST_ME};
  matrix mxhpht = {DKF_MEASM, DKF_MEASM, mxhp.pData + DKF_ST_ME};
  matrix mxs    = {DKF_MEASM, DKF_MEASM, mxhpht.pData + DKF_MEASM_SQ};
  matrix mxpht  = {DKF_MEASM, DKF_STATE, mxs.pData + DKF_MEASM_SQ};

  mtranspose(&g_mxh, &mxht);
  matrix_mul(&g_mxh, &g_mxp, &mxhp);
  matrix_mul(&mxhp, &mxht, &mxhpht);
  matrix_add(&mxhpht, &g_mxr, &mxs);
  matrix_inv(&mxs, &mxhpht);
  matrix_mul(&g_mxp, &mxht, &mxpht);
  matrix_mul(&mxpht, &mxhpht, &mxht);

  /* mxht -> "mxk"; mxs -> "mxhx"; mxhpht -> "mxzhx";
   * mxhp -> "mxkzhx"
   */

  mxs.numCols = mxhp.numCols = 1;
  mxhp.numRows = DKF_STATE;

  matrix_mul(&g_mxh, &presv, &mxs);
  matrix_sub(&measm, &mxs, &mxhpht);
  matrix_mul(&mxht, &mxhpht, &mxhp);
  matrix_add(&presv, &mxhp, &cursv);

  /* Merge mxhp + mxhpht; mxs + mxpht.
   * mxhp -> "mxkh" -> "mxp_f"; mxs -> "mxkhp"
   */

  mxhp.numRows = mxhp.numCols = DKF_STATE;
  mxs.numRows = mxs.numCols = DKF_STATE;

  matrix_mul(&mxht, &g_mxh, &mxhp);
  matrix_mul(&mxhp, &g_mxp, &mxs);
  matrix_sub(&g_mxp, &mxs, &mxhp);

  memcpy(P_stacov, mxhp.pData, DKF_STATE_SQ);

  kffree(mk);

  sweetbench_catch(4);
}


/* 
 * In EKF, "R_measno" will serve F (transition Jacobian),
 * "A_genpur" will serve Omega (quaternion propagation).
 */
void ascent_initialize(void) 
{
  kf_clear_shared_buffers();

  for (fu8 i = 0; i < EKF_STATE; ++i)
  {
    R_measno[i][i] = 1.0f;
  }

  H_measjc[0][0] = 1.0f;

  Q_procno[0][0] = 1e-5f;
  Q_procno[1][1] = 1e-4f;
  Q_procno[2][2] = 1e-7f;
  Q_procno[3][3] = Q_procno[4][4] = Q_procno[5][5] = 1e-10f;

  g_mxp.numRows = g_mxp.numCols = EKF_STATE;
  g_mxq.numRows = g_mxq.numCols = EKF_STATE;
  g_mxa.numCols = g_mxa.numRows = EKF_OMEGA;
  g_mxr.numCols = g_mxr.numRows = EKF_MEASM;
  g_mxh.numRows = EKF_STATE;
  g_mxh.numCols = 1;

  if (!(load(&g_conf, Acq) & option(Using_Ascent_KF)))
  {
    /* Switched to Ascent mid-flight or user messed with
     * FC_DEFAULTS. */

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
void ascent_predict(const float dt, fu32 conf)
{
  sweetbench_start(3, 50, true);

  f_xyz w = meas.gyro;

  w.x -= svec(1).bias.gx;
  w.y -= svec(1).bias.gy;
  w.z -= svec(1).bias.gz;

  A_genpur[0][1] = A_genpur[3][2] = -w.x;
  A_genpur[1][0] = A_genpur[2][3] = w.x;
  A_genpur[0][2] = A_genpur[1][3] = -w.y;
  A_genpur[2][0] = A_genpur[3][1] = w.y;
  A_genpur[2][1] = A_genpur[0][3] = -w.z;
  A_genpur[1][2] = A_genpur[3][0] = w.z;

  float *mk = kfalloc(EKF_PREDICT_BYTES);



  svec(0).alt = svec(1).alt + dt * svec(1).vel;

  if (conf & option(Launch_Requested))
  {
    // ???
  }

  kffree(mk);

  sweetbench_catch(3);
}

/*
 * Update step of the Ascent filter.
 * Prerequisites: Barometer.
 */
void ascent_update(void)
{

}
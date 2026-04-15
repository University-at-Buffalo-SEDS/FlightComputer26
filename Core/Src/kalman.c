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

static kf_buf kf = {
  {0, 0, &kf.P_stacov[0][0]}, {0},
  {0, 0, &kf.Q_procno[0][0]}, {0},
  {0, 0, &kf.A_genpur[0][0]}, {0},
  {0, 0, &kf.R_measno[0][0]}, {0},
  {0, 0, &kf.H_measjc[0][0]}, {0}
};


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
 * Calculate offset from previous heap-allocated matrix.
 */
static inline constexpr float *mxoff(const matrix *prev)
{
  return prev->pData + prev->numRows * prev->numCols;
}


/*
 * In DKF, "A_genpur" will serve A (state transition).
 */
void descent_initialize(void)
{
  kf_clear_shared_buffers();

  for (fu8 i = 0; i < DKF_MEASM; ++i)
  {
    kf.H_measjc[i][i] = 1.0f;
  }
  for (fu8 i = 0; i < DKF_STATE; ++i)
  {
    kf.A_genpur[i][i] = 1.0f;
  }

  kf.Q_procno[0][0] = kf.Q_procno[1][1] = TOLERANCE;
  kf.Q_procno[2][2] = kf.Q_procno[3][3] = DKF_TOLER;
  kf.R_measno[0][0] = kf.R_measno[1][1] = DKF_GPS_TRUST;
  kf.R_measno[2][2] = DKF_BARO_TRUST;

  kf.mxp.numRows = kf.mxp.numCols = DKF_STATE;
  kf.mxq.numRows = kf.mxq.numCols = DKF_STATE;
  kf.mxa.numCols = kf.mxa.numRows = DKF_STATE;
  kf.mxr.numCols = kf.mxr.numRows = DKF_MEASM;
  kf.mxh.numRows = DKF_MEASM;
  kf.mxh.numCols = DKF_STATE;

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
  kf.A_genpur[DKF_STATE -  1][DKF_MEASM - 1] = dt;

  float *mk = kfalloc(DKF_PREDICT_BYTES);

  memcpy(mk, dkf_view(&svec(1)), DKF_STATE);

  matrix mxat = {DKF_STATE, 1, mk};
  matrix mxap = {DKF_STATE, DKF_STATE, mxoff(&mxat)};
  matrix mxfi = {DKF_STATE, DKF_STATE, mxoff(&mxap)};

	matvec_mul(&kf.mxa, mxat.pData, dkf_view(&svec(1)));
  
  mxat.numCols = DKF_STATE;
  mtranspose(&kf.mxa, &mxat);

  matrix_mul(&kf.mxa, &kf.mxp, &mxap);
  matrix_mul(&mxap, &mxat, &mxfi);
  matrix_add(&mxfi, &kf.mxq, &kf.mxp);

  kffree(mk);
}

/*
 * Allocated buffer is partitioned into blocks,
 * coalesced when necessary to hold bigger matrices.
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
  matrix mxhp   = {DKF_MEASM, DKF_STATE, mxoff(&mxht)};
  matrix mxhpht = {DKF_MEASM, DKF_MEASM, mxoff(&mxhp)};
  matrix mxs    = {DKF_MEASM, DKF_MEASM, mxoff(&mxhpht)};
  matrix mxpht  = {DKF_MEASM, DKF_STATE, mxoff(&mxs)};

  mtranspose(&kf.mxh, &mxht);
  matrix_mul(&kf.mxh, &kf.mxp, &mxhp);
  matrix_mul(&mxhp, &mxht, &mxhpht);
  matrix_add(&mxhpht, &kf.mxr, &mxs);
  matrix_inv(&mxs, &mxhpht);
  matrix_mul(&kf.mxp, &mxht, &mxpht);
  matrix_mul(&mxpht, &mxhpht, &mxht);

  /* mxht -> "mxk"; mxs -> "mxhx"; mxhpht -> "mxzhx";
   * mxhp -> "mxkzhx"
   */

  mxs.numCols = mxhp.numCols = 1;
  mxhp.numRows = DKF_STATE;

  matvec_mul(&kf.mxh, presv.pData, mxs.pData);
  matrix_sub(&measm, &mxs, &mxhpht);
  matvec_mul(&mxht, mxhpht.pData, mxhp.pData);
  matrix_add(&presv, &mxhp, &cursv);

  /* Coalesce mxhp + mxhpht; mxs + mxpht.
   * mxhp -> "mxkh" -> "mxp_f"; mxs -> "mxkhp"
   */

  mxhp.numRows = mxhp.numCols = DKF_STATE;
  mxs.numRows = mxs.numCols = DKF_STATE;

  matrix_mul(&mxht, &kf.mxh, &mxhp);
  matrix_mul(&mxhp, &kf.mxp, &mxs);
  matrix_sub(&kf.mxp, &mxs, &kf.mxp);

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
    kf.R_measno[i][i] = 1.0f;
  }

  kf.H_measjc[0][0] = 1.0f;

  kf.Q_procno[0][0] = 1e-5f;
  kf.Q_procno[1][1] = 1e-4f;
  kf.Q_procno[2][2] = 1e-7f;
  kf.Q_procno[3][3] = kf.Q_procno[4][4] = kf.Q_procno[5][5] = 1e-10f;

  kf.mxp.numRows = kf.mxp.numCols = EKF_STATE;
  kf.mxq.numRows = kf.mxq.numCols = EKF_STATE;
  kf.mxa.numCols = kf.mxa.numRows = EKF_OMEGA;
  kf.mxr.numCols = kf.mxr.numRows = EKF_STATE;
  kf.mxh.numRows = 1;
  kf.mxh.numCols = EKF_MEASM;

  for (fu8 k = 0; k < STATE_HISTORY; ++k)
  {
    svec(k).bias.gx = EKF_BIAS_GYRO_X;
    svec(k).bias.gy = EKF_BIAS_GYRO_Y;
    svec(k).bias.gz = EKF_BIAS_GYRO_Z;
    svec(k).bias.az = EKF_BIAS_ACCL_Z;
  }

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
 * Bias is compile-time, set during initialization.
 * Prerequisites: IMU.
 */
void ascent_predict(const float dt, fu32 conf)
{
  sweetbench_start(3, 50, true);

  fc_lock(&meas_locks[0]);

  f_xyz w = meas.gyro;
  f_xyz a = meas.accl;

  fc_unlock(&meas_locks[0]);

  w.x -= svec(1).bias.gx;
  w.y -= svec(1).bias.gy;
  w.z -= svec(1).bias.gz;

  kf.A_genpur[0][1] = kf.A_genpur[3][2] = -w.x;
  kf.A_genpur[1][0] = kf.A_genpur[2][3] = w.x;
  kf.A_genpur[0][2] = kf.A_genpur[1][3] = -w.y;
  kf.A_genpur[2][0] = kf.A_genpur[3][1] = w.y;
  kf.A_genpur[2][1] = kf.A_genpur[0][3] = -w.z;
  kf.A_genpur[1][2] = kf.A_genpur[3][0] = w.z;

  float *mk = kfalloc(EKF_PREDICT_BYTES);
  const float qv_scale = 0.5f + dt;

  kf.R_measno[0][1] = dt;

  matrix veqv = {EKF_OMEGA, 1, &qv.q0};
  matrix veqm = {EKF_OMEGA, 1, mk};
  matrix veqk = {EKF_OMEGA, 1, mxoff(&veqm)};
  matrix mxpq = {EKF_STATE, EKF_STATE, NULL};

  matvec_mul(&kf.mxa, &qv.q0, veqm.pData);
  matrix_scl(&veqm, qv_scale, &veqm);

  matrix_add(&veqm, &veqv, &veqm);
  matvec_mul(&kf.mxa, veqm.pData, veqk.pData);

  matrix_scl(&veqv, 0.5f * qv_scale, &veqv);
  matrix_add(&veqv, &veqk, &veqv);

  float inormq = inorm4(qv.q0, qv.rho1, qv.rho2, qv.rho3);

  qv.q0   *= inormq;
  qv.rho1 *= inormq;
  qv.rho2 *= inormq;
  qv.rho3 *= inormq;

  float r13 = 2.0f * (qv.rho1*qv.rho3 - qv.q0*qv.rho2);
  float r23 = 2.0f * (qv.rho2*qv.rho3 + qv.q0*qv.rho1);
  float r33 = 1.0f - 2.0f * (qv.rho1*qv.rho1 + qv.rho2*qv.rho2);

  float a_vert = (r13 * a.x + r23 * a.y + r33 * a.z);

  svec(0).alt = svec(1).alt + dt * svec(1).vel;
  svec(0).vel = svec(1).vel + dt * (
    sm.flight >= Launch || a.z < AZ_RAIL_THRES
        ? a_vert - svec(1).bias.az
        : a_vert - svec(1).bias.az - GRAVITY_SI
  );

  /* Coalesce veqm, veqk; veqm -> mxfp, veqk -> mxft
   */

  veqm.numRows = veqm.numCols = EKF_STATE;
  veqk.numRows = veqk.numCols = EKF_STATE;
  veqm.pData = mk;
  veqk.pData = mxoff(&veqm);
  mxpq.pData = mxoff(&veqk);
  
  mtranspose(&kf.mxr, &veqk);
  matrix_mul(&kf.mxr, &kf.mxp, &veqm);
  matrix_mul(&veqm, &veqk, &mxpq);
  matrix_add(&mxpq, &kf.mxq, &kf.mxp);

  kffree(mk);

  sweetbench_catch(3);
}

/*
 * Update step of the Ascent filter.
 * Prerequisites: Barometer.
 */
void ascent_update(void)
{
  fc_lock(&meas_locks[1]);

  const float baroz_innv = meas.baro.alt - svec(1).alt;

  fc_unlock(&meas_locks[1]);

  
}
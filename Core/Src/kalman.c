/* Core/Src/kalman.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "testing.h"


#ifdef PARALLEL_PREDICT_UPDATE
TX_BYTE_POOL kfpool;
#endif

quat qv = {0};
kf_svec imedsv = {0};
static kf_matrix kf = {0};

static float EKF_P[EKF_STATE][EKF_STATE] = {0};
static float EKF_Q[EKF_STATE][EKF_STATE] = {0};
static float EKF_O[EKF_OMEGA][EKF_OMEGA] = {0};
static float EKF_F[EKF_STATE][EKF_STATE] = {0};
static float EKF_H[1][EKF_MEASM] = {0};

static float DKF_P[DKF_STATE][DKF_STATE] = {0};
static float DKF_Q[DKF_STATE][DKF_STATE] = {0};
static float DKF_A[DKF_STATE][DKF_STATE] = {0};
static float DKF_R[DKF_MEASM][DKF_MEASM] = {0};
static float DKF_H[DKF_MEASM][DKF_STATE] = {0};


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
    st = tx_byte_allocate(&kfpool, &ptr, size, 5);
  }
  while (st == TX_WAIT_ABORTED);

  if (st != TX_SUCCESS)
  {
    if (tx_thread_identify() == TX_NULL)
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

static inline void clear_shared_buffers(bool ascent)
{
  memset(&imedsv, 0, sizeof imedsv);

  if (ascent)
  {
    memset(EKF_P, 0, sizeof EKF_P);
    memset(EKF_Q, 0, sizeof EKF_Q);
    memset(EKF_O, 0, sizeof EKF_O);
    memset(EKF_F, 0, sizeof EKF_F);
    memset(EKF_H, 0, sizeof EKF_H);
  }
  else
  {
    memset(DKF_P, 0, sizeof DKF_P);
    memset(DKF_Q, 0, sizeof DKF_Q);
    memset(DKF_A, 0, sizeof DKF_A);
    memset(DKF_R, 0, sizeof DKF_R);
    memset(DKF_H, 0, sizeof DKF_H);
  }
}


static inline constexpr float isqrtf_quake(float x)
{
  if (within(x - 1, TOLERANCE))
  {
    return 1.0f;
  }

  union { float f; fu32 d; } alias = {x};

  alias.d = 0x5f3759df - (alias.d >> 1);

  for (fu8 i = 0; i < NR_ITERATIONS; ++i)
  {
    alias.f *= 1.5f - 0.5f * x * alias.f * alias.f;
  }

  return alias.f;
}

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
  qv.rho3 = cosphi * costhesinpsi - sinphi * sinthecospsi;

  float norm = inorm4(qv.q0, qv.rho1, qv.rho2, qv.rho3);

  qv.q0 *= norm;
  qv.rho1 *= norm;
  qv.rho2 *= norm;
  qv.rho3 *= norm;

  float init_quat[EKF_STATE];

  *((quat *)init_quat) = qv;
  init_quat[EKF_STATE - 2] = 0.0f;
  init_quat[EKF_STATE - 1] = 0.0f;

#ifdef SD_AVAILABLE
  sd_append_f32(SEDS_DT_ASCENT_STATE, init_quat, EKF_STATE);
#endif
  log_f32(SEDS_DT_ASCENT_STATE, EKF_STATE, init_quat);
}

void accel_to_quaternion(const f_xyz *accl)
{
  conditional eul ang, rep_in_deg;

  fatan2(accl->y, accl->z, &ang.phi);
  vnorm2(accl->y, accl->z, &ang.psi);
  fatan2(-accl->x, ang.psi, &ang.theta);
  ang.psi = 0;

  rep_in_deg.phi = deg(ang.phi);
  rep_in_deg.theta = deg(ang.theta);
  rep_in_deg.psi = deg(ang.psi);

#ifdef SD_AVAILABLE
  sd_append_f32(SEDS_DT_EULER_ANGLES, (float *)&rep_in_deg, 3);
#endif
  log_f32(SEDS_DT_EULER_ANGLES, 3, &rep_in_deg);

  euler_to_quat(&ang);
}

static inline pure float *mxoff(const matrix *prev)
{
  return prev->pData + prev->numRows * prev->numCols;
}


/* Descent (regular) KF.
 * Implementer's notes:
 *    1. Uses mxa to store A (state transition matrix)
 *    2. Uses imedsv to store x_2 (predict -> update) 
 *    3. Can reuse space within the heap-allocated buffer
 *       and repurpose stack-allocated matrix wrappers.
 *       Comments relate such wrappers to MATLAB names.
 *    4. Prerequisites for Predict: None.
 *    5. Prerequisites for Update:  Barometer or GPS.     */


void descent_initialize(fu32 conf)
{
  clear_shared_buffers(false);

  for (fu8 i = 0; i < DKF_MEASM; ++i)
  {
    DKF_H[i][i] = 1.0f;
  }
  for (fu8 i = 0; i < DKF_STATE; ++i)
  {
    DKF_A[i][i] = 1.0f;
  }

  DKF_P[0][0] = DKF_P[1][1] = DKF_TOLER;
  DKF_P[2][2] = DKF_P[3][3] = 1.0f;

  DKF_Q[0][0] = DKF_Q[1][1] = TOLERANCE;
  DKF_Q[2][2] = DKF_Q[3][3] = DKF_TOLER;

  DKF_R[0][0] = DKF_R[1][1] = DKF_GPS_TRUST;
  DKF_R[2][2] = DKF_BARO_TRUST;

  kf.mxp.numRows = kf.mxp.numCols = DKF_STATE;
  kf.mxq.numRows = kf.mxq.numCols = DKF_STATE;
  kf.mxa.numCols = kf.mxa.numRows = DKF_STATE;
  kf.mxr.numCols = kf.mxr.numRows = DKF_MEASM;
  kf.mxh.numRows = DKF_MEASM;
  kf.mxh.numCols = DKF_STATE;

  kf.mxp.pData = (float *)DKF_P;
  kf.mxq.pData = (float *)DKF_Q;
  kf.mxa.pData = (float *)DKF_A;
  kf.mxr.pData = (float *)DKF_R;
  kf.mxh.pData = (float *)DKF_H;

  fc_msg toggle = Using_Ascent_KF;
  fc_msg cmd = fc_mask(Disable_IMU);
  tx_queue_send(&shared, &cmd, TX_NO_WAIT);

  if (conf & option(Defer_Baro_Fallback))
  {
    toggle |= Defer_Baro_Fallback;
    conf = Monitor_Altitude | Measm_Reports;
    cmd = fc_mask(Reinit_Barometer);

    fetch_or(&g_conf, option(conf), Rlx);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }

  timer_update(DescentKF);
  fetch_and(&g_conf, ~option(toggle), Rel);
}

void descent_predict(const float dt)
{
  DKF_A[DKF_MEASM -  1][DKF_STATE - 1] = dt;

  float *start = kfalloc(DKF_PREDICT_BYTES);

  matrix m_at = {DKF_STATE, DKF_STATE, start};
  matrix m_ap = {DKF_STATE, DKF_STATE, mxoff(&m_at)};
  matrix m_fi = {DKF_STATE, DKF_STATE, mxoff(&m_ap)};

  matvec_mul(&kf.mxa, dkf_view(&svec(1)), dkf_view(&imedsv));
  
  mx_transpose(&kf.mxa, &m_at);

  mx_mul(&kf.mxa, &kf.mxp, &m_ap);
  mx_mul(&m_ap, &m_at, &m_fi);
  mx_add(&m_fi, &kf.mxq, &kf.mxp);

  kffree(start);
}

void descent_update(void)
{
  matrix v_sv0 = {DKF_STATE, 1, dkf_view(&svec(0))};
  matrix v_sv1 = {DKF_STATE, 1, dkf_view(&imedsv)};
  matrix v_msm = {DKF_MEASM, 1, dkf_view(&meas.gps)};

  float *start = kfalloc(DKF_UPDATE_BYTES);

  matrix m_ht   = {DKF_STATE, DKF_MEASM, start};
  matrix m_hp   = {DKF_MEASM, DKF_STATE, mxoff(&m_ht)};
  matrix m_hpht = {DKF_MEASM, DKF_MEASM, mxoff(&m_hp)};
  matrix m_s    = {DKF_MEASM, DKF_MEASM, mxoff(&m_hpht)};
  matrix m_pht  = {DKF_STATE, DKF_MEASM, mxoff(&m_s)};

  mx_transpose(&kf.mxh, &m_ht);
  mx_mul(&kf.mxh, &kf.mxp, &m_hp);
  mx_mul(&m_hp, &m_ht, &m_hpht);
  mx_add(&m_hpht, &kf.mxr, &m_s);

#ifndef MATH_FN_DEBUG
  math_status maybe_singular = mx_inverse(&m_s, &m_hpht);

  if (maybe_singular != ARM_MATH_SUCCESS)
  {
    /* Dead reckon from Predict */
    memcpy(dkf_view(&svec(0)), dkf_view(&imedsv), fbyte(DKF_STATE));
    kffree(start);
    return;
  }
#else
  mx_inverse(&m_s, &m_hpht);
#endif

  mx_mul(&kf.mxp, &m_ht, &m_pht);
  mx_mul(&m_pht, &m_hpht, &m_ht);

  /* m_ht -> "m_k"; m_s -> "m_hx"; m_hpht -> "m_zhx";
   * m_hp -> "m_kzhx"
   */

  m_s.numCols = m_hp.numCols = m_hpht.numCols = 1;
  m_hp.numRows = DKF_STATE;

  matvec_mul(&kf.mxh, v_sv1.pData, m_s.pData);
  mx_sub(&v_msm, &m_s, &m_hpht);
  matvec_mul(&m_ht, m_hpht.pData, m_hp.pData);
  mx_add(&v_sv1, &m_hp, &v_sv0);

  /* Coalesce m_hp + m_hpht; m_s + m_pht.
   * m_hp -> "m_kh" -> "m_p_f"; m_s -> "m_khp"
   */

  m_hp.numRows = m_hp.numCols = DKF_STATE;
  m_s.numRows = m_s.numCols = DKF_STATE;

  mx_mul(&m_ht, &kf.mxh, &m_hp);
  mx_mul(&m_hp, &kf.mxp, &m_s);
  mx_sub(&kf.mxp, &m_s, &kf.mxp);

  kffree(start);
}


/* Ascent (extended) KF.
 * Implementer's notes:
 *    1. Uses mxa to store Omega (quaternion propagation)
 *    2. Uses mxr to store (in Predict) F (transition Jacobian)
                           (in Update)  I (referenced identity)
 *    3. Biases are either constant or user-set at runtime.
 *    4. Can reuse space within the heap-allocated buffer
 *       and repurpose stack-allocated matrix wrappers.
 *       Comments relate such wrappers to MATLAB names.
 *    5. Predict includes currently non-functional midpoint
 *       integration for quaternion (commented out).
 *    6. Prerequisites for Predict: IMU.
 *    7. Prerequisites for Update:  Barometer.               */


void ascent_initialize(fu32 conf) 
{
  clear_shared_buffers(true);

  for (fu8 i = 0; i < EKF_STATE; ++i)
  {
    EKF_F[i][i] = 1.0f;
  }

  EKF_H[0][0] = 1.0f;
  EKF_Q[0][0] = 1e-2;
  EKF_Q[1][1] = 1e-2;
  EKF_Q[2][2] = 1e-7f;
  EKF_Q[3][3] = EKF_Q[4][4] = EKF_Q[5][5] = 1e-10f;

  kf.mxp.numRows = kf.mxp.numCols = EKF_STATE;
  kf.mxq.numRows = kf.mxq.numCols = EKF_STATE;
  kf.mxa.numCols = kf.mxa.numRows = EKF_OMEGA;
  kf.mxr.numCols = kf.mxr.numRows = EKF_STATE;
  kf.mxh.numRows = 1;
  kf.mxh.numCols = EKF_MEASM;

  kf.mxp.pData = (float *)EKF_P;
  kf.mxq.pData = (float *)EKF_Q;
  kf.mxa.pData = (float *)EKF_O;
  kf.mxr.pData = (float *)EKF_F;
  kf.mxh.pData = (float *)EKF_H;

  if (!(conf & option(Manual_Biases)))
  {
    for (fu8 k = 0; k < STATE_HISTORY; ++k)
    {
      svec(k).bias.gx = EKF_BIAS_GYRO_X;
      svec(k).bias.gy = EKF_BIAS_GYRO_Y;
      svec(k).bias.gz = EKF_BIAS_GYRO_Z;
      svec(k).bias.az = EKF_BIAS_ACCL_Z;
    }
    
    imedsv.bias.gx = EKF_BIAS_GYRO_X;
    imedsv.bias.gy = EKF_BIAS_GYRO_Y;
    imedsv.bias.gz = EKF_BIAS_GYRO_Z;
    imedsv.bias.az = EKF_BIAS_ACCL_Z;
  }

  if (!(conf & option(Using_Ascent_KF)))
  {
    /* Switched to Ascent mid-flight or user messed with
     * FC_DEFAULTS. */

    fc_msg cmd = fc_mask(Reinit_IMU);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }

  timer_update(AscentKF);
  fetch_or(&g_conf, option(Using_Ascent_KF), Rel);
}

void ascent_predict(const float dt, fu32 conf)
{
  fc_lock(&meas_locks[IMU]);

  f_xyz w = meas.gyro;
  f_xyz a = meas.accl;

  fc_unlock(&meas_locks[IMU]);

  w.x = rad(w.x) - svec(1).bias.gx;
  w.y = rad(w.y) - svec(1).bias.gy;
  w.z = rad(w.z) - svec(1).bias.gz;

  EKF_O[0][1] = EKF_O[3][2] = -w.x;
  EKF_O[1][0] = EKF_O[2][3] = w.x;
  EKF_O[0][2] = EKF_O[1][3] = -w.y;
  EKF_O[2][0] = EKF_O[3][1] = w.y;
  EKF_O[2][1] = EKF_O[0][3] = -w.z;
  EKF_O[1][2] = EKF_O[3][0] = w.z;

  float *start = kfalloc(EKF_PREDICT_BYTES);
  const float qmid_scale = 0.5f * dt;

  EKF_F[0][1] = dt;

  matrix v_quat = {EKF_OMEGA, 1, &qv.q0};
  matrix v_qmid = {EKF_OMEGA, 1, start};
  matrix v_k1k2 = {EKF_OMEGA, 1, mxoff(&v_qmid)};
  matrix m_pq = {EKF_STATE, EKF_STATE, NULL};

  offok(&v_k1k2, &v_qmid, "o k1k2");

  matvec_mul(&kf.mxa, &qv.q0, v_qmid.pData);
  mx_scale(&v_qmid, qmid_scale, &v_qmid);

  /* If using midpoint method, adjust qmid_scale */

  // mx_add(&v_qmid, &v_quat, &v_qmid);
  // matvec_mul(&kf.mxa, v_qmid.pData, v_k1k2.pData);
  // mx_scale(&v_quat, qmid_scale, &v_quat);

  mx_add(&v_quat, &v_qmid, &v_quat);

  veok(&v_quat, &qv.q0, "v quat");

  float inormq = inorm4(qv.q0, qv.rho1, qv.rho2, qv.rho3);

  qv.q0   *= inormq;
  qv.rho1 *= inormq;
  qv.rho2 *= inormq;
  qv.rho3 *= inormq;

  float r13 = 2.0f * (qv.rho1*qv.rho3 + qv.q0*qv.rho2);
  float r23 = 2.0f * (qv.rho2*qv.rho3 + qv.q0*qv.rho1);
  float r33 = 1.0f - 2.0f * (qv.rho1*qv.rho1 + qv.rho2*qv.rho2);

  float a_vert = (r13 * a.x + r23 * a.y + r33 * a.z);

  imedsv.alt = svec(1).alt + dt * svec(1).vel;
  imedsv.vel = svec(1).vel + dt * (a_vert - svec(1).bias.az - GRAVITY_SI);

  /* v_qmid -> "m_fp"; v_k1k2 (start moved) -> "m_ft"
   */

  v_qmid.numRows = v_qmid.numCols = EKF_STATE;
  v_k1k2.numRows = v_k1k2.numCols = EKF_STATE;
  v_qmid.pData = start;
  v_k1k2.pData = v_qmid.pData + EKF_STATE_SQ;
  m_pq.pData = v_k1k2.pData + EKF_STATE_SQ;

  mxok(&v_qmid, EKF_STATE, EKF_STATE, "#2 m fp");
  mxok(&v_k1k2, EKF_STATE, EKF_STATE, "#2 m ft");
  offok(&v_k1k2, &v_qmid, "#2 o ft");
  offok(&m_pq, &v_k1k2, "#2 o pq");
  
  mx_transpose(&kf.mxr, &v_k1k2);
  mx_mul(&kf.mxr, &kf.mxp, &v_qmid);
  mx_mul(&v_qmid, &v_k1k2, &m_pq);
  mx_add(&m_pq, &kf.mxq, &kf.mxp);

  kffree(start);
}

void ascent_update(void)
{
  fc_lock(&meas_locks[Baro]);

  const float baroz_innv = meas.baro.alt - imedsv.alt;

  fc_unlock(&meas_locks[Baro]);

  float *start = kfalloc(EKF_UPDATE_BYTES);
  
  matrix r_hp  = {1, EKF_STATE, start};
  matrix v_ht  = {EKF_STATE, 1, mxoff(&r_hp)};
  matrix v_pht = {EKF_STATE, 1, mxoff(&v_ht)};

  offok(&v_ht, &r_hp, "#1 o ht");
  offok(&v_pht, &v_ht, "#1 o pht");

  float k_denom;

  mx_mul(&kf.mxh, &kf.mxp, &r_hp);
  mx_transpose(&kf.mxh, &v_ht);
  matvec_mul(&r_hp, v_ht.pData, &k_denom);

  k_denom += EKF_BARO_VARIANCE;

  matvec_mul(&kf.mxp, v_ht.pData, v_pht.pData);
  mx_scale(&v_pht, 1.0f / k_denom, &v_pht);

  veok(&v_pht, v_ht.pData + EKF_STATE, "#1 v pht");

  /* v_pht -> "v_k"; r_hp -> "v_ky"
   */

  matrix v_sv1 = {EKF_STATE, 1, ekf_view(&imedsv)};
  matrix v_sv0 = {EKF_STATE, 1, ekf_view(&svec(0))};

  r_hp.numRows = EKF_STATE;
  r_hp.numCols = 1;

  veok(&v_sv1, (float *)&imedsv + 3, "#2 v sv1");
  veok(&v_sv0, (float *)&svec(0) + 3, "#2 v sv0");

  mx_scale(&v_pht, baroz_innv, &r_hp);
  mx_add(&v_sv1, &r_hp, &v_sv0);

  /* v_sv1 -> "m_kh" -> "m_p_"; v_sv0 -> "m_i-kh"
   */

  v_sv1.numCols = v_sv0.numCols = EKF_STATE;
  v_sv1.pData = mxoff(&v_pht);
  v_sv0.pData = mxoff(&v_sv1);
  EKF_F[0][1] = 0;

  mxok(&v_sv1, EKF_STATE, EKF_STATE, "#3 m kh");
  mxok(&v_sv0, EKF_STATE, EKF_STATE, "#3 m i-kh");
  offok(&v_sv1, &v_pht, "#3 o kh");
  offok(&v_sv0, &v_sv1, "#3 o i-kh");
  
  mx_mul(&v_pht, &kf.mxh, &v_sv1);
  mx_sub(&kf.mxr, &v_sv1, &v_sv0);

  memcpy(v_sv1.pData, EKF_P, fbyte(EKF_STATE_SQ));

  mx_mul(&v_sv0, &v_sv1, &kf.mxp);

  kffree(start);
}
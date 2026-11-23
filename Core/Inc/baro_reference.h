/*
 * Barometer ground reference: the pad pressure that barometric altitude is
 * measured from, tracked against weather and sensor drift while the vehicle
 * sits still on the pad and frozen from Launch on.
 *
 * Pure C (no HAL), driven from the distribution thread: every fresh IMU
 * sample goes to baro_ref_accel()/baro_ref_gyro() and every fresh pressure
 * sample to baro_ref_update().
 *
 * Tracking: the reference follows a 1 s average of the pressure with a
 * BARO_REF_TAU_MS time constant, at most BARO_REF_SLEW_PA_S, and only while
 *   - the accelerometer reads 1 g within BARO_REF_ACC_MAG_G, no axis moves
 *     more than BARO_REF_ACC_JITTER_G from its 0.5 s average, every gyro axis
 *     is under BARO_REF_GYRO_DPS, both sensors are fresh, and
 *   - the 1 s and 10 s pressure averages agree within BARO_REF_STEP_PA
 *     (a door slam, a gust or a real climb breaks this),
 * all continuously for BARO_REF_QUIET_MS. Otherwise it is paused.
 *
 * Freeze: the first sample at Launch or later freezes the reference at its
 * value from BARO_REF_LOOKBACK_MS earlier (a short history kept every
 * BARO_REF_RING_MS), so neither the launch transient nor the motor's
 * pressure effects before detection reach it. It stays frozen in flight and
 * resumes tracking only when the FC is back before Launch (a new flight).
 *
 * Temperature: the BMP390 compensates its pressure for die temperature
 * itself; the residual offset drift is slow and tracked like weather on the
 * pad. The sensor temperature stays in every barometer record (SD and
 * BAROMETER_DATA) for review; nothing compensates it in flight.
 */
#ifndef BARO_REFERENCE_H
#define BARO_REFERENCE_H

#include <stdbool.h>
#include <stdint.h>

#define BARO_REF_TAU_MS        30000u  /* tracking time constant */
#define BARO_REF_SLEW_PA_S     0.25f   /* max tracking rate (9 hPa/h) */
#define BARO_REF_MAX_SHIFT_PA  2000.0f /* max tracked shift from the last zero */
#define BARO_REF_FAST_MS       1000u   /* pressure average the reference follows */
#define BARO_REF_SLOW_MS       10000u  /* pressure average for the transient gate */
#define BARO_REF_STEP_PA       8.0f    /* |fast - slow| above this: transient */
#define BARO_REF_QUIET_MS      2000u   /* still this long before tracking */
#define BARO_REF_GAP_MS        1000u   /* a longer pressure gap restarts the averages */
#define BARO_REF_G             9.80665f /* the accelerometer reports m/s^2 */
#define BARO_REF_ACC_MAG_G     0.15f   /* | |a| - 1 g | limit, in g (raw, unbiased) */
#define BARO_REF_ACC_JITTER_G  0.04f   /* per-axis deviation from the 0.5 s average, in g */
#define BARO_REF_ACC_AVG_MS    500u
#define BARO_REF_GYRO_DPS      5.0f    /* per-axis rate limit (raw, unbiased) */
#define BARO_REF_IMU_STALE_MS  250u    /* IMU sample older than this: not still */
#define BARO_REF_RING          8u
#define BARO_REF_RING_MS       250u
#define BARO_REF_LOOKBACK_MS   1500u

enum baro_ref_mode {
  BARO_REF_NONE,     /* no reference yet */
  BARO_REF_TRACKING, /* following slow drift on the pad */
  BARO_REF_PAUSED,   /* on the pad, not still: held */
  BARO_REF_FROZEN,   /* Launch or later: fixed */
};

/* Kept across a watchdog resume (all floats; ref first, it must be > 0). */
typedef struct {
  float ref;    /* current reference (Pa) */
  float boot;   /* first reference since power-up (Pa) */
  float zero;   /* reference at the last (re)zero (Pa) */
  float frozen; /* 1: frozen */
} baro_ref_kept;

extern baro_ref_kept baro_ref;

/* (Re)zero: the reference becomes p. */
void baro_ref_reset(float p, uint32_t now);
/* Fresh accelerometer (m/s^2) and gyroscope (deg/s) samples. */
void baro_ref_accel(float x, float y, float z, uint32_t now);
void baro_ref_gyro(float x, float y, float z, uint32_t now);
/* Fresh pressure sample. on_pad: before Launch. Returns the reference. */
float baro_ref_update(float p, uint32_t now, bool on_pad);
uint8_t baro_ref_mode(void);

/*
 * Height above the reference p0 for pressure p (m): the ISA pressure altitude
 * of p minus that of p0,
 *   h(p) = T0/L * (1 - (p / P0)^(R*L/g0)),   AGL = h(p) - h(p0)
 *        = T0/L * (p0 / P0)^(R*L/g0) * (1 - (p / p0)^(R*L/g0)),
 * the hypsometric equation with the ISA temperature at the pad's pressure
 * altitude (T0/L * (p0/P0)^(R*L/g0) = T_pad/L). The sea-level form
 * T0/L * (1 - (p/p0)^...) assumes a 15 C pad and reads about 2% high at a
 * 900 m site. The ISA pad temperature, not the BMP390's: that sensor sits in
 * the avionics bay and reads the electronics, not the air.
 * The pad factor is recomputed only when p0 changes (every sample while the
 * reference tracks on the pad, never after the freeze at Launch). Not
 * reentrant: the distribution thread is the only caller in flight.
 */
#define BARO_ISA_T0_L  44330.77f   /* 288.15 K / 0.0065 K/m */
#define BARO_ISA_EXP   0.190263f   /* R*L/g0 = 287.053 * 0.0065 / 9.80665 */
#define BARO_ISA_P0    101325.0f
float baro_agl(float p, float p0);

/* For FC BOARD_HEALTH word 15 bits 18..31 (fc_estimator_status.h): bits
 * 0..1 the mode, bits 2..13 the reference minus the boot reference in Pa
 * (signed, saturating at +-2047). 0 before there is a reference. */
uint32_t baro_ref_bits(void);

#endif

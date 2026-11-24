/* Core/Inc/deploy_backup.h */

#ifndef FC_DEPLOY_BACKUP_H
#define FC_DEPLOY_BACKUP_H

#include <stdbool.h>
#include <stdint.h>

/* Chutes without trustworthy sensors (docs/rud-abort.md section 6).
 *
 * Drogue timer. The flight model (GroundStation26 tools/flight_profile,
 * SWEET_V6 on the 10,000 ft motor, the longest flight planned) reaches apogee
 * 25.9 s after the FC detects Launch (26.4 s after liftoff). The model's
 * apogee time is uncertain by about 1 s (drag coefficient 10 %, impulse 5 %),
 * and the sensor path itself fires 0.8-2.3 s after apogee. So the drogue
 * fires BACKUP_DROGUE_MS after Launch if no sensor path has fired it: 3 s
 * past the model apogee plus the sensor path's own latency, never before a
 * working sensor path. While a trustworthy barometer still climbs, it waits
 * up to BACKUP_DROGUE_HOLD_MS more (a flight longer than the model). It is
 * armed only once a measured altitude reached BACKUP_ARM_ALT, so a false
 * Launch detection on the pad never fires a chute there.
 *
 * Main timer. With no trustworthy altitude (barometer implausible, frozen or
 * silent, and no fresh, moving GPS fix) the descent dead-reckons from the
 * last trustworthy altitude. The prediction is built to stay below the true
 * altitude, so the main, which fires when it crosses 457 m, is early (higher,
 * more drift) and never late:
 *
 *   - while the vehicle may still climb (its last trustworthy velocity at a
 *     deceleration of at most DR_DECEL_MAX), the prediction holds;
 *   - until the drogue is out it falls freely, without drag (at most
 *     DR_MAX_MPS);
 *   - under the drogue it descends at DR_DROGUE_MPS, the model's drogue rate
 *     (apogee to 457 m: 23.9-25.0 m/s in the IGT flights) plus 15 % (rate ~
 *     1/sqrt(Cd rho): Cd 15 %, air density 5 %), or at the last trustworthy
 *     descent rate under the drogue if that was faster (a partly opened
 *     drogue); from a faster free fall it slows by at most DR_DROGUE_DECEL;
 *   - under the main at DR_MAIN_MPS (6.3-6.7 m/s modelled, plus 15 %), down
 *     to the pad altitude, where it stops, so Landed still comes.
 *
 * For a reference taken from the drogue at apogee, the main comes
 * (h_ref - 457 m) / DR_DROGUE_MPS after it: up to 25 % of that descent early.
 *
 * A reference from the IMU alone (the barometer out in ascent) is capped by
 * the no-drag ballistic trajectory from the last clean barometer step: in
 * coast nothing climbs higher or faster. Apogee from the IMU alone is not
 * believed before that state could have reached it (backup_apogee_possible):
 * the earliest apogee at DR_DECEL_MAX. When the drogue timer fires, nothing
 * found Apogee, and the reference is the last clean barometer state again
 * (a lower bound in ascent). */

#define BACKUP_DROGUE_MS      30000U  /* after Launch: model apogee 25.9 s + 4.1 s */
#define BACKUP_DROGUE_HOLD_MS 10000U
#define BACKUP_ARM_ALT        30.0f   /* FLYING_ALTITUDE: a measured climb arms the timer */
#define DR_DROGUE_MPS         28.75f  /* 25.0 m/s x 1.15 */
#define DR_MAIN_MPS           7.7f    /* 6.7 m/s x 1.15 */
#define DR_MAX_MPS            150.0f  /* faster than any descent of this airframe */
#define DR_G                  9.80665f
#define DR_DECEL_MAX          (2.0f * DR_G)  /* coast: gravity and at most 0.54 g of drag (IGT) */
#define DR_DROGUE_DECEL       (3.0f * DR_G)  /* an opening drogue slows the fall */
#define DR_DROGUE_SETTLE_MS   3000U

typedef struct {
  /* The last trustworthy state, and the last clean barometer one. */
  float ref_alt, ref_vel, clean_alt, clean_vel;
  uint32_t ref_ms, clean_ms;
  /* When the drogue and the main came out (first seen). */
  uint32_t drogue_ms, main_ms;
  /* The prediction while dead-reckoning: altitude and fall speed. */
  float alt, fall, rate;
  uint32_t last_ms, climb_until;
  bool have_ref, ref_measured, have_clean, drogue, main, active;
} backup_dr;

void backup_dr_reset(backup_dr *d);
/* A step whose altitude is trustworthy: 'measured' (barometer or GPS), or
 * the IMU's dead reckoning, then capped. Dead reckoning ends. */
void backup_dr_trust(backup_dr *d, uint32_t t_ms, float alt, float vel, bool measured);
/* The drogue timer fired: nothing found Apogee, so an IMU-only reference is
 * suspect; the last clean barometer state is the reference again. */
void backup_dr_measured_only(backup_dr *d);
/* The deployments, as the estimator sees them. */
void backup_dr_chutes(backup_dr *d, uint32_t t_ms, bool drogue_out, bool main_out);
/* One step with no trustworthy altitude: the predicted altitude at t_ms and
 * its velocity. 'fallback_alt' is the reference if none was ever trusted
 * (after a restart). */
float backup_dr_step(backup_dr *d, uint32_t t_ms, float fallback_alt, float *vel);
/* Apogee on the IMU alone: not before the last clean barometer state could
 * have reached it. */
bool backup_apogee_possible(const backup_dr *d, uint32_t t_ms);
/* The drogue timer: 'since_launch_ms' after Launch, held while 'climbing'
 * for at most BACKUP_DROGUE_HOLD_MS. */
bool backup_drogue_due(uint32_t since_launch_ms, bool climbing);

#endif /* FC_DEPLOY_BACKUP_H */

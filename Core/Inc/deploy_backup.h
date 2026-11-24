/* Core/Inc/deploy_backup.h */

#ifndef FC_DEPLOY_BACKUP_H
#define FC_DEPLOY_BACKUP_H

#include <stdbool.h>
#include <stdint.h>

/* The main without a trustworthy altitude (docs/rud-abort.md section 6).
 *
 * With no trustworthy altitude (barometer implausible or frozen, and no
 * fresh, moving GPS fix) the descent dead-reckons from the last trustworthy
 * altitude. The prediction is built to stay below the true altitude, so the
 * main, which fires when it crosses 457 m, is early (higher, more drift) and
 * never late:
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
 * A reference that is not measured (the IMU's dead reckoning) is capped by
 * the no-drag ballistic trajectory from the last measured one: in coast
 * nothing climbs higher or faster. */

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
  bool have_ref, have_clean, drogue, main, active;
} backup_dr;

void backup_dr_reset(backup_dr *d);
/* A step whose altitude is trustworthy: 'measured' (barometer or GPS), or
 * the IMU's dead reckoning, then capped. Dead reckoning ends. */
void backup_dr_trust(backup_dr *d, uint32_t t_ms, float alt, float vel, bool measured);
/* The deployments, as the estimator sees them. */
void backup_dr_chutes(backup_dr *d, uint32_t t_ms, bool drogue_out, bool main_out);
/* One step with no trustworthy altitude: the predicted altitude at t_ms and
 * its velocity. 'fallback_alt' is the reference if none was ever trusted
 * (after a restart). */
float backup_dr_step(backup_dr *d, uint32_t t_ms, float fallback_alt, float *vel);

#endif /* FC_DEPLOY_BACKUP_H */

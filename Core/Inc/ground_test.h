/* Core/Inc/ground_test.h
 *
 * Integrated ground test (GroundStation26 docs/integrated-ground-test.md).
 * After a real Launch on the stand the FC replays IMU, barometer and GPS rows
 * from SD-card CSVs (/IGT/IMU0.CSV, BARO0.CSV, GPS0.CSV ...) into its real
 * estimator and state machine, and gates the real pyro outputs (dry-run by
 * default). Built only with -DINTEGRATED_GROUND_TEST=ON (FC_GROUND_TEST=1);
 * a flight image compiles every hook below to a constant. The mode lives in
 * RAM only: any reset or power cycle starts Off. */

#ifndef FC_GROUND_TEST_H
#define FC_GROUND_TEST_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FC_GROUND_TEST
#define FC_GROUND_TEST 0
#endif

/* BOARD_HEALTH word 14 bits 4..6 */
enum igt_mode {
  IGT_OFF, IGT_VALIDATING, IGT_READY, IGT_SIM, IGT_DONE, IGT_FAULT
};

/* BOARD_HEALTH word 14 bits 12..15 */
enum igt_reason {
  IGT_OK, IGT_E_SD, IGT_E_MISSING, IGT_E_HEADER, IGT_E_PARSE, IGT_E_TIME,
  IGT_E_RANGE, IGT_E_SHORT, IGT_E_MOVING, IGT_E_UNUSED_9,
  IGT_I_UNDERRUN, IGT_E_REFUSED, IGT_I_DATA_ENDED
};

/* SIM_SAMPLE element 0 */
enum igt_kind {
  IGT_KIND_IMU = 1, IGT_KIND_BARO, IGT_KIND_GPS,
  IGT_KIND_ASCENT, IGT_KIND_DESCENT
};

/* Command index: fc_msg Actionable_Decrees + 17 + index (compat 29 + index) */
enum igt_command {
  IGT_CMD_ENTER, IGT_CMD_LIVE_PYRO, IGT_CMD_DRY_RUN, IGT_CMD_EXIT
};

#define IGT_HEALTH_SUPPORTED (1U << 7)
#define IGT_HEALTH_LIVE      (1U << 8)
#define IGT_HEALTH_GPS_SIM   (1U << 9)
#define IGT_HEALTH_DRY_CO2   (1U << 10)
#define IGT_HEALTH_DRY_REEF  (1U << 11)

#if FC_GROUND_TEST

/* Recovery task. 'pad' = FC Startup, no Postinit/Launch request, ground
 * station heard and no admitted flight. Returns whether it was accepted. */
bool igt_command(unsigned cmd, bool pad, bool launch_requested);
/* Stop, Abort, reset to Startup: back to Off (dry-run). */
void igt_stop(void);
/* Operator SD retry: once mounted, a card fault re-runs the validation. */
void igt_sd_retry(void);
/* Launch accepted. Starts the replay only from Ready and at rest; the real
 * samples are accl (m/s^2), gyro (deg/s) and baro altitude (m). On success
 * *seed receives the replay's first IMU row (accl) for the attitude. */
bool igt_launch(uint32_t now, const float *accl, const float *gyro,
                float alt, float *seed);
/* The FC entered Landed: the replay ends, real sensors and GPS resume. */
void igt_landed(void);

/* Distribution task: replace a fresh real sample with the replay at
 * t = now - T0. The real value is kept for the normal telemetry types. */
bool igt_sim_running(void);
void igt_override_accl(float *v, uint32_t now);
void igt_override_gyro(float *v, uint32_t now);
void igt_override_baro(float *v, uint32_t now);
/* The replayed GPS fix due at 'now' (10 Hz), as {sea, lon, lat}. */
bool igt_gps_simulated(void);
bool igt_gps_fix(float *kf_gps, uint32_t now);
/* Real samples (6 IMU floats: accl then gyro; 3 barometer floats). */
const float *igt_real_imu(void);
const float *igt_real_baro(void);
/* Publish a replayed sample / estimator output as SIM_SAMPLE. */
void igt_publish(unsigned kind, const float *v, unsigned count, uint32_t now);
/* The replay started a flight whose checkpoint must never be resumed. */
bool igt_engaged(void);

/* Pyro gate (deployment_pulse_start, IRQs masked: no I/O). */
bool igt_pyro_dry(void);
void igt_note_dry(bool reef);

/* BOARD_HEALTH word 14 bits 4..15 (0 when Off and never entered). */
uint32_t igt_health_bits(void);

/* SD task: validation and ring refills; owns every IGT FileX call. It
 * shares the thread with the SD read service (sdpipeline.c) and holds no
 * FileX handle between calls. */
void igt_sd_service(void);
/* SD task, from sd_pipeline_media_remounted (a fresh mount replaced the old
 * one): Validating/Ready validate the new card again; mid-replay the
 * streams end (the last rows are held, as on a read error). */
void igt_media_remounted(void);

#else

static inline bool igt_sim_running(void) { return false; }
static inline bool igt_gps_simulated(void) { return false; }
static inline bool igt_engaged(void) { return false; }
static inline bool igt_pyro_dry(void) { return false; }
static inline void igt_note_dry(bool reef) { (void)reef; }
static inline void igt_stop(void) {}
static inline void igt_landed(void) {}
static inline uint32_t igt_health_bits(void) { return 0U; }
static inline void igt_sd_service(void) {}
static inline void igt_media_remounted(void) {}

#endif /* FC_GROUND_TEST */
#endif /* FC_GROUND_TEST_H */

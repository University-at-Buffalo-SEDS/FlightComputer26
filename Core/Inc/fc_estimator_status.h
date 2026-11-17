#ifndef FC_ESTIMATOR_STATUS_H
#define FC_ESTIMATOR_STATUS_H
#include <stdint.h>
/* Flight estimator (Kalman filter) status for the ground station, carried in
 * FC BOARD_HEALTH words 13 and 15 (board_health.c). Other boards use those
 * words only with word 1 flags 4 and 64, which this FC never sets.
 *   word 13: bits 24..31 the 'K' tag, bits 20..23 the phase, bits 0..19 the
 *            completed filter cycles this flight (wraps; liveness and rate).
 *   word 15: the FC uptime tick (word 2's low word, word 4 Hz) at which the
 *            filter last started running; 0 until it has run.
 * Launch resumes the evaluator: Idle -> Init, and its first completed cycle
 * makes it Running. A phase change is reported at once and again 1 s later,
 * then every 5 s with the rest of the report. */
enum {
  FC_EST_IDLE,      /* not started this flight */
  FC_EST_INIT,      /* Launch accepted, filter initializing */
  FC_EST_RUNNING,
  FC_EST_CONVERGED, /* FC_EST_CONVERGE_CYCLES finite cycles since start */
  FC_EST_PAUSED,    /* recovery holds the evaluator */
  FC_EST_STOPPED,   /* evaluation stopped (Evaluation Abort, sensor restart) */
  FC_EST_DIVERGED,  /* non-finite state, or the FC stopped trusting it */
};
#define FC_EST_TAG 0x4BU
#define FC_EST_CONVERGE_CYCLES 64U
#define FC_EST_WORD13(phase, cycles) \
  ((FC_EST_TAG << 24) | (((uint32_t)(phase) & 0xFU) << 20) | ((uint32_t)(cycles) & 0xFFFFFU))
extern volatile uint8_t g_fc_est_phase;
extern volatile uint32_t g_fc_est_cycles, g_fc_est_start;
#endif

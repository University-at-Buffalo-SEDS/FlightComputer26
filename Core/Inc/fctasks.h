/* Core/Inc/fctasks.h */

#ifndef FC_TASKS
#define FC_TASKS

#include "platform.h"


/* Telemetry Task */

#if defined TELEMETRY_ENABLED || defined(FAKESTATION)

#define TLMT_INPUT 0
#define TLMT_PRIORITY 5
#define TLMT_STACK_BYTES (48U * 1024U)
#define TLMT_STACK_ULONG (TLMT_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD telemetry_task;
extern TX_MUTEX telemetry_mu;
extern TX_BYTE_POOL telemetry_pool;

void telemetry_lock(void);
void telemetry_unlock(void);
void telemetry_wake(void);
void telemetry_entry(ULONG);
UINT create_telemetry_task(TX_BYTE_POOL *);

#endif /* TELEMETRY_ENABLED + FAKESTATION */


/* Recovery (control) Task. Thread priorities, as on the other boards: 4
 * control, 5 telemetry, acquisition (distribution) and evaluation, 5 FileX
 * (FX_APP_THREAD_PRIO). No time slices and no preemption thresholds. */

#define RECV_INPUT 0
#define RECV_PRIORITY 4
#define RECV_STACK_BYTES (12U * 1024U)
#define RECV_STACK_ULONG (RECV_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD recovery_task;

void recovery_entry(ULONG);
UINT create_recovery_task(TX_BYTE_POOL *);


/* Evaluation Task */

#define EVAL_INPUT 0
#define EVAL_PRIORITY 5
/* No preemption threshold and no time slice. A manual Deploy/Expand runs
 * descent_initialize, which rebinds the Kalman matrices to the 4x4 descent
 * set; an ascent predict/update interrupted by it would write 6x6 results
 * into them. The estimator guard (thread_comm.h) keeps recovery commands out
 * of a pass; the other priority-5 threads never preempt it (no slicing). A
 * pass is ~1 ms; pyro pulses are timed in SysTick, not by a thread. */
/* Flight-state changes publish through the SEDSNet router's immediate send
 * path on this stack (measured 6.4 KiB through Landed in simulation). */
#define EVAL_STACK_BYTES (12U * 1024U)
#define EVAL_STACK_ULONG (EVAL_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD evaluation_task;

void evaluation_entry(ULONG);
UINT create_evaluation_task(TX_BYTE_POOL *);


/* Distribution Task */

#define DIST_INPUT 0
#define DIST_PRIORITY 5
/* Sensor publication, the Launch ignition request and the descent evaluator
 * all run the SEDSNet router's immediate send path here. 8 KiB overflowed at
 * Launch (simulation peak: 8.8 KiB at Launch, 9.3 KiB by Recovery), and a
 * router re-initialisation from this task reaches ~13 KiB statically. */
#define DIST_STACK_BYTES (16U * 1024U)
#define DIST_STACK_ULONG (DIST_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD distribution_task;

void distribution_entry(ULONG);
UINT create_distribution_task(TX_BYTE_POOL *);


#endif /* FC_TASKS */

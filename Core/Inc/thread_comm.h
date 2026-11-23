/* Core/Inc/thread_comm.h */

#ifndef FC_THREAD_COMM_H
#define FC_THREAD_COMM_H

#include "tx_api.h"
#include <stdbool.h>

/* Inter-thread objects of the flight computer (docs/task-layout.md). Created
 * once by thread_comm_init() before any thread that uses them starts. */
UINT thread_comm_init(void);

/* Estimator guard. The Kalman matrices and the flight state machine are
 * changed by the estimator pass (evaluation thread) and by commands on the
 * recovery thread; a manual Deploy/Expand's descent_initialize rebinds the
 * matrices to the 4x4 descent set, which must never happen in the middle of
 * an ascent predict/update. The evaluation thread holds the guard for a whole
 * pass and the recovery thread holds it while it executes a command, so the
 * two never interleave. A holder never waits on the other thread while it
 * holds the guard: the evaluator sleeps only through
 * thread_comm_estimator_sleep(), which gives the guard up for the sleep. */
void thread_comm_estimator_enter(void);
void thread_comm_estimator_leave(void);
bool thread_comm_estimator_held(void);
void thread_comm_estimator_sleep(ULONG ticks);

#endif /* FC_THREAD_COMM_H */

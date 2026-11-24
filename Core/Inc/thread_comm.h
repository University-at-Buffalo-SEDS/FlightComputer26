/* Core/Inc/thread_comm.h */

#ifndef FC_THREAD_COMM_H
#define FC_THREAD_COMM_H

#include "tx_api.h"
#include <stdbool.h>
#include <stdint.h>

/* Inter-thread objects of the flight computer (docs/task-layout.md). Created
 * once by thread_comm_init() before any thread that uses them starts. */
UINT thread_comm_init(void);

/* Command queue to the control (recovery) thread. A message is one fc_msg
 * (fcstructs.h: an operator command, an internal decree, a sensor or GPS
 * report, a configuration option) and where it came from. Senders never wait
 * except where they pass a wait option; the timer ISR uses TX_NO_WAIT. */
#define THREAD_COMM_QUEUE_DEPTH 8U
enum { THREAD_COMM_LOCAL = 0U, THREAD_COMM_NETWORK = 1U };
typedef struct {
  uint32_t cmd;     /* fc_msg */
  uint32_t origin;  /* THREAD_COMM_LOCAL or THREAD_COMM_NETWORK */
} thread_comm_msg_t;

/* A command raised on the board (task or ISR). */
UINT thread_comm_send(uint32_t cmd, ULONG wait_option);
/* An admitted operator command from the network (telemetry thread). */
UINT thread_comm_send_network(uint32_t cmd);
UINT thread_comm_receive(thread_comm_msg_t *msg, ULONG wait_option);
/* Drops every queued command (a held sequence after Stop/reset). */
void thread_comm_flush(void);
/* The thread is blocked in thread_comm_receive(). */
bool thread_comm_receiver_waiting(const TX_THREAD *thread);

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

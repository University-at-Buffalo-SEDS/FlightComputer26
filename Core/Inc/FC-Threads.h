#pragma once
#include "tx_api.h"

#ifdef TELEMETRY_ENABLED

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
#define TLMT_INPUT 0
#define TLMT_PRIORITY 5
#define TLMT_TIME_SLICE 20
#define TLMT_STACK_BYTES (16U * 1024U)
#define TLMT_STACK_ULONG (TLMT_STACK_BYTES / sizeof(ULONG))

void telemetry_thread_entry(ULONG initial_input);
UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool);
/* ------ Telemetry Thread ------ */

#endif // TELEMETRY_ENABLED

/* ------ Recovery Task ------ */
#define RECV_INPUT 0
#define RECV_PRIORITY 0
#define RECV_STACK_BYTES 2048
#define RECV_STACK_ULONG (RECV_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD recovery_task;
void recovery_entry(ULONG input);
UINT create_recovery_task(TX_BYTE_POOL *byte_pool);
/* ------ Recovery Task ------ */

/* ------ Evaluation Task ------ */
#define EVAL_INPUT 0
/* Used when preemption threshold is
 * disabled for this task */
#define EVAL_TIME_SLICE 25
#define EVAL_SLEEP_NO_DATA 20
#define EVAL_SLEEP_RT_CONF 10
#define EVAL_PRIORITY 5
#define EVAL_PREEMPT_THRESHOLD 1
#define EVAL_STACK_BYTES 8192
#define EVAL_STACK_ULONG (EVAL_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD evaluation_task;

void evaluation_entry(ULONG input);
UINT create_evaluation_task(TX_BYTE_POOL *byte_pool);
/* ------ Evaluation Task ------ */

/* ------ Distribution Task ------ */
#define DIST_INPUT 0
#define DIST_TIME_SLICE 20
#define DIST_SLEEP_NO_DATA 25
#define DIST_PRIORITY 5
#define DIST_STACK_BYTES 2048
#define DIST_STACK_ULONG (DIST_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD distribution_task;
void distribution_entry(ULONG input);
UINT create_distribution_task(TX_BYTE_POOL *byte_pool);
/* ------ Distribution Task ------ */
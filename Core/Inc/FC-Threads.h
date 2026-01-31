#pragma once
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
extern ULONG telemetry_thread_stack[];

void telemetry_thread_entry(ULONG initial_input);
void create_telemetry_thread(void);
/* ------ Telemetry Thread ------ */

/* ------ Recovery Task ------ */
#define RECV_INPUT 0
#define RECV_PRIORITY 0
#define RECV_STACK_BYTES 2048
#define RECV_STACK_ULONG (RECV_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD recovery_task;
extern ULONG recovery_stack[];

void recovery_entry(ULONG input);
void create_recovery_task(void);
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
extern ULONG evaluation_stack[];

void evaluation_entry(ULONG input);
void create_evaluation_task(void);
/* ------ Evaluation Task ------ */

/* ------ Distribution Task ------ */
#define DIST_INPUT 0
#define DIST_TIME_SLICE 20
#define DIST_SLEEP_NO_DATA 5
#define DIST_PRIORITY 5
#define DIST_STACK_BYTES 2048
#define DIST_STACK_ULONG (DIST_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD distribution_task;
extern ULONG distribution_stack[];

void distribution_entry(ULONG input);
void create_distribution_task(void);
/* ------ Distribution Task ------ */
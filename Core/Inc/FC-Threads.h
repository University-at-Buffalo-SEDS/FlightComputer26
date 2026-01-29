#pragma once
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
extern ULONG telemetry_thread_stack[];

void telemetry_thread_entry(ULONG initial_input);
void create_telemetry_thread(void);
/* ------ Telemetry Thread ------ */

/* ------ Recovery Task ------ */
#define RECOVERY_INPUT 0
#define RECOVERY_PRIORITY 10
#define RECOVERY_STACK_BYTES 2048
#define RECOVERY_STACK_ULONG (RECOVERY_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD recovery_task;
extern ULONG recovery_stack[];

void recovery_entry(ULONG flag);
void create_recovery_task(void);
/* ------ Recovery Task ------ */

/* ------ Prediction Task ------ */
#define PREDICT_INPUT 0
#define PREDICT_SLEEP 45
#define PREDICT_PRIORITY 4
#define PREDICT_STACK_BYTES 8192
#define PREDICT_STACK_ULONG (PREDICT_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD predict_task;
extern ULONG predict_stack[];

void predict_entry(ULONG last);
void create_predict_task(void);
/* ------ Prediction Task ------ */

/* ------ Distribution Task ------ */
#define DISTRIB_INPUT 0
#define DISTRIB_SLEEP 20
#define DISTRIB_PRIORITY 5
#define DISTRIB_STACK_BYTES 2048
#define DISTRIB_STACK_ULONG (DISTRIB_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD distribution_task;
extern ULONG distribution_stack[];

void distribution_entry(ULONG input);
void create_distribution_task(void);
/* ------ Distribution Task ------ */
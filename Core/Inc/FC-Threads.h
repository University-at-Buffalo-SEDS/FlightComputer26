#pragma once
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
extern ULONG telemetry_thread_stack[];

void telemetry_thread_entry(ULONG initial_input);
void create_telemetry_thread(void);
/* ------ Telemetry Thread ------ */

/* ------ Deployment Thread ------ */
#define DEPLOYMENT_THREAD_SLEEP         60
#define DEPLOYMENT_THREAD_PRIORITY      6
#define DEPLOYMENT_THREAD_INPUT         0u
#define DEPLOYMENT_THREAD_STACK_SIZE    6144u
#define DEPLOYMENT_THREAD_MAX_RETRIES   60

extern TX_THREAD deployment_thread;
extern ULONG deployment_thread_stack[];

void deployment_thread_entry(ULONG input);
void create_deployment_thread(void);
/* ------ Deployment Thread ------ */
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
#define DEPLOYMENT_THREAD_RETRIES       3u
#define DEPLOYMENT_RESTART_ON_FAIL      1

extern TX_THREAD deployment_thread;
extern ULONG deployment_thread_stack[];

void deployment_thread_entry(ULONG cycle);
void create_deployment_thread(void);
/* ------ Deployment Thread ------ */

/* ------ UKF Thread ------ */
#define UKF_THREAD_SLEEP      45
#define UKF_THREAD_PRIORITY   3
#define UKF_THREAD_INPUT      0UL
#define UKF_THREAD_STACK_SIZE 6144u

extern TX_THREAD ukf_thread;
extern ULONG ukf_thread_stack[];

void ukf_thread_entry(ULONG input);
void create_ukf_thread(void);
/* ------ UKF Thread ------ */
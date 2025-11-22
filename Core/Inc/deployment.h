/*
 * Public API for the parachute deployment logic and thread.
 */

#pragma once

/* Deployment thread configuration */
#define DEPLOYMENT_THREAD_SLEEP         10
#define DEPLOYMENT_THREAD_PRIORITY      5
#define DEPLOYMENT_THREAD_INPUT         0u
#define DEPLOYMENT_THREAD_STACK_SIZE    8192u
#define DEPLOYMENT_THREAD_MAX_RETRIES   70
#define DEPLOYMENT_DEFAULT_LOG_SIZE     36

/* Creates a parachute deployment thread with defined parameters */
void create_deployment_thread(void);
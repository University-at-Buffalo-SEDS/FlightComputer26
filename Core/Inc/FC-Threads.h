/* Core/Inc/FC-Threads.h */

#ifndef FC_THREADS_H
#define FC_THREADS_H

#include "platform.h"


/* ------ Telemetry thread (telemetry.c) ------ */

#if defined TELEMETRY_ENABLED || defined(FAKESTATION)

#define TELEMETRY_INPUT 0
#define TELEMETRY_PRIORITY 5
#define TELEMETRY_STACK_BYTES (48U * 1024U)
#define TELEMETRY_STACK_ULONG (TELEMETRY_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD telemetry_thread;
extern TX_MUTEX telemetry_mu;
extern TX_BYTE_POOL telemetry_pool;

void telemetry_lock(void);
void telemetry_unlock(void);
void telemetry_wake(void);
void telemetry_thread_entry(ULONG);
UINT create_telemetry_thread(TX_BYTE_POOL *);

#endif /* TELEMETRY_ENABLED + FAKESTATION */


/* ------ Control thread (control_thread.c) ------
 * Thread priorities, as on the other boards: 4 control; 5 telemetry,
 * sensor (acquisition), estimator and the FileX SD thread
 * (FX_APP_THREAD_PRIO). No time slices and no preemption thresholds. */

#define CONTROL_INPUT 0
#define CONTROL_PRIORITY 4
#define CONTROL_STACK_BYTES (12U * 1024U)
#define CONTROL_STACK_ULONG (CONTROL_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD control_thread;

void control_thread_entry(ULONG);
UINT create_control_thread(TX_BYTE_POOL *);


/* ------ Estimator thread (estimator_thread.c) ------ */

#define ESTIMATOR_INPUT 0
#define ESTIMATOR_PRIORITY 5
/* No preemption threshold and no time slice. A manual Deploy/Expand runs
 * descent_initialize, which rebinds the Kalman matrices to the 4x4 descent
 * set; an ascent predict/update interrupted by it would write 6x6 results
 * into them. The estimator guard (thread_comm.h) keeps recovery commands out
 * of a pass; the other priority-5 threads never preempt it (no slicing). A
 * pass is ~1 ms; pyro pulses are timed in SysTick, not by a thread. */
/* Flight-state changes publish through the SEDSNet router's immediate send
 * path on this stack (measured 6.4 KiB through Landed in simulation). */
#define ESTIMATOR_STACK_BYTES (12U * 1024U)
#define ESTIMATOR_STACK_ULONG (ESTIMATOR_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD estimator_thread;

void estimator_thread_entry(ULONG);
UINT create_estimator_thread(TX_BYTE_POOL *);


/* ------ Sensor thread (sensor_thread.c, sensor_dma.c) ------ */

#define SENSOR_INPUT 0
#define SENSOR_PRIORITY 5
/* Sensor publication runs the SEDSNet router's immediate send path here
 * (the descent filter and state machine moved to the evaluation thread). 8 KiB overflowed at
 * Launch (simulation peak: 8.8 KiB at Launch, 9.3 KiB by Recovery), and a
 * router re-initialisation from this task reaches ~13 KiB statically. */
#define SENSOR_STACK_BYTES (16U * 1024U)
#define SENSOR_STACK_ULONG (SENSOR_STACK_BYTES / sizeof(ULONG))

extern TX_THREAD sensor_thread;

void sensor_thread_entry(ULONG);
UINT create_sensor_thread(TX_BYTE_POOL *);


#endif /* FC_THREADS_H */

#ifndef FC_MONITOR_BENCH_H
#define FC_MONITOR_BENCH_H
#include <stdbool.h>
/* Recovery owner services session/lease independently of flight state. */
void fc_monitor_bench_poll(void);
void fc_monitor_bench_sample(bool gyro, bool accel, bool baro);
bool kalman_monitor_initialize(void);
#endif

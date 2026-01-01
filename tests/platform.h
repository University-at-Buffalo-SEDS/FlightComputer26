/*
 * Platform abstraction header (HAL, Drivers, ThreadX, Telemetry).
 * Used for the host (emulation) profile.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include "emulation.h"

/* Generic definitions */

/* Max/min helpers with double-eval safety */

#define MAX(x, y)       \
  ({                    \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x > _y ? _x : _y;  \
  })

#define MIN(x, y)       \
  ({                    \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x < _y ? _x : _y;  \
  })

/* Deployment specific fake Kalman struct 
* (remove when Kalman API is exposed) */

typedef struct {
  float alt;
  float vel;
  float vax;
} filter_t;


/* The following alias definitions in emulation.h */

/* ThreadX substitution API */

#define TX_SUCCESS 0
#define TX_FAILURE -1
#define TX_NO_TIME_SLICE 0
#define TX_AUTO_START 0

extern TX_THREAD ukf_thread;
extern TX_THREAD deployment_thread;

#define DEPLOYMENT_THREAD_SLEEP         60
#define DEPLOYMENT_THREAD_PRIORITY      6
#define DEPLOYMENT_THREAD_INPUT         0u
#define DEPLOYMENT_THREAD_STACK_SIZE    6144u
#define DEPLOYMENT_THREAD_RETRIES       3u
#define DEPLOYMENT_RESTART_ON_FAIL      1

#define UKF_THREAD_SLEEP      45
#define UKF_THREAD_PRIORITY   3
#define UKF_THREAD_INPUT      0UL
#define UKF_THREAD_STACK_SIZE 6144u

#define tx_thread_sleep(duration) emu_sleep((duration))

#define tx_thread_resume(thread)  emu_yield((thread))

#define tx_thread_create(thread, name, entry, input,        \
                         stack, stack_size, priority,       \
                         preemption, time_slice, autostart) \
  emu_create_thread((thread), (name), (entry), (input),     \
                    (stack), (stack_size), (priority),      \
                    (preemption), (time_slice), (autostart))\


/* Telemetry API abstraction */

#define log_msg_sync(msg, size)                             \
  emu_print_buf(SEDS_DT_MESSAGE_DATA,                       \
                (msg), (size), sizeof(char))

#define log_msg(msg, size)                                  \
  emu_print_buf(SEDS_DT_MESSAGE_DATA,                       \
                (msg), (size), sizeof(char))

#if defined (__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                              \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define log_err(fmt, ...)                                   \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...)                                   \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)

#else
#ifdef __GNUC__

#define log_err_sync(fmt, ...)                              \
  emu_print_err(fmt, ##__VA_ARGS__)

#define log_err(fmt, ...)                                   \
  emu_print_err(fmt, ##__VA_ARGS__)

#define log_die(fmt, ...)                                   \
  emu_print_err(fmt, ##__VA_ARGS__)

#else /* Does not support 0 variadic arguments */

#define log_err_sync(fmt, ...)                              \
  emu_print_err(fmt, __VA_ARGS__)

#define log_err(fmt, ...)                                   \
  emu_print_err(fmt, __VA_ARGS__)

#define log_die(fmt, ...)                                   \
  emu_print_err(fmt, __VA_ARGS__)

#endif // GNUC
#endif // >= C23


/* HAAL (hardware abstraction abstraction layer) <3 */

/* Time */

#define hal_time_ms() emu_time_ms()

/* Parachute deployment definitions
 * This can also be used for manual emergency deployment */

#define co2_low()                                           \
  emu_print_event("CO2 set up to low")

#define co2_high()                                          \
  emu_print_event("Rose edge on CO2, fired parachute")

#define reef_low()                                          \
  emu_print_event("REEF set up to low")

#define reef_high()                                         \
  emu_print_event("Rose edge on REEF, expanded parachute")

/* Data cache calls */

#define invalidate_dcache()                                 \
  emu_print_event("Invalidated entire data cache")

/* Interrupts toggle */

#define __disable_irq()  emu_disable_irq()
#define __enable_irq()   emu_enable_irq()

/*
 * Host runs a multithreaded CPU
 * and needs a hardware memory barrier.
 */
#define __DMB() __sync_synchronize()

/* Sensor drivers and data collection */

#define init_baro() emu_init_sensor(0)
#define init_gyro() emu_init_sensor(1)
#define init_accel() emu_init_sensor(2)

/* Duplicate declaration of deployment entry for testing */
void create_ukf_thread();
void create_deployment_thread();

#endif // PLATFORM_H
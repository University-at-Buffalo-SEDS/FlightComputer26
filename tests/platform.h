/*
 * Platform abstraction header (HAL, Drivers, ThreadX, Telemetry).
 * Used for the host (emulation) profile.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

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


/* ThreadX API abstraction */

extern unsigned sensor_thread;
extern unsigned kalman_thread;
extern unsigned telemetry_thread;
extern unsigned deployment_thread;

#define DEPLOYMENT_THREAD_SLEEP         60
#define DEPLOYMENT_THREAD_PRIORITY      6
#define DEPLOYMENT_THREAD_INPUT         0u
#define DEPLOYMENT_THREAD_STACK_SIZE    6144u
#define DEPLOYMENT_THREAD_MAX_RETRIES   60

#define FC_TX_UINT unsigned
#define FC_TX_SUCCESS 0
#define FC_TX_FAILURE -1

#define FC_TX_WAIT(duration) emu_sleep((duration))

#define FC_TX_YIELD(thread)  emu_yield((thread))

#define FC_CREATE_THREAD(thread, name, entry, input,        \
                         stack, stack_size, priority,       \
                         preemption, time_slice, autostart) \
  emu_create_thread((thread), (name), (entry), (input),     \
                    (stack), (stack_size), (priority),      \
                    (preemption), (time_slice), (autostart))\


/* Telemetry API abstraction */

typedef enum { SEDS_DT_MESSAGE_DATA } emu_data_e;

#define LOG_MSG_SYNC(msg, size)                             \
  emu_print_buf(SEDS_DT_MESSAGE_DATA,                       \
                (msg), (size), sizeof(char))

#define LOG_MSG(msg, size)                                  \
  emu_print_buf(SEDS_DT_MESSAGE_DATA,                       \
                (msg), (size), sizeof(char))

#if defined (__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define LOG_ERR_SYNC(fmt, ...)                              \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define LOG_ERR(fmt, ...)                                   \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)

#define LOG_DIE(fmt, ...)                                   \
  emu_print_err(fmt __VA_OPT__(,) __VA_ARGS__)

#else
#ifdef __GNUC__

#define LOG_ERR_SYNC(fmt, ...)                              \
  emu_print_err(fmt, ##__VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  emu_print_err(fmt, ##__VA_ARGS__)

#define LOG_DIE(fmt, ...)                                   \
  emu_print_err(fmt, ##__VA_ARGS__)

#else /* Does not support 0 variadic arguments */

#define LOG_ERR_SYNC(fmt, ...)                              \
  emu_print_err(fmt, __VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  emu_print_err(fmt, __VA_ARGS__)

#define LOG_DIE(fmt, ...)                                   \
  emu_print_err(fmt, __VA_ARGS__)

#endif // GNUC
#endif // >= C23


/* HAAL (hardware abstraction abstraction layer) <3 */

/* Parachute deployment definitions
 * This can also be used for manual emergency deployment */

#define CO2_LOW()                                           \
  emu_print_event("CO2 set up to low")

#define CO2_HIGH()                                          \
  emu_print_event("Rose edge on CO2, fired parachute")

#define REEF_LOW()                                          \
  emu_print_event("REEF set up to low")

#define REEF_HIGH()                                         \
  emu_print_event("Rose edge on REEF, expanded parachute")

/* Data cache calls */

#define INVALIDATE_DCACHE()                                 \
  emu_print_event("Invalidated entire data cache")

/* Interrupts toggle */

#define DISABLE_HAL_INTS()  emu_disable_irq()
#define ENABLE_HAL_INTS()   emu_enable_irq()


/* Sensor drivers and data collection */

#define BARO_INIT() emu_baro_init()
#define GYRO_INIT() emu_gyro_init()
//#define ACCEL_INIT() emu_accel_init()


/* Deployment specific fake Kalman struct 
 * (remove when Kalman API is exposed) */

typedef struct {
  float alt;
  float vel;
  float vax;
} filter_t;


#endif // PLATFORM_H
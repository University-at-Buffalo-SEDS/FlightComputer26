/*
 * Platform abstraction header (HAL, Drivers, ThreadX, Telemetry).
 * Used for the embedded (target) profile.
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

#include "tx_api.h"
#include "tx_port.h"
#include "FC-Threads.h"

#define FC_TX_UINT UINT
#define FC_TX_SUCCESS TX_SUCCESS

#define FC_TX_WAIT(duration) tx_thread_sleep((duration))

#define FC_TX_YIELD(thread)  tx_thread_resume((thread))

#define FC_CREATE_THREAD(thread, name, entry, input,        \
                         stack, stack_size, priority,       \
                         preemption, time_slice, autostart) \
  tx_thread_create((thread), (name), (entry), (input),      \
                   (stack), (stack_size), (priority),       \
                   (preemption), (time_slice), (autostart)) \


/* Telemetry API abstraction */

#include <sedsprintf.h>
#include "telemetry.h"

#define LOG_MSG_SYNC(msg, size)                             \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,           \
                            (msg), (size), sizeof(char))

#define LOG_MSG(msg, size)                                  \
  log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,          \
                             (msg), (size), sizeof(char))

#if defined (__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define LOG_ERR_SYNC(fmt, ...)                              \
  log_error_syncronous(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define LOG_ERR(fmt, ...)                                   \
  log_error_asyncronous(fmt __VA_OPT__(,) __VA_ARGS__)

#define LOG_DIE(fmt, ...) die(fmt __VA_OPT__(,) __VA_ARGS__)

#else
#ifdef __GNUC__

#define LOG_ERR_SYNC(fmt, ...)                              \
  log_error_syncronous(fmt, ##__VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  log_error_asyncronous(fmt, ##__VA_ARGS__)

#define LOG_DIE(fmt, ...) die(fmt, ##__VA_ARGS__)

#else /* Does not support 0 variadic arguments */

#define LOG_ERR_SYNC(fmt, ...)                              \
  log_error_syncronous(fmt, __VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  log_error_asyncronous(fmt, __VA_ARGS__)

#define LOG_DIE(fmt, ...) die(fmt, __VA_ARGS__)

#endif // GNUC
#endif // >= C23


/* HAAL (hardware abstraction abstraction layer) <3 */

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_dcache.h"

extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;
extern atomic_uint_fast8_t newdata;

/* Parachute deployment definitions
 * This can also be used for manual emergency deployment */

#define CO2_LOW()                                           \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define CO2_HIGH()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET)

#define REEF_LOW()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define REEF_HIGH()                                         \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_SET)

/* Data cache calls */

#define INVALIDATE_DCACHE() HAL_DCACHE_Invalidate(&hdcache1)

/* Interrupts toggle */

#define DISABLE_HAL_INTS()  __disable_irq()
#define ENABLE_HAL_INTS()   __enable_irq()


/* Sensor drivers and data collection */

#include "gyro.h"
//#include "accel.h"
#include "barometer.h"
//#include "dma-ring.h"

#define BARO_INIT() init_barometer(&hspi1)
#define GYRO_INIT() gyro_init(&hspi1)
//#define ACCEL_INIT() accel_init(&hspi1)


/* Deployment specific fake Kalman struct 
 * (remove when Kalman API is exposed) */

typedef struct {
  float alt;
  float vel;
  float vax;
} filter_t;


#endif // PLATFORM_H
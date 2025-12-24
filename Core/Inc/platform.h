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


/* ThreadX API includes */

#include "tx_api.h"
#include "tx_port.h"
#include "FC-Threads.h"

/* Telemetry API abstraction */

#include <sedsprintf.h>
#include "telemetry.h"

#define log_msg_sync(msg, size)                             \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,           \
                            (msg), (size), sizeof(char))

#define log_msg(msg, size)                                  \
  log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,          \
                             (msg), (size), sizeof(char))

#if defined (__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...) die(fmt __VA_OPT__(,) __VA_ARGS__)

#else
#ifdef __GNUC__

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt, ##__VA_ARGS__)

#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt, ##__VA_ARGS__)

#define log_die(fmt, ...) die(fmt, ##__VA_ARGS__)

#else /* Does not support 0 variadic arguments */

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt, __VA_ARGS__)

#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt, __VA_ARGS__)

#define log_die(fmt, ...) die(fmt, __VA_ARGS__)

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

/* Parachute deployment definitions
 * This can also be used for manual emergency deployment */

#define co2_low()                                           \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define co2_high()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET)

#define reef_low()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define reef_high()                                         \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_SET)

/* Data cache calls */

#define invalidate_dcache() HAL_DCACHE_Invalidate(&hdcache1)

/* Data memory barrier
 * (#else branch is to be invented :D) */

#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"
#else
#define __DMB() (void)0
#endif // DMB support


/* Sensor drivers and data collection */

#include "gyro.h"
//#include "accel.h"
#include "barometer.h"
//#include "dma-ring.h"

#define init_baro() init_barometer(&hspi1)
#define init_gyro() gyro_init(&hspi1)
//#define init_accel() accel_init(&hspi1)


/* Deployment specific fake Kalman struct 
 * (remove when Kalman API is exposed) */

typedef struct {
  float alt;
  float vel;
  float vax;
} filter_t;


#endif // PLATFORM_H
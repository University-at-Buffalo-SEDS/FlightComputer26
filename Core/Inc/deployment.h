/*
 * Deployment logic header and confirguration file.
 */

#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include <sedsprintf.h>
#include "telemetry.h"
#include "FC-Threads.h"

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"

/* Local ring configuration */

#define DEPL_BUF_SIZE 4

/* Threshold and timing configuration */

#define MIN_SAMP_BURNOUT  6
#define MIN_SAMP_DESCENT  6
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   12

#define MIN_TIME_LAUNCH_MS  300
#define MIN_TIME_BURNOUT_MS 300

#define CONFIRM_INTERVAL_TICKS 20

/* FC '26 GPIO port maps */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6

/* Extrernal API helper macros */

#define WAIT_BEFORE_CONFIRM()                               \
  tx_thread_sleep(CONFIRM_INTERVAL_TICKS)

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

#else
#ifdef __GNUC__

#define LOG_ERR_SYNC(fmt, ...)                              \
  log_error_syncronous(fmt, ##__VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  log_error_asyncronous(fmt, ##__VA_ARGS__)

#else /* Does not support 0 variadic arguments */

#define LOG_ERR_SYNC(fmt, ...)                              \
  log_error_syncronous(fmt, __VA_ARGS__)

#define LOG_ERR(fmt, ...)                                   \
  log_error_asyncronous(fmt, __VA_ARGS__)

#endif // GNUC
#endif // >= C23

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

/* This can also be used for manual emergency deployment */

#define CO2_LOW()                                           \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define CO2_HIGH()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET)

#define REEF_LOW()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define REEF_HIGH()                                         \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_SET)

/* Type definitions */

typedef enum {
  DEPL_OK,
  DEPL_BAD_DATA,
  DEPL_NO_INPUT,
  INFER_INITIAL,
  INFER_CONFIRM
} inference_e;

typedef enum {
  IDLE,
  LAUNCH,
  ASCENT,
  BURNOUT,
  APOGEE,
  DESCENT,
  REEF,
  LANDED
} state_e;

// Needs clock
typedef union {
  uint32_t launch_ms;
  uint32_t burnout_ms;
  uint32_t apogee_ms;
  uint32_t pyro_ms;
  uint32_t reef_ms;
  uint32_t landed_ms;
} time_ex;

typedef union {
  uint_fast16_t burnout;
  uint_fast16_t descent;
  uint_fast16_t landing;
  uint_fast16_t idle;
} samples_ex;

typedef struct {
  uint32_t launch_time_ms;
  uint16_t apogee_height_m;
  uint32_t apogee_time_ms;
  uint16_t reef_height_m;
  uint32_t reef_time_ms;
} stats_t;

typedef struct {
  state_e state;
  time_ex time_of;
  samples_ex samp_of;
  struct {
    uint_fast16_t klm;
    uint_fast16_t cur;
    uint_fast16_t prv;
  } i;
} rocket_t;

// sample struct until kalman api exposed
typedef struct {
  float height_m;
  float vel_mps;
  float accel_mps2;
} filter_t;

#endif // DEPLOYMENT_H
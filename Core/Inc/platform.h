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

/* HAAL */

#include "stm32h5xx_hal.h"

#define hal_time_ms() HAL_GetTick()

#endif
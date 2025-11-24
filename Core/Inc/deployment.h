/*
 * Deployment logic header and confirguration file.
 */

#pragma once

#include <stdio.h>
#include <stdint.h>

/* Threshold and timing configuration */

#define MIN_BURNOUT 8
#define MIN_DESCENT 8
#define MIN_REEF    4
#define MIN_LANDED  12

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
  DEPL_EMPTY_RING
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
  state_e state;
  time_ex time_of;
  samples_ex sampl_of;
  uint_fast16_t ring_index;
  uint_fast16_t apogee_height_ft;
} rocket_t;
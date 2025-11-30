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

/* Local configuration */

#define DEPL_BUF_SIZE 4

#define MIN_SAMP_BURNOUT  6
#define MIN_SAMP_DESCENT  6
#define MIN_SAMP_REEF     4
#define MIN_SAMP_LANDED   12

#define CONFIRM_INTERVAL_TICKS 20

/* Measurement thresholds.
 * Units: altitude ALT (m), velocity VEL (m/s)
 * vertical acceleration VAX (m/s^2) */

#define SANITY_MAX_ALT 15240.0f
#define SANITY_MAX_VEL 150.0f
#define SANITY_MAX_VAX (GRAVITY_MPS2 * 8.0f)

#define SANITY_MIN_ALT -8.0f
#define SANITY_MIN_VEL -4.0f
#define SANITY_MIN_VAX -2.0f

#define LAUNCH_MIN_VEL  6.0f
#define LAUNCH_MIN_VAX  4.0f
#define BURNOUT_MIN_VEL 6.0f
#define BURNOUT_MAX_VAX 1.0f
#define APOGEE_MAX_VEL  1.0f

/* FC '26 GPIO port maps */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6

/* Service definitions */

#define GRAVITY_MPS2 9.80665f

#define DEPL_CODE_MASK (DEPL_BUF_SIZE + 1)

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

/*
 * Service enum used to report inference result, and to
 * provide a compile-time argument to inline functions.
 * DEPL_CODE_MASK is used to adjust bad data reporting
 * range based on a reasonable local buffer size.
 */
typedef enum {
  DEPL_BAD_HEIGHT = -(DEPL_CODE_MASK)*(DEPL_CODE_MASK),
  DEPL_BAD_VEL    = -(DEPL_CODE_MASK),
  DEPL_BAD_ACCEL  = -1,

  DEPL_OK         = 0,

  DEPL_NO_INPUT   = 1,
  DEPL_F_LAUNCH   = 2,
  DEPL_N_BURNOUT  = 3,
  DEPL_N_APOGEE   = 4,
  DEPL_F_APOGEE   = 5,
  DEPL_N_REEF     = 6,
  DEPL_N_LANDED   = 7,

  INFER_INITIAL   = 24,
  INFER_CONFIRM   = 25,

  DEPL_DOOM       = 127 // Assert unreachable
} inference_e;

/*
 * The backbone of state machine.
 */
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

/*
 * Minimum required stats to have some variety
 * of checks we use to make a decision. Calculated
 * for current buffer, useful for two iterations.
 */
typedef struct {
  float min_alt;
  float max_alt;
  float avg_vel;
  float avg_vax;
} stats_t;

/*
 * This struct unifies rocket state, the
 * amount of successive detections of last
 * concerned state (one explicit int), and
 * a toolkit of various indices (see below).
 */
typedef struct {
  state_e state;
  union {
    uint_fast8_t burnout;
    uint_fast8_t descent;
    uint_fast8_t landing;
    uint_fast8_t idle;
  } succ;
  struct {
    uint_fast8_t ex;  /* Position in external filter ring   */
    uint_fast8_t sc;  /* # used elements in "current" buf   */
    uint_fast8_t sp;  /* # used elements in "previous" buf  */
    uint_fast8_t a;   /* Index of "current" buffer (0 or 1) */
  } i;
} rocket_t;

// sample struct until kalman api exposed
typedef struct {
  float alt;
  float vel;
  float vax;
} filter_t;

#endif // DEPLOYMENT_H
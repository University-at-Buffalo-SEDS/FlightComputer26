/*
 * Shared platform header
 *
 * The purpose of this file is to unify the Flight
 * Computer board API and helpers, providing a single
 * point of reference and modification. This file also
 * enables easier conditional compilation and poisoning.
 *
 * This header provides the following components the
 * DMA, Distribution, Evaluation, and Recovery modules:
 *
 * - Type-safe Max, Min, and Abs helper macros;
 * - GPIO and EXTI port mappings;
 * - ThreadX, HAL, sedsprintf_rs, and driver includes;
 * - Variadic aliases for select sedsprintf_rs functions;
 * - Aliases for select HAL and driver functions;
 * - Shared inlined timer implementation with single storage;
 * - Misc aliases and includes as required by the modules.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdatomic.h>


/* ------ Numerical helpers ------ */

#define Max(x, y)       \
  ({                    \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x > _y ? _x : _y;  \
  })

#define Min(x, y)       \
  ({                    \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x < _y ? _x : _y;  \
  })

#define Abs(x)          \
  ({                    \
    typeof(x) _d = (x); \
    _d >= 0 ? _d : -_d; \
  })


/* ------ Platform integer aliases ----- */

/* This fastN type is needed if, for example, our libc
 * implementation defines integers (u)uintN_t to be less
 * than N bits. Minimum width guarantees are then provided
 * by (u)int_fastN_t and (u)int_leastN_t types, with
 * the former being considered the fastest on our platform,
 * and the latter - the smallest given minimum size. */

typedef uint_fast8_t  fu8;
typedef uint_fast16_t fu16;
typedef uint_fast32_t fu32;

typedef int_fast8_t  fi8;
typedef int_fast16_t fi16;
typedef int_fast32_t fi32;


/* ------ Atomic ops and MO aliases ------ */

enum seds_atomic_mo {
  Rlx    = memory_order_relaxed,
  Con    = memory_order_consume,
  Acq    = memory_order_acquire,
  Rel    = memory_order_release,
  AcqRel = memory_order_acq_rel,
  SeqCst = memory_order_seq_cst
};

#define load        atomic_load_explicit
#define store       atomic_store_explicit
#define swap        atomic_exchange_explicit
#define fetch_add   atomic_fetch_add_explicit
#define fetch_sub   atomic_fetch_sub_explicit
#define fetch_and   atomic_fetch_and_explicit
#define fetch_or    atomic_fetch_or_explicit
#define fetch_xor   atomic_fetch_xor_explicit
#define cas_weak    atomic_compare_exchange_weak_explicit
#define cas_strong  atomic_compare_exchange_strong_explicit


/* ------ Task utilities ------ */

#define DO_NOT_EXIT 0
#define task_loop(exit_predicate) while (!(exit_predicate))

/* Data memory barrier */

#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"

#else
#define __DMB() atomic_thread_fence(AcqRel)

#endif // DMB support


/* ------ FC '26 GPIO port maps ------ */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6


/* ------ ThreadX API includes ------ */

#include "tx_api.h"
#include "tx_port.h"
#include "FC-Threads.h"


/* ------ Telemetry API abstraction ------ */

#include <sedsprintf.h>
#include "telemetry.h"

#define MESSAGE_BATCHING_ENABLED -1

#define log_msg_sync(msg, size)                             \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,           \
                            (msg), (size), sizeof(char))

#define log_msg(msg, size)                                  \
  log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,          \
                             (msg), (size), sizeof(char))

#define log_measurement(type, buf)                          \
  log_telemetry_asynchronous((type), (buf), 3, sizeof(float));

#define log_ukf_data(buf, size)                             \
  log_telemetry_asynchronous(SEDS_DT_KALMAN_FILTER_DATA,    \
                             (buf), (size), sizeof(float));

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...) die(fmt __VA_OPT__(,) __VA_ARGS__)

#else
#if defined(__GNUC__)

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

#endif // GNU C
#endif // >= C23

/* Ignition request from the Valve board over telemetry */

#define IGNITION_COMMAND 1

#define request_ignition()                                  \
  ({                                                        \
    uint8_t vcmd = IGNITION_COMMAND;                        \
    log_telemetry_synchronous(SEDS_DT_VALVE_COMMAND,        \
                              &vcmd, 1, sizeof(uint8_t));   \
  })


/* ------ HAL Aliases ------ */

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_dcache.h"
#include "core_cm33.h"

extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;

/* Get currect tick for custom timer */

#define hal_time_ms() HAL_GetTick()

/* Parachute deployment functions */

#define co2_low()                                           \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define co2_high()                                          \
  ({                                                        \
    HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET);    \
    /* Always guarantee all tasks observe PYRO fire */      \
    fetch_or(&config, SAFE_EXPAND_REEF, Rel);               \
  })

#define reef_low()                                          \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define reef_high()                                         \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_SET)

/* Data cache calls */

#define invalidate_dcache() HAL_DCACHE_Invalidate(&hdcache1)

#define invalidate_dcache_addr_int(buf, size)         \
  (HAL_DCACHE_InvalidateByAddr_IT((&hdcache1),        \
                                  (uint32_t *)(buf),  \
                                  (size)))

#define clean_dcache_addr_int(buf, size)              \
  (HAL_DCACHE_CleanByAddr_IT((&hdcache1),             \
                             (uint32_t *)(buf),       \
                             (size)))

/* DMA transmit-receive */

#define dma_spi_txrx(txbuf, rxbuf, size)              \
  (HAL_SPI_TransmitReceive_DMA((&hspi1), (txbuf),     \
                               (rxbuf), (size)))


/* ------ Sensor drivers and data collection ------ */

#include "gyro.h"
#include "accel.h"
#include "barometer.h"
#include "dma.h"

#define init_baro() init_barometer(&hspi1)
#define init_gyro() gyro_init(&hspi1)
#define init_accel() accel_init(&hspi1)

#define finish_transfer(device) \
  ({                            \
    switch (device) {           \
      case BAROMETER:           \
        BARO_CS_HIGH(); break;  \
      case GYROSCOPE:           \
        GYRO_CS_HIGH(); break;  \
      case ACCELEROMETER:       \
        ACCEL_CS_HIGH(); break; \
      default: return;          \
    }                           \
  })

#define terminate_transfers() \
  ({                          \
    BARO_CS_HIGH();           \
    GYRO_CS_HIGH();           \
    ACCEL_CS_HIGH();          \
  })

/* Identification bytes for DMA Tx buffers */

#define BARO_TX_BYTE  ((uint8_t)(BARO_DATA_0 | BMP390_SPI_READ_BIT))
#define GYRO_TX_BYTE  ((uint8_t)(GYRO_CMD_READ(GYRO_RATE_X_LSB)))
#define ACCEL_TX_BYTE ((uint8_t)(ACCEL_CMD_READ(ACCEL_X_LSB)))

/* Peripheral sensor EXT interrupt pins */

#define ACCEL_INT_PIN_1 GPIO_PIN_4
#define ACCEL_INT_PIN_2 GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

/* Driver-specific data conversions */

#define U32(b0, b1, b2, b3)                                         \
  (((uint32_t)(b2) << 24) | ((uint32_t)(b2) << 16) |                \
   ((uint32_t)(b1) << 8 ) |  (uint32_t)(b0))

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))


/* ------ On-board relative timer implementation ------ */

enum fc_timer {
  Predict,
  Recovery_FC,
  Recovery_GND,

  Time_Users
};

/// Last recorded time for each UKF timer user.
/// Defined in recovery.c to avoid multiple linkage.
/// u32 wrap is not handled (flight assumed < 49 days :D).
extern uint32_t local_time[Time_Users];

/// Report time elapsed since last call to either 
/// timer_fetch_update or timer_update,
/// and set local time to current HAL tick (ms).
static inline uint32_t timer_fetch_update(enum fc_timer u)
{
  uint32_t prev = local_time[u];
  local_time[u] = hal_time_ms();
  return local_time[u] - prev;
}

/// Set local time to current HAL tick (ms).
static inline void timer_update(enum fc_timer u)
{
  local_time[u] = hal_time_ms();
}

/// Report time elapsed since last call to either 
/// timer_fetch_update or timer_update.
static inline uint32_t timer_fetch(enum fc_timer u)
{
  return hal_time_ms() - local_time[u];
}


#endif // PLATFORM_H
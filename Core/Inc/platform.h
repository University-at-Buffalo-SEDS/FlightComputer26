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
 * - Type-specific Max, Min, and Abs helper macros;
 * - GPIO and EXTI port mappings;
 * - ThreadX, HAL, sedsprintf_rs, and driver includes;
 * - Variadic aliases for select sedsprintf_rs functions;
 * - Variadic aliases for substitute stdio functions.
 * - Aliases for select HAL and driver functions;
 * - Shared inlined timer implementation with single storage;
 * - Misc aliases and includes as required by the modules.
 *
 * This header should be included in all high-level Flight Computer
 * logic except for drivers, middleware, and utilized libraries (*),
 * which are considered to be the platform this header abstracts away.
 *
 * (*) Libraries include vendor, bundled, external, and optional,
 *     but exclude RTOS, as the FC makes use of ThreadX scheduling.
 */

#ifndef PLATFORM_H
#define PLATFORM_H


/* ------ Bundled std headers used ------ */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>
#include <string.h>


/* ------ Pre-compilation checks ------ */

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 201112L
  #error "C11 or later required for atomic and generics."
#endif

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L
  #warning "!C23 * !GNUC -> Hacky types and variadics."
#endif

#define CM_PTR 0xFFFFFFFFUL

_Static_assert(UINTPTR_MAX == CM_PTR, "Invalid pointer size.");

#ifndef STM32H523xx
  #error "STM32H523xx series MCU required."
#endif


/* ------ Platform integer aliases ----- */

/* Fast means the _fastest_ integer of minimum width. 
 * Determined by the bundled library. */

typedef uint_fast8_t  fu8;
typedef uint_fast16_t fu16;
typedef uint_fast32_t fu32;
typedef uint_fast64_t fu64;

typedef int_fast8_t  fi8;
typedef int_fast16_t fi16;
typedef int_fast32_t fi32;
typedef int_fast64_t fi64;


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


/* ------ ARM fast math bundled library ------ */

#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"

/* CMSIS matrix aliases for convenience */
typedef arm_matrix_instance_f32 matrix;

/* If used without ARM CC, will degrade to math.h sqrtf(),
  * which will emit a single VSQRT.F32 FPU instruction.
  *
  * developer.arm.com:
  * The-Cortex-M33-Instruction-Set/Floating-point-instructions/VSQRT 
  *
  * There is no such instruction for inverse square root
  * => bithack + Newton-Raphson below to avoid FP division. */
#define vsqrt     arm_sqrt_f32;

/* These functions take pointers to arm_matrix_instance_f32;
 * this is the reason there are wrappers inside KF functions. */
#define chol			arm_mat_cholesky_f32
#define transpose arm_mat_trans_f32
#define xmul 			arm_mat_mult_f32
#define xadd			arm_mat_add_f32
#define xsub      arm_mat_sub_f32
#define xinv      arm_mat_inverse_f32

/* I don't quite get the point of this function.
 * Maybe because it's *floating point*. */
#define xinit     arm_mat_init_f32


/* ------ Numerical helpers ------ */

#include <math.h>

static inline fu8  max_fu8 (fu8  a, fu8  b) { return a > b ? a : b; }
static inline fu16 max_fu16(fu16 a, fu16 b) { return a > b ? a : b; }
static inline fu32 max_fu32(fu32 a, fu32 b) { return a > b ? a : b; }

static inline fu8  min_fu8 (fu8 a, fu8 b)   { return a < b ? a : b; }
static inline fu16 min_fu16(fu16 a, fu16 b) { return a < b ? a : b; }
static inline fu32 min_fu32(fu32 a, fu32 b) { return a < b ? a : b; }

static inline fi8  abs_fi8 (fi8  a) { return a < 0 ? -a : a; }
static inline fi16 abs_fi16(fi16 a) { return a < 0 ? -a : a; }
static inline fi32 abs_fi32(fi32 a) { return a < 0 ? -a : a; }

#define Max(x, y)       \
  _Generic((x),         \
    fu8:  max_fu8,      \
    fu16: max_fu16,     \
    fu32: max_fu32,     \
    float: fmaxf        \
    default: fmaxf      \
  ) (x, y)

#define Min(x, y)       \
  _Generic((x),         \
    fu8:  min_fu8,      \
    fu16: min_fu16,     \
    fu32: min_fu32,     \
    float: fminf        \
    default: fminf      \
  ) (x, y)

#define Abs(x)          \
  _Generic((x),         \
    fi8:  abs_fi8,      \
    fi16: abs_fi16,     \
    fi32: abs_fi32,     \
    float: fabsf        \
    default: fabsf      \
  ) (x)


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


/* ------ Type attributes ------ */

#define serial __attribute__((packed, aligned(4)))

#define tx_align __attribute__((aligned(sizeof(ULONG))))


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

#define co2_low()                                               \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define co2_high()                                              \
  do {                                                          \
    HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET);        \
    /* Always guarantee all tasks observe PYRO fire */          \
    fetch_or(&config, static_option(Parachute_Deployed), Rel);  \
  } while (0)

#define reef_low()                                              \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define reef_high()                                             \
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

#define init_baro() init_barometer(&hspi1)
#define init_gyro() gyro_init(&hspi1)
#define init_accl() accel_init(&hspi1)

#define baro_comp_temp(temp) \
  compensate_temperature((uint32_t)temp)

#define baro_comp_pres(pres) \
  compensate_pressure((uint32_t)pres)

#define baro_calc_alt(pres) \
  compute_relative_altitude((float)pres)

#define finish_transfer(device) \
  do {                          \
    switch (device) {           \
      case Sensor_Baro:         \
        BARO_CS_HIGH(); break;  \
      case Sensor_Gyro:         \
        GYRO_CS_HIGH(); break;  \
      case Sensor_Accl:         \
        ACCEL_CS_HIGH(); break; \
      default: return;          \
    }                           \
  } while (0)

#define terminate_transfers() \
  do {                        \
    BARO_CS_HIGH();           \
    GYRO_CS_HIGH();           \
    ACCEL_CS_HIGH();          \
  } while (0)

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
  (((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) |                \
   ((uint32_t)(b1) << 8 ) |  (uint32_t)(b0))

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))


/* ------ Telemetry API abstraction ------ */

#ifdef TELEMETRY_ENABLED

#include <sedsprintf.h>
#include "telemetry.h"

#define log_msg_sync(msg, size)                             \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,           \
                            (msg), (size), sizeof(char))

#define log_msg(msg, size)                                  \
  log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,          \
                             (msg), (size), sizeof(char))

#define log_measurement(type, buf)                          \
  log_telemetry_asynchronous((type), (buf), 3, sizeof(float));

#define log_filter_data(buf, size)                          \
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

/* Ground Station repo: backend/src/rocket_commands.rs
 * pub enum ActuatorBoardCommands -> IgniterSequence */
#define IGNITION_COMMAND 13

static inline SedsResult request_ignition()
{
  uint8_t vcmd = IGNITION_COMMAND;
  return log_telemetry_synchronous(SEDS_DT_VALVE_COMMAND,
                                   &vcmd, 1, sizeof(uint8_t));
}

#else /* Log to terminal emulator */

#define SEDS_OK 0

#define SEDS_DT_BAROMETER_DATA "Barometer"
#define SEDS_DT_GYRO_DATA      "Gyroscope"
#define SEDS_DT_ACCEL_DATA     "Accelerometer"

#include <stdio.h>

#define log_msg_sync(msg, size) printf("\n%s\n", (msg))

#define log_msg log_msg_sync

#define log_measurement(type, buf)                            \
  do {                                                        \
    printf("Measurement: " type "\n");                        \
    fwrite((buf), sizeof(float), 3, stdout);                  \
    putchar('\n');                                            \
  } while (0)

#define log_filter_data(buf, size)                            \
  do {                                                        \
    printf("State vector:\n");                                \
    fwrite((buf), sizeof(float), (size), stdout);             \
    putchar('\n');                                            \
  } while (0)

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                                \
  fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...)                                     \
  do {                                                        \
    while (1) {                                               \
      fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__);         \
      HAL_Delay(1000);                                        \
    }                                                         \
  } while (0)

#else
#if defined(__GNUC__)

#define log_err_sync(fmt, ...)                                \
  fprintf(stderr, fmt, ##__VA_ARGS__)

#define log_die(fmt, ...)                                     \
  do {                                                        \
    while (1) {                                               \
      fprintf(stderr, fmt, ##__VA_ARGS__);                    \
      HAL_Delay(1000);                                        \
    }                                                         \
  } while (0)

#else /* Does not support 0 variadic arguments */

#define log_err_sync(fmt, ...)                                \
  fprintf(stderr, fmt __VA_ARGS__)

#define log_die(fmt, ...)                                     \
  do {                                                        \
    while (1) {                                               \
      fprintf(stderr, fmt, __VA_ARGS__);                      \
      HAL_Delay(1000);                                        \
    }                                                         \
  } while (0)

#endif // GNU C
#endif // >= C23

#define log_err log_err_sync

#define request_ignition()                                    \
  ( (void)( printf("Ignition requested.\n") ), SEDS_OK )

#endif // TELEMETRY_ENABLED


/* ------ On-board SD card (conditional) ------ */

#ifdef SD_AVAILABLE

#include "sd_card.h"
#include "fx_stm32_sd_driver.h"

#endif // SD_AVAILABLE


#endif // PLATFORM_H
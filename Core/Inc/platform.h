/*
 * Shared platform header
 *
 * The purpose of this file is to unify external APIs
 * used by the Flight Computer, providing a single point
 * of reference and modification, easier conditional
 * compilation and poisoning.
 *
 * This header provides the following components for the
 * DMA, Kalman, Distribution, Evaluation, and Recovery modules:
 *
 * - GPIO and EXTI port mappings;
 * - ThreadX, HAL, sedsprintf_rs, and driver includes;
 * - Variadic aliases for select sedsprintf_rs functions;
 * - Variadic aliases for substitute stdio functions;
 * - Aliases for select HAL and driver functions;
 * - Misc aliases and includes as required by the modules.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdio.h>                      // IWYU pragma: export
#include <string.h>                     // IWYU pragma: export
#include <assert.h>                     // IWYU pragma: export
#include <math.h>                       // IWYU pragma: export

#include "dsp/fast_math_functions.h"    // IWYU pragma: export
#include "dsp/matrix_functions.h"       // IWYU pragma: export

#include "tx_api.h"                     // IWYU pragma: export
#include "tx_port.h"                    // IWYU pragma: export

#include "stm32h5xx.h"                  // IWYU pragma: export
#include "stm32h5xx_hal.h"              // IWYU pragma: export
#include "stm32h5xx_hal_def.h"          // IWYU pragma: export
#include "stm32h5xx_hal_spi.h"          // IWYU pragma: export
#include "stm32h5xx_hal_gpio.h"         // IWYU pragma: export
#include "stm32h5xx_hal_dcache.h"       // IWYU pragma: export
#include "core_cm33.h"                  // IWYU pragma: export

#include "main.h"                       // IWYU pragma: export
#include "barometer.h"                  // IWYU pragma: export
#include "gyroscope.h"                  // IWYU pragma: export
#include "accelerometer.h"              // IWYU pragma: export


/* Checks are applicable only to this non-generic header */

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 201112L
  #error "C11 or later required for atomic and generics."
#endif

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L
  #error "This platform requires GNU C11 or ISO C23."
#endif

#define CM_PTR 0xFFFFFFFFUL

static_assert(UINTPTR_MAX == CM_PTR, "32-bit MCU required.");

#ifndef STM32H523xx
  #error "STM32H523xx series MCU required."
#endif


/* ARM CMSIS */

typedef arm_status math_status;
typedef arm_matrix_instance_f32 matrix;

#define fvsqrt        arm_sqrt_f32
#define fatan2        arm_atan2_f32
#define fcos          arm_cos_f32
#define fsin          arm_sin_f32

#define chol_lotri		arm_mat_cholesky_f32
#define mtranspose    arm_mat_trans_f32
#define matrix_mul    arm_mat_mult_f32
#define matrix_add		arm_mat_add_f32
#define matrix_sub    arm_mat_sub_f32
#define matrix_inv    arm_mat_inverse_f32
#define matrix_scl    arm_mat_scale_f32
#define matvec_mul    arm_mat_vec_mult_f32


/* Misc utilities */

#define DO_NOT_EXIT 0

#define task_loop(exit_predicate) while (!(exit_predicate))

#define popcount(mask) (fu16)__builtin_popcount((unsigned)(mask))

extern void *_sbrk(ptrdiff_t);

#if defined(__ARMCC_VERSION) || defined(__GNUC__)

#include "cmsis_compiler.h" // IWYU pragma: export

#else

#define __DMB() atomic_thread_fence(memory_order_acqrel)

#endif


/* HAL */

extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;

#define now_ms() HAL_GetTick()

#define irq_off(irq) HAL_NVIC_DisableIRQ(irq)
#define irq_on(irq)  HAL_NVIC_EnableIRQ(irq)

#define invalidate_dcache() HAL_DCACHE_Invalidate(&hdcache1)

#define invalidate_dcache_addr_int(buf, size)           \
  (HAL_DCACHE_InvalidateByAddr_IT((&hdcache1),          \
                                  (uint32_t *)(buf),    \
                                  (size)))

#define clean_dcache_addr_int(buf, size)                \
  (HAL_DCACHE_CleanByAddr_IT((&hdcache1),               \
                             (uint32_t *)(buf),         \
                             (size)))

#define dma_spi_txrx(txbuf, rxbuf, size)                \
  (HAL_SPI_TransmitReceive_DMA((&hspi1), (txbuf),       \
                               (rxbuf), (size)))

#define gpio_cs_low(sens)                               \
  HAL_GPIO_WritePin((GPIO_TypeDef *)gpio.port[sens],    \
                    gpio.pin[sens], GPIO_PIN_RESET)

#define gpio_cs_high(sens)                              \
  HAL_GPIO_WritePin((GPIO_TypeDef *)gpio.port[sens],    \
                    gpio.pin[sens], GPIO_PIN_SET)


/* Sensors */

#define terminate_transfers() \
  do {                        \
    baro_cs_high();           \
    gyro_cs_high();           \
    accl_cs_high();           \
  } while (0)

#define BARO_CS_PORT CS_BARO_GPIO_Port
#define GYRO_CS_PORT CS_GYRO_GPIO_Port
#define ACCL_CS_PORT CS_ACCEL_GPIO_Port

#define BARO_CS_PIN CS_BARO_Pin
#define GYRO_CS_PIN CS_GYRO_Pin
#define ACCL_CS_PIN CS_ACCEL_Pin

#define BARO_TX_BYTE ((uint8_t)(BARO_DATA_0 | BARO_SPI_READ_BIT))
#define GYRO_TX_BYTE ((uint8_t)(gyro_cmd_read(GYRO_RATE_X_LSB)))
#define ACCL_TX_BYTE ((uint8_t)(accl_cmd_read(ACCL_X_LSB)))

#define ACCL_INT_PIN_1  GPIO_PIN_4
#define ACCL_INT_PIN_2  GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

#define U32(b0, b1, b2, b3)                                         \
  (((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) |                \
   ((uint32_t)(b1) << 8 ) |  (uint32_t)(b0))

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))

#define SPI1_GLOBAL_IRQ   SPI1_IRQn
#define DMA_RECEIVER_SPI1 GPDMA1_Channel0_IRQn

#define Baro_EXTI   EXTI7_IRQn
#define Gyro_EXTI_1 EXTI0_IRQn
#define Gyro_EXTI_2 EXTI1_IRQn
#define Accl_EXTI_1 EXTI4_IRQn
#define Accl_EXTI_2 EXTI5_IRQn


/* Deployment */

#define co2_low()                                               \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_RESET)

#define co2_high()                                              \
  HAL_GPIO_WritePin(PYRO_PORT, CO2_PIN, GPIO_PIN_SET)

#define reef_low()                                              \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_RESET)

#define reef_high()                                             \
  HAL_GPIO_WritePin(PYRO_PORT, REEF_PIN, GPIO_PIN_SET)

#define toggle_green_led()                                      \
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin)

  
/* Telemetry */

#ifdef TELEMETRY_ENABLED

#include <sedsprintf.h> // IWYU pragma: export
#include "telemetry.h"  // IWYU pragma: export

extern void telemetry_set_byte_pool(TX_BYTE_POOL *pool);
extern void telemetry_init_lock(void);

#define log_msg_sync(msg, size)                               \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,             \
                            (msg), (size), sizeof(char))

#define log_msg(msg)                                          \
  log_telemetry_string_asynchronous(SEDS_DT_MESSAGE_DATA,     \
                                    (msg))

#define log_critical(msg)                                     \
  log_telemetry_string_asynchronous(SEDS_DT_ORDERED_MESSAGE,  \
                                    (msg))

#define log_valve_board_command(cmd)                          \
  log_telemetry_asynchronous(SEDS_DT_VALVE_COMMAND,           \
                             &(cmd), 1, sizeof(uint8_t))      \

#define log_flight_state(state)                               \
  log_telemetry_asynchronous(SEDS_DT_FLIGHT_STATE,            \
                             (const void *)&(state),          \
                             1, sizeof(uint8_t))              \

#define log_measm(k, buf)                                     \
  log_telemetry_asynchronous((k), (buf), 3, sizeof(float))

#define log_ascent_state(buf)                                 \
  log_telemetry_asynchronous(SEDS_DT_ASCENT_STATE, (buf),     \
                             EKF_STATE, sizeof(float))

#define log_descent_state(buf)                                \
  log_telemetry_asynchronous(SEDS_DT_DESCENT_STATE, (buf),    \
                             DKF_STATE, sizeof(float))

#define log_euler_angles(buf)                                 \
  log_telemetry_asynchronous(SEDS_DT_EULER_ANGLES, (buf),     \
                             3, sizeof(float))

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                                \
  log_error_synchronous(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define log_err(fmt, ...)                                     \
  log_error_asynchronous(fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...) die(fmt __VA_OPT__(,) __VA_ARGS__)

#else /* !C 23 */
#ifdef __GNUC__

#define log_err_sync(fmt, ...)                                \
  log_error_synchronous(fmt, ##__VA_ARGS__)

#define log_err(fmt, ...)                                     \
  log_error_asynchronous(fmt, ##__VA_ARGS__)

#define log_die(fmt, ...) die(fmt, ##__VA_ARGS__)

#endif /* GNUC */
#endif /* C 23 */

#else /* !TELEMETRY_ENABLED */

#define SEDS_OK  0
#define SEDS_ERR 1

#define SEDS_DT_ASCENT_BIASES  2
#define SEDS_DT_BAROMETER_DATA "Barometer"
#define SEDS_DT_GYRO_DATA      "Gyroscope"
#define SEDS_DT_ACCEL_DATA     "Accelerometer"

#ifdef USB_ENUMERATES

#define log_msg_sync(msg, size) printf("\n%s\n", (msg))

#define log_valve_board_command(cmd)                          \
  ( (void)( printf("Valve cmd sent: %u\n", cmd) ), SEDS_OK )

#define log_flight_state(state)                               \
  printf("Flight state propagated to %u\n", state)

#define log_measm(type, buf)                                  \
  do {                                                        \
    printf("Measurement: %s\n", #type);                       \
    fwrite((buf), sizeof(float), 3, stdout);                  \
    putchar('\n');                                            \
  } while (0)

#define log_ascent_state(buf)                                 \
  do {                                                        \
    printf("Ascent state:\n");                                \
    fwrite((buf), sizeof(float), EKF_STATE, stdout);          \
    putchar('\n');                                            \
  } while (0)

#define log_descent_state(buf)                                \
  do {                                                        \
    printf("Descent state:\n");                               \
    fwrite((buf), sizeof(float), DKF_STATE, stdout);          \
    putchar('\n');                                            \
  } while (0)

#define log_euler_angles(buf)                                 \
  do {                                                        \
    printf("Euler angles:\n");                                \
    fwrite((buf), sizeof(float), 3, stdout);                  \
    putchar('\n');                                            \
  } while (0)

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                                \
  fprintf(stderr, fmt "\n" __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...)                                     \
  do {                                                        \
    while (1) {                                               \
      fprintf(stderr, fmt "\n" __VA_OPT__(,) __VA_ARGS__);    \
      HAL_Delay(1000);                                        \
    }                                                         \
  } while (0)

#else /* !C 23 */
#ifdef __GNUC__

#define log_err_sync(fmt, ...)                                \
  fprintf(stderr, fmt "\n", ##__VA_ARGS__)

#define log_die(fmt, ...)                                     \
  do {                                                        \
    while (1) {                                               \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);               \
      HAL_Delay(1000);                                        \
    }                                                         \
  } while (0)

#endif /* GNUC */
#endif /* C 23 */

#else /* !USB_ENUMERATES */

#pragma GCC diagnostic ignored "-Wunused-variable"

#define log_msg_sync(msg, size) 

#define log_valve_board_command(cmd) (SEDS_OK)

#define log_flight_state(state)

#define log_measm(type, buf) 

#define log_ascent_state(buf) 
#define log_descent_state(buf) 

#define log_euler_angles(buf)

#define log_err_sync(fmt, ...) 
#define log_die(fmt, ...) Error_Handler()

#endif /* USB_ENUMERATES */

#define log_msg(msg) log_msg_sync(msg, 0)
#define log_err log_err_sync
#define log_critical log_msg

#endif /* TELEMETRY_ENABLED */


/* SD */

#ifdef SD_AVAILABLE

#include "sd_card.h"            // IWYU pragma: export
#include "fx_stm32_sd_driver.h" // IWYU pragma: export

#endif /* SD_AVAILABLE */


#endif /* PLATFORM_H */
/*
 * Platform abstraction header (HAL, Drivers, ThreadX, Telemetry).
 * Used for the embedded (target) profile.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/* Numerical helpers with double-eval safety */

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

/* FC '26 GPIO port maps */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6

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
#include "core_cm33.h"

extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;

/* Get currect tick for custom timer */

#define hal_time_ms() HAL_GetTick()

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

#define invalidate_dcache_addr_int(buf, size)         \
  (HAL_DCACHE_InvalidateByAddr_IT((&hdcache1),        \
                                  (uint32_t *)(buf),  \
                                  (size)))

#define clean_dcache_addr_int(buf, size)              \
  (HAL_DCACHE_CleanByAddr_IT((&hdcache1),             \
                             (uint32_t *)(buf),       \
                             (size)))

/* Sensor drivers and data collection */

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

/* DMA transmit-receive */

#define dma_spi_txrx(txbuf, rxbuf, size)              \
  (HAL_SPI_TransmitReceive_DMA((&hspi1), (txbuf),     \
                               (rxbuf), (size)))

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

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))

/* Data memory barrier
 * (#else branch is to be invented :D) */

#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"
#else
#define __DMB() (void)0
#endif // DMB support


#endif // PLATFORM_H
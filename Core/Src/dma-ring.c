/**
 * Logic related to DMA and ring buffer.
 */

#include <sedsprintf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_dcache.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_def.h"
#include "core_cm33.h"

#include "dma-ring.h"
#include "telemetry.h"
#include "barometer.h"
#include "gyro.h"

// HAL handles from main.c.
extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;

// DMA buffers for each sensor. Note: HAL expects non-volatile buffers.
// Tx: Producer: (final literal value). Consumer: DMA peripheral.
// Rx: Producer: DMA peripheral. Consumer: ReceiveComplete callback.
static uint8_t baro_dma_tx[BMP390_BUF_SIZE] = {[0] = (uint8_t)(BARO_DATA_0 | BMP390_SPI_READ_BIT)};
static uint8_t baro_dma_rx[BMP390_BUF_SIZE];
static uint8_t gyro_dma_tx[GYRO_BUF_SIZE] = {[0] = (uint8_t)(GYRO_CMD_READ(GYRO_RATE_X_LSB))};
static uint8_t gyro_dma_rx[GYRO_BUF_SIZE];
// uint8_t accel_dma_tx[ACCEL_BUF_SIZE] = {[0] = (uint8_t)(ACCEL_FIRST_DATA_ADDRESS | ACCEL_SPI_READ_BIT)};
// uint8_t accel_dma_rx[ACCEL_BUF_SIZE];

// Ring buffer. Producer: ReceiveComplete callback. Consumer: dequeue_and_send_next().
static volatile payload_t ring[RING_SIZE];
static volatile atomic_uint_fast16_t head = ATOMIC_VAR_INIT(0);
static volatile atomic_uint_fast16_t tail = ATOMIC_VAR_INIT(0);

// Producer: dequeue_and_send_next(). Consumer: sedsprintf::log_telemetry_asynchronous.
static payload_t buf = {.type = NONE, .data = {0}};
static volatile atomic_bool lib_busy = ATOMIC_VAR_INIT(false);

// Device to expect next time ReceiveComplete is invoked by HAL.
static volatile atomic_uint_fast8_t next = ATOMIC_VAR_INIT(NONE);
static uint_fast8_t expected = ATOMIC_VAR_INIT(NONE); // Constant to user

/**
 * @brief Calls compensation function and passes data to the library.
 * @param None
 * @retval SedsResult, passed from library to caller.
 */
static inline SedsResult send_dequeued() {
  SedsResult st;
  atomic_store_explicit(&lib_busy, true, memory_order_release);

  switch (buf.type) {
    case BAROMETER:
    {
      buf.data.baro.temp = compensate_temperature(buf.data.baro.temp);
      buf.data.baro.pres = compensate_pressure(buf.data.baro.pres);
      buf.data.baro.alt = compute_relative_altitude(buf.data.baro.pres);
      st = log_telemetry_asynchronous(SEDS_DT_BAROMETER_DATA, &buf.data.baro, 3, sizeof(float));
    }
    case GYROSCOPE:
    {
      st = log_telemetry_asynchronous(SEDS_DT_GYRO_DATA, &buf.data.gyro, 3, sizeof(uint16_t));
    }
    case ACCELEROMETER:
    {
      // return log_telemetry_asynchronous(SEDS_DT_ACCELEROMETER_DATA, &buf.data, 3, sizeof(float));
    }
    case NONE: st = SEDS_ERR;
  }

  atomic_store_explicit(&lib_busy, false, memory_order_release);
  return st;
}

/**
 * @brief Enqueues data and signals transfer completion.
 * @param type
 * @retval None
 */
static inline void enqueue(const expected_e type) {
  uint16_t h = atomic_load_explicit(&head, memory_order_relaxed);
  uint16_t t = atomic_load_explicit(&tail, memory_order_acquire);
  uint16_t i = h & RING_MASK;

  // If the ring is full, overwrite oldest entry to keep slow consumer updated.
  if (((h + 1) & RING_MASK) == (t & RING_MASK))
    atomic_store_explicit(&tail, t + 1, memory_order_release);

  switch (type) {
    case BAROMETER:
    {
      HAL_DCACHE_InvalidateByAddr_IT(&hdcache1, (uint32_t *)baro_dma_rx, BMP390_BUF_SIZE); 

      ring[i].type = BAROMETER;
      ring[i].data.baro.pres = u24(baro_dma_rx[1], baro_dma_rx[2], baro_dma_rx[3]);
      ring[i].data.baro.temp = u24(baro_dma_rx[4], baro_dma_rx[5], baro_dma_rx[6]);

      BARO_CS_HIGH();
      break;
    }
    case GYROSCOPE:
    {
      ring[i].type = GYROSCOPE;
      ring[i].data.gyro.rate_x = (int16_t)((uint16_t)gyro_dma_rx[2] << 8 | gyro_dma_rx[1]);
      ring[i].data.gyro.rate_y = (int16_t)((uint16_t)gyro_dma_rx[4] << 8 | gyro_dma_rx[3]);
      ring[i].data.gyro.rate_z = (int16_t)((uint16_t)gyro_dma_rx[6] << 8 | gyro_dma_rx[5]);

      GYRO_CS_HIGH();
      break;
    }
    case ACCELEROMETER:
    {
      break;
    }
    case NONE: return;
  }

  atomic_store_explicit(&head, h + 1, memory_order_release);
}

/**
 * @brief Single function to dequeue and pass data to the library.
 * @param None
 * @retval A SedsResult indicating success or specific error.
 */
inline SedsResult dequeue_and_send_next() {
  uint16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint16_t t = atomic_load_explicit(&tail, memory_order_relaxed);
  bool st = atomic_load_explicit(&lib_busy, memory_order_acquire);

  // Check if the ring is empty (head is on tail), or if library is busy
  if ((h & RING_MASK) == (t & RING_MASK) || st)
    return SEDS_ERR;

  buf = ring[t & RING_MASK];

  atomic_store_explicit(&tail, t + 1, memory_order_release);
  return send_dequeued();
}

/**
 * @brief Enqueues data and signals transfer completion.
 * @param hspi
 * @retval None
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Lock == HAL_LOCKED)
    return;

  enqueue(atomic_load_explicit(&next, memory_order_acquire));
  atomic_store_explicit(&next, NONE, memory_order_release);
}

/**
 * @brief Cleans cache for a device that produced an error.
 * @param hspi
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Lock == HAL_LOCKED)
    return;

  expected_e t = atomic_load_explicit(&next, memory_order_acquire);
  atomic_store_explicit(&next, NONE, memory_order_release);

  switch (t) {
    case BAROMETER:
    {
      HAL_DCACHE_CleanByAddr_IT(&hdcache1, (uint32_t *)baro_dma_rx, BMP390_BUF_SIZE);
      BARO_CS_HIGH();
      break;
    }
    case GYROSCOPE:
    {
      HAL_DCACHE_CleanByAddr_IT(&hdcache1, (uint32_t *)gyro_dma_rx, GYRO_BUF_SIZE);
      GYRO_CS_HIGH();
      break;
    }
    case ACCELEROMETER:
    {
      // HAL_DCACHE_CleanByAddr_IT(&hdcache1, (uint32_t *)accel_dma_rx, ACCEL_BUF_SIZE);
      // ACCEL_CS_HIGH();
      break;
    }
    case NONE: return;
  }
}

/**
 * @brief Interrupt used to trigger DMA transfer
 * @param GPIO_Pin
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  HAL_StatusTypeDef st;

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, BAROMETER))
        return;

      BARO_CS_LOW();
      st = HAL_SPI_TransmitReceive_DMA(&hspi1, baro_dma_tx, baro_dma_rx, BMP390_BUF_SIZE);

      if (st != HAL_OK) {
        BARO_CS_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, GYROSCOPE))
        return;

      GYRO_CS_LOW();
      st = HAL_SPI_TransmitReceive_DMA(&hspi1, gyro_dma_tx, gyro_dma_rx, GYRO_BUF_SIZE);

      if (st != HAL_OK) {
        GYRO_CS_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, ACCELEROMETER))
        return;

      // ACCEL_CS_LOW();
      // st = HAL_SPI_TransmitReceive_DMA(hspi, accel_dma_tx, accel_dma_rx, ACCEL_BUF_SIZE);

      if (st != HAL_OK) {
        // ACCEL_CS_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    default: return;
  }
}
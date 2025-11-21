/**
 * Logic related to DMA and ring buffer.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_dcache.h"
#include "core_cm33.h"

#include "dma-ring.h"
#include "barometer.h"
#include "gyro.h"
#include "accel.h"

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
static uint8_t accel_dma_tx[ACCEL_BUF_SIZE] = {[0] = (uint8_t)(ACCEL_CMD_READ(ACCEL_X_LSB))};
static uint8_t accel_dma_rx[ACCEL_BUF_SIZE];

// Ring buffer. Producer: ReceiveComplete callback. Consumers: Sensor and Kalman tasks.
static volatile payload_t ring[RING_SIZE];
static volatile atomic_uint_fast16_t head = ATOMIC_VAR_INIT(0);
static volatile atomic_uint_fast16_t tail = ATOMIC_VAR_INIT(0);

// Device to expect next time ReceiveComplete is invoked by HAL.
static volatile atomic_uint_fast8_t next = ATOMIC_VAR_INIT(NONE);
static uint_fast8_t expected = ATOMIC_VAR_INIT(NONE);

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
      ring[i].data.baro[0] = u24(baro_dma_rx[1], baro_dma_rx[2], baro_dma_rx[3]);
      ring[i].data.baro[1] = u24(baro_dma_rx[4], baro_dma_rx[5], baro_dma_rx[6]);

      BARO_CS_HIGH();
      break;
    }
    case GYROSCOPE:
    {
      HAL_DCACHE_InvalidateByAddr_IT(&hdcache1, (uint32_t *)gyro_dma_rx, GYRO_BUF_SIZE); 

      ring[i].type = GYROSCOPE;
      ring[i].data.gyro[0] = (int16_t)((uint16_t)gyro_dma_rx[2] << 8 | gyro_dma_rx[1]);
      ring[i].data.gyro[1] = (int16_t)((uint16_t)gyro_dma_rx[4] << 8 | gyro_dma_rx[3]);
      ring[i].data.gyro[2] = (int16_t)((uint16_t)gyro_dma_rx[6] << 8 | gyro_dma_rx[5]);

      GYRO_CS_HIGH();
      break;
    }
    case ACCELEROMETER:
    {
      HAL_DCACHE_InvalidateByAddr_IT(&hdcache1, (uint32_t *)accel_dma_rx, ACCEL_BUF_SIZE);

      ring[i].type = ACCELEROMETER;
      ring[i].data.accel[0] = (float)((accel_dma_rx[3] << 8) | accel_dma_rx[2]);
      ring[i].data.accel[1] = (float)((accel_dma_rx[5] << 8) | accel_dma_rx[4]);
      ring[i].data.accel[2] = (float)((accel_dma_rx[7] << 8) | accel_dma_rx[6]);

      ACCEL_CS_HIGH();
      break;
    }
    case NONE: return;
  }

  atomic_store_explicit(&head, h + 1, memory_order_release);
}

/**
 * @brief Copy but not dequeue the oldest element of the ring.
 * @param None
 * @retval A boolean indicating success or that the ring is empty.
 */
inline bool dma_ring_copy_oldest(payload_t *buf) {
  uint16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint16_t t = atomic_load_explicit(&tail, memory_order_relaxed);
  uint16_t i = t & RING_MASK;

  if ((h & RING_MASK) == i)
    return false;

  *buf = ring[i];
  return true;
}

/**
 * @brief Dequeue the oldest element of the ring and advance tail.
 * @param None
 * @retval A boolean indicating success or that the ring is empty.
 */
inline bool dma_ring_dequeue_oldest(payload_t *buf) {
  uint16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint16_t t = atomic_load_explicit(&tail, memory_order_relaxed);
  uint16_t i = t & RING_MASK;

  // Check if the ring is empty (head is on tail)
  if ((h & RING_MASK) == i)
    return false;

  *buf = ring[i];

  atomic_store_explicit(&tail, t + 1, memory_order_release);
  return true;
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
      HAL_DCACHE_CleanByAddr_IT(&hdcache1, (uint32_t *)accel_dma_rx, ACCEL_BUF_SIZE);
      ACCEL_CS_HIGH();
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

      ACCEL_CS_LOW();
      st = HAL_SPI_TransmitReceive_DMA(&hspi1, accel_dma_tx, accel_dma_rx, ACCEL_BUF_SIZE);

      if (st != HAL_OK) {
        ACCEL_CS_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    default: return;
  }
}
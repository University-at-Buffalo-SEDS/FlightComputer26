/*
 * Interrupt-based sensor data collection.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"
#include "dma.h"

/// Control struct and tx/rx buffers.
/// Two shared rx bufs implement double buffering.
static dma_t ctr = {0};

static const uint8_t tx[3][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [1][0] = GYRO_TX_BYTE, [2][0] = ACCEL_TX_BYTE
};
static uint8_t rx[2][SENSOR_BUF_SIZE];

/// Checks whether pointers are equal, or that p points
/// inside any of the two buffers (due to HAL offset).
static inline uint_fast8_t index_from_ptr(uint8_t *p)
{
  if (p == rx[0] || ((uintptr_t)p >= (uintptr_t)rx[0] &&
      (uintptr_t)p < (uintptr_t)rx[0] + SENSOR_BUF_SIZE))
  {
    return 0u;
  }
  else if (p == rx[1] || ((uintptr_t)p >= (uintptr_t)rx[1] &&
           (uintptr_t)p < (uintptr_t)rx[1] + SENSOR_BUF_SIZE))
  {
    return 1u;
  }
  else return DMA_RX_NULL;
}

/// Reads device type for buffer i and pulls the
/// corresponding CS pin high.
static inline void switch_pull_cs_high(uint_fast8_t i)
{
  switch (atomic_load_explicit(&ctr.t[i], memory_order_relaxed)) {
    case BAROMETER:
      BARO_CS_HIGH(); break;
    case GYROSCOPE:
      GYRO_CS_HIGH(); break;
    case ACCELEROMETER:
      ACCEL_CS_HIGH(); break;
    case NONE: return;
  }
}

/// Successful transfer routine that publishes new buffer.
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  uint_fast8_t a = index_from_ptr(hspi->pRxBuffPtr);

  if (a == DMA_RX_NULL) {
    invalidate_dcache_addr_int(rx[0], SENSOR_BUF_SIZE * 2);
    BARO_CS_HIGH();
    GYRO_CS_HIGH();
    ACCEL_CS_HIGH();
    return;
  }

  invalidate_dcache_addr_int(rx[a], SENSOR_BUF_SIZE);
  atomic_store_explicit(&ctr.w, a, memory_order_release);
  switch_pull_cs_high(a);
}

/// Edge case: invalidate cache for both buffers and
/// raise CS pin for all sensors.
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  uint_fast8_t a = index_from_ptr(hspi->pRxBuffPtr);
  
  if (a == DMA_RX_NULL) {
    invalidate_dcache_addr_int(rx[0], SENSOR_BUF_SIZE * 2);
    BARO_CS_HIGH();
    GYRO_CS_HIGH();
    ACCEL_CS_HIGH();
    return;
  }

  invalidate_dcache_addr_int(rx[a], SENSOR_BUF_SIZE);
  switch_pull_cs_high(a);
}

/// Selects a buffer different from currently reading (if any),
/// and attempts to initiate DMA transfer.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_StatusTypeDef st;
  uint_fast8_t a = atomic_load_explicit(&ctr.w, memory_order_acquire);

  /* There is only one consumer, safely switch to another buffer. */
  if (a == atomic_load_explicit(&ctr.r, memory_order_acquire))
    a = !a;

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
      BARO_CS_LOW();
      /*
       * If another transfer is currently in progress, this
       * call will return HAL_BUSY (see try-lock on line 2438
       * of stm32h5xx_hal_spi.c). We should neither update the type
       * nor pull CS high to avoid corrupting active transfer.
       * Only on HAL_OK the type is adjusted, and only on HAL_ERROR
       * (when HAL aborts its side effects) CS is pulled back high.
       */
      st = dma_spi_txrx(tx[0], rx[a], SENSOR_BUF_SIZE);
      if (st == HAL_OK) {
        atomic_store_explicit(&ctr.t[a], BAROMETER, memory_order_relaxed);
      } else if (st == HAL_ERROR) {
        BARO_CS_HIGH();
      }
      break;
    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
      GYRO_CS_LOW();
      st = dma_spi_txrx(tx[1], rx[a], SENSOR_BUF_SIZE);
      if (st == HAL_OK) {
        atomic_store_explicit(&ctr.t[a], GYROSCOPE, memory_order_relaxed);
      } else if (st == HAL_ERROR) {
        GYRO_CS_HIGH();
      }
      break;
    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
      ACCEL_CS_LOW();
      st = dma_spi_txrx(tx[2], rx[a], SENSOR_BUF_SIZE);
      if (st == HAL_OK) {
        atomic_store_explicit(&ctr.t[a], ACCELEROMETER, memory_order_relaxed);
      } else if (st == HAL_ERROR) {
        ACCEL_CS_HIGH();
      }
      break;
    default: break;
  }
}

/// Transforms bits into raw data, storing the
/// result in provided buffer.
dma_e dma_read_latest(payload_t *buf)
{
  if (buf == NULL) return DMA_GENERR;

  uint_fast8_t a = atomic_load_explicit(&ctr.w, memory_order_acquire);
  /*
   * Happens-before is ensured by loading "a" with acquire.
   * We only need to guarantee atomicity of this assignment.
   */
  buf->type = atomic_load_explicit(&ctr.t[a], memory_order_relaxed);
  /*
   * Claim buffer for reading. EXTI guarantees that the buffer
   * for which ctr.w (write) flag is published is not
   * currently in transfer (double buffering).
   */
  atomic_store_explicit(&ctr.r, a, memory_order_release);
  
  dma_e st = DMA_OK;
  switch (buf->type) {
    case BAROMETER:
      buf->data.baro[0] = U24(rx[a][1], rx[a][2], rx[a][3]);
      buf->data.baro[1] = U24(rx[a][4], rx[a][5], rx[a][6]);
      break;
    case GYROSCOPE:
      buf->data.gyro[0] = I16(rx[a][1], rx[a][2]);
      buf->data.gyro[1] = I16(rx[a][3], rx[a][4]);
      buf->data.gyro[2] = I16(rx[a][5], rx[a][6]);
      break;
    case ACCELEROMETER:
      buf->data.accel[0] = F16(rx[a][2], rx[a][3]);
      buf->data.accel[1] = F16(rx[a][4], rx[a][5]);
      buf->data.accel[2] = F16(rx[a][6], rx[a][7]);
      break;
    case NONE: st = DMA_EMPTY;
  }

  /*
   * Unclaim buffer to let EXTI initiate DMA on any buffer.
   */
  atomic_store_explicit(&ctr.r, BUF_UNCLAIMED, memory_order_release);
  return st;
}
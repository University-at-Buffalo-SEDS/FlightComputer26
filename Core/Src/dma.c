/*
 * Direct Memory Access implementation.
 * Sensors: Barometer, Gyroscope, Accelerometer.
 * Consumer: single - sensor task.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "dma.h"
#include "platform.h"


/// Each device data readiness mask for two buffers.
/// Baro, Gyro, Accel, and unused bits (in this order).
/// 0 0 0 0 0 A G B | 0 0 0 0 0 0 A G B
static uint_fast8_t is_complete[2] = {0};

/// Rx buffer marked for accepting DMA transfers.
static atomic_uint_fast8_t in_transfer = 0;

/// Separate Tx buffers for each sensor.
static const uint8_t tx[3][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [1][0] = GYRO_TX_BYTE, [2][0] = ACCEL_TX_BYTE
};

/// Double-buffered Rx for each sensor.
static uint8_t rx[2][3][SENSOR_BUF_SIZE] = {0};


/* ------ Helpers ------ */

/// Determines active buffer and device type from pointer offset.
/// Returns DMA_RX_NULL if p is NULL or does not point inside rx.
/// Context: callbacks.
static inline uint_fast8_t decode_ptr(uint8_t *p, uint_fast8_t *type)
{
  static const uint_fast8_t buf[2 * 3] = {0, 0, 0, 1, 1, 1};
  static const uint_fast8_t dev[2 * 3] = {0, 1, 2, 0, 1, 2};

  if (!p) return DMA_RX_NULL;

  ptrdiff_t offset = p - &rx[0][0][0];
  if (offset < 0 || offset >= sizeof(rx))
    return DMA_RX_NULL;

  /* >> 3 is division by 8 (i.e., SENSOR_BUF_SIZE) */
  uint_fast8_t idx = (uint_fast8_t)offset >> 3;

  *type = dev[idx];
  return buf[idx];
}


/* ------ Callbacks ------ */

/// Finish transfer and publish device data flag.
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  uint_fast8_t t;
  uint_fast8_t i = decode_ptr(hspi->pRxBuffPtr, &t);

  if (i == DMA_RX_NULL) {
    terminate_transfers();
    return;
  }

  invalidate_dcache_addr_int(rx[i][t], SENSOR_BUF_SIZE);
  finish_transfer(t);

  is_complete[i] |= 1u << t;
}

/// Finish transfer but do not publish flag (drop sample).
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  uint_fast8_t t;
  uint_fast8_t i = decode_ptr(hspi->pRxBuffPtr, &t);

  if (i == DMA_RX_NULL) {
    terminate_transfers();
    return;
  }

  finish_transfer(t);
}


/* ------ ISR ------ */

/// Attempts to initialize DMA transfer.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_StatusTypeDef st;
  uint_fast8_t i;
  i = atomic_load_explicit(&in_transfer, memory_order_acquire);

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
      BARO_CS_LOW();
      /*
       * If another transfer is currently in progress, this
       * call will return HAL_BUSY (see try-lock on line 2438
       * of stm32h5xx_hal_spi.c). We should not pull CS high -
       * - one of callbacks will do it on time. Only on HAL_ERROR
       * (when HAL aborts its side effects) CS is pulled back high.
       */
      st = dma_spi_txrx(tx[0], rx[i][0], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR)
        BARO_CS_HIGH();
      break;

    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
      GYRO_CS_LOW();
      st = dma_spi_txrx(tx[1], rx[i][1], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR)
        GYRO_CS_HIGH();
      break;

    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
      ACCEL_CS_LOW();
      st = dma_spi_txrx(tx[2], rx[i][2], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR)
        ACCEL_CS_HIGH();
      break;

    default: break;
  }
}


/* ------ Public API ------ */

/// Fetches whatever is available in a free (tm) DMA buffer.
/// Returns 1 when all 3 sensor buckets have been filled,
/// (accumulates acrosss calls), 0 otherwise, and -1 on bag argument.
int dma_try_fetch(sensor_meas_t *buf)
{
  static uint_fast8_t i = 1;
  static uint_fast8_t cache = 0;

  if (!buf) return -1;

  cache |= is_complete[i];

  if (cache & BARO_DONE) {
    buf->baro.p = U24(rx[i][0][1], rx[i][0][2], rx[i][0][3]);
    buf->baro.t = U24(rx[i][0][4], rx[i][0][5], rx[i][0][6]);
  }
  if (cache & GYRO_DONE) {
    buf->gyro.x = F16(rx[i][1][1], rx[i][1][2]);
    buf->gyro.y = F16(rx[i][1][3], rx[i][1][4]);
    buf->gyro.z = F16(rx[i][1][5], rx[i][1][6]);
  }
  if (cache & ACCL_DONE) {
    buf->accl.x = F16(rx[i][2][2], rx[i][2][3]);
    buf->accl.y = F16(rx[i][2][4], rx[i][2][5]);
    buf->accl.z = F16(rx[i][2][6], rx[i][2][7]);  
  }
  
  is_complete[i] = 0;
  atomic_store_explicit(&in_transfer, i, memory_order_release);
  i ^= 1;

  if (cache & RX_DONE) {
    cache = 0;
    return 1;
  }

  return 0;
}
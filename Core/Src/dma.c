/*
 * Direct Memory Access implementation.
 * Sensors: Barometer, Gyroscope, Accelerometer.
 * Consumer: single - sensor task.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"
#include "dma.h"


/// Each device data readiness mask for two buffers.
/// Baro, Gyro, Accel, and Unused Bit (in this order).
/// B G A 0 | B G A 0
static atomic_uint_fast8_t mask = 0;

/// Rx buffer marked for fetching from task context.
static atomic_uint_fast8_t curr = 0;

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

  uint_fast8_t flag = 1u << (i*4 + t);
  atomic_fetch_or_explicit(&mask, flag, memory_order_release);
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

/// Attempts to initialize DMA transfer on 'curr' buffer.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_StatusTypeDef st;
  uint_fast8_t i = !atomic_load_explicit(&curr, memory_order_acquire);

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

/// Tries to fetch data from DMA rxbuf into provided buffer.
/// Context: sensor task.
dma_e dma_try_fetch(payload_t *buf)
{
  static uint_fast8_t i = 0;

  if (!buf) return DMA_BADARG;

  uint_fast8_t m;
  m = atomic_load_explicit(&mask, memory_order_acquire);

  if ((i == 0 && (m & RX0_DONE) != RX0_DONE) ||
      (i == 1 && (m & RX1_DONE) != RX1_DONE))
  {
    return DMA_WAIT;
  }
  
  buf->baro.alt  = U24(rx[i][0][1], rx[i][0][2], rx[i][0][3]);
  buf->baro.temp = U24(rx[i][0][4], rx[i][0][5], rx[i][0][6]);

  buf->gyro.x = I16(rx[i][1][1], rx[i][1][2]);
  buf->gyro.y = I16(rx[i][1][3], rx[i][1][4]);
  buf->gyro.z = I16(rx[i][1][5], rx[i][1][6]);

  buf->accl.x = F16(rx[i][2][2], rx[i][2][3]);
  buf->accl.y = F16(rx[i][2][4], rx[i][2][5]);
  buf->accl.z = F16(rx[i][2][6], rx[i][2][7]);  

  uint_fast8_t flag = (i) ? 0x0Fu : 0xF0u;
  /* Clear bits for the buffer just read. 
   * Relaxed is used because DMA currently writes to !i,
   * meaning it must observe this 'AND' only after buf switch.*/
  atomic_fetch_and_explicit(&mask, flag, memory_order_relaxed);

  /* Switch 'curr' buffer for the next time.
   * Release is used because only this function writes 'curr'. */
  i = atomic_fetch_xor_explicit(&curr, 1u, memory_order_release);
  return DMA_OK;
}
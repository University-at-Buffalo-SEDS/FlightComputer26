/*
 * Direct Memory Access implementation
 *
 * Sensors:  Barometer, Gyroscope, Accelerometer
 * Consumer: Distribution Task
 *
 * This module implements the HAL Interrupt Service
 * Routine and two DMA transfer callbacks for
 * completion and error handling.
 *
 * The ISR is called by HAL after it receivies a signal
 * on one or two of the designated (by Avionics Hardware)
 * GPIO pins. Depedning on the pin, a DMA transfer is
 * initiated for a corresponding device into one of two
 * receive buffers. The device's CS pin is set low.
 *
 * One such buffer consists of 3 sections (for each device).
 * Every time a completion callback receives, it will:
 *
 * 1. Infer the buffer and section (thus the device type)
 *    by calculating pointer offset from the beginning of
 *    the receive buffer (*).
 * 2. If the pointer matches any valid section, invalidate
 *    data cache for that section and pull the CS pin of
 *    the corresponding device high.
 *    [!] If the offset is beyond the Receive buffers, it
 *        will pull CS pins of ALL device up, thus possibly
 *        terminating active transfers.
 * 3. Set the 'ready' flag for the concerned section (device).
 *
 * The error callback will perform the same sequence of events
 * EXCEPT for invalidating data cache and setting 'ready' flag.
 *
 * The consumer will read whatever is reported as 'ready' by the
 * completion callback into the provided buffer, and THEN (after
 * reading) atomically switch the active buffer using rel/acq
 * semantics on ARM. Such sequence of events avoids waiting for
 * the remaining DMA transfers to complete -- when the caller
 * returns to the buffer previously switched to, all transfers
 * will be completeed and flags set for that buffer. This also
 * avoids using memory orderings on the 'ready' flags, as they
 * are separate for each buffer. A consumer may wish to ignore
 * certain devices on fetch and calibration; it is achieved
 * through passing a mask OR-ed with device flags as per dma.h.
 *
 * If, on any call, the fetch function determines that it filled
 * all sections demanded by consumer of the provided argument buffer,
 * it will report this to the caller, allowing it to proceed.
 */

#include "platform.h"
#include "dma.h"


/// Each device data readiness mask for two buffers.
/// Baro, Gyro, Accel, and unused bits (in this order).
/// 0 0 0 0 0 A G B | 0 0 0 0 0 0 A G B
static volatile fu8 is_complete[2] = {0};

/// Rx buffer marked for accepting DMA transfers.
static atomic_uint_fast8_t in_transfer = 0;

/// Separate Tx buffers for each sensor.
static const uint8_t tx[Sensors][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [1][0] = GYRO_TX_BYTE, [2][0] = ACCEL_TX_BYTE
};

/// Double-buffered Rx for each sensor.
static volatile uint8_t rx[2][Sensors][SENSOR_BUF_SIZE] = {0};


/* ------ Helpers ------ */

/// Determines active buffer and device type from pointer offset.
/// Returns DMA_RX_NULL if p is NULL or does not point inside rx.
/// Context: callbacks.
static inline fu8
decode_ptr(uint8_t *p, fu8 *type)
{
  static const fu8 buf[2 * Sensors] = {0, 0, 0, 1, 1, 1};
  static const fu8 dev[2 * Sensors] = {0, 1, 2, 0, 1, 2};

  if (!p) return DMA_RX_NULL;

  ptrdiff_t offset = p - &rx[0][0][0];
  if (offset < 0 || offset >= sizeof rx) {
    return DMA_RX_NULL;
  }

  /* >> 3 is division by 8 (i.e., SENSOR_BUF_SIZE) */
  fu8 idx = (fu8)(offset) >> 3;

  *type = dev[idx];
  return buf[idx];
}


/* ------ Callbacks ------ */

/// Finish transfer and publish device data flag.
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  fu8 t;
  fu8 i = decode_ptr(hspi->pRxBuffPtr, &t);

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
  fu8 t;
  fu8 i = decode_ptr(hspi->pRxBuffPtr, &t);

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
  fu8 i = load(&in_transfer, Acq);

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
      st = dma_spi_txrx(tx[0], (uint8_t *)rx[i][0], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR) {
        BARO_CS_HIGH();
      }
      break;

    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
      GYRO_CS_LOW();
      st = dma_spi_txrx(tx[1], (uint8_t *)rx[i][1], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR) {
        GYRO_CS_HIGH();
      }
      break;

    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
      ACCEL_CS_LOW();
      st = dma_spi_txrx(tx[2], (uint8_t *)rx[i][2], SENSOR_BUF_SIZE);
      if (st == HAL_ERROR) {
        ACCEL_CS_HIGH();
      }
      break;

    default: break;
  }
}


/* ------ Public API ------ */

/// Fetches whatever is available in a free (tm) DMA buffer.
/// Returns 1 when all required sensor buckets have been filled,
/// (accumulates acrosss calls), 0 otherwise, and -1 on bag argument.
int dma_try_fetch(struct measurement *buf, fu8 skip_mask)
{
  static fu8 i = 1;
  static fu8 cache = 0;

  if (!buf) {
    return -1;
  }
  
  if (is_complete[i] & BARO_DONE) {
    buf->baro.p = U24(rx[i][0][1], rx[i][0][2], rx[i][0][3]);
    buf->baro.t = U24(rx[i][0][4], rx[i][0][5], rx[i][0][6]);
  }

  if (!(skip_mask & GYRO_DONE) && is_complete[i] & GYRO_DONE) {
    buf->gyro.x = F16(rx[i][1][1], rx[i][1][2]);
    buf->gyro.y = F16(rx[i][1][3], rx[i][1][4]);
    buf->gyro.z = F16(rx[i][1][5], rx[i][1][6]);
  }

  if (!(skip_mask & ACCL_DONE) && is_complete[i] & ACCL_DONE) {
    buf->d.accl.x = F16(rx[i][2][2], rx[i][2][3]);
    buf->d.accl.y = F16(rx[i][2][4], rx[i][2][5]);
    buf->d.accl.z = F16(rx[i][2][6], rx[i][2][7]);  
  }

  cache |= is_complete[i];
  cache |= skip_mask;

  is_complete[i] = 0;
  
  store(&in_transfer, i, Rel);
  i ^= 1;

  if (cache & RX_DONE) {
    cache = 0;
    return 1;
  }

  return 0;
}


/// Aggregates sensor compensation functions.
/// Run before reporting or publishing data.
void compensate(struct measurement *buf, fu8 skip_mask)
{
  buf->baro.t   = baro_comp_temp(buf->baro.t);
  buf->baro.p   = baro_comp_pres(buf->baro.p);
  buf->baro.alt = baro_calc_alt (buf->baro.p);
  
  if (!(skip_mask & ACCL_DONE)) {
    buf->d.accl.x *= MG;
    buf->d.accl.y *= MG;
    buf->d.accl.z *= MG;
  }
}
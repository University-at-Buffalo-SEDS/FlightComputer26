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
 * on one or two of the designated GPIO pins. Depedning
 * on the pin, a DMA transfer is initiated for a corre-
 * -sponding device into one of two receive buffers.
 * The device's CS pin is set low.
 *
 * One buffer consists of 3 sections (for each device).
 * Every time a completion callback receives, it will:
 *
 * 1. Infer the buffer and section (thus the device
 *    type) by calculating pointer offset from the
 *    beginning of the receive buffer.
 *
 * 2. If the pointer matches any valid section,
 *    invalidate data cache for that section and
 *    pull CS pin of the corresponding device high.
 *
 *    [!] If the offset is beyond the Receive buffers,
 *        it will pull CS pins of ALL device up, thus
 *        possibly terminating active transfers.
 *
 * 3. Set the 'ready' flag for the concerned device.
 *
 * The error callback will perform the same sequence
 * of events EXCEPT for invalidating data cache and
 * setting the 'ready' flag.
 *
 * The consumer is provided with two options:
 * 
 * 1. Parse specific device {Barometer, IMU}.
 *    In this case, data is copied for the
 *    whole device or not copied at all.
 *
 * 2. Fetch anything that is available per sensor
 *    (Barometer, Gyroscope, Accelerometer). In
 *    this case, a mask is returned that represents
 *    devices for which data was fetched.
 *
 * Compensation aggregators per sensor are provided
 * as inline helpers in the header (dma.h).
 */

#include "platform.h"
#include "dma.h"


/*
 * Each sensor data readiness flag for two buffers.
 */
static volatile fu8 is_complete[2][Sensors] = {0};

/*
 * Each device's buffer accepting DMA transfers.
 */
static atomic_uint_fast8_t transfer_imu = 0;
static atomic_uint_fast8_t transfer_baro = 0;

/*
 * Per device flag for signaling ongoing DMA transfer.
 */
static atomic_uint_fast32_t in_progress[Sensors] = {0};

/*
 * Double-buffered Rx for each sensor.
 */
static volatile uint8_t rx[2][Sensors][SENSOR_BUF_SIZE] = {0};


/* ------ Helpers ------ */


/*
 * Determines active buffer and device type from pointer offset.
 * Returns DMA_RX_NULL if p is NULL or does not point inside rx.
 * Context: DMA callbacks.
 */
static inline fu8
decode_ptr(uint8_t *p, fu8 *type)
{
  static const fu8 buf[2 * Sensors] = {0, 0, 0, 1, 1, 1};
  static const enum device dev[2 * Sensors] = {
    Sensor_Baro, Sensor_Gyro, Sensor_Accl,
    Sensor_Baro, Sensor_Gyro, Sensor_Accl
  };

  if (!p) {
    return DMA_RX_NULL;
  }

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


/*
 * Finish transfer and publish device data flag.
 */
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

  is_complete[i][t] = 1;
  store(&in_progress[t], 0, Rel);
}


/*
 * Finish transfer but do not publish flag (drop sample).
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  fu8 t;
  fu8 i = decode_ptr(hspi->pRxBuffPtr, &t);

  if (i == DMA_RX_NULL) {
    terminate_transfers();
    return;
  }

  finish_transfer(t);
  store(&in_progress[t], 0, Rel);
}


/* ------ ISR ------ */


/*
 * Attempts to initialize DMA transfer.
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  fu8 i;
  fu32 t = 0;
  HAL_StatusTypeDef st;

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
    {
      static const uint8_t tx_baro[BARO_DMA_BUF_SIZE] = {
        [0] = BARO_TX_BYTE, [1 ... 7] = 0x00
      };

      if (cas_strong(&in_progress[Sensor_Baro], &t, 1, Acq, Rlx))
      {
        i = load(&transfer_baro, Acq);
        baro_cs_low();
  
        st = dma_spi_txrx(tx_baro, (uint8_t *)rx[i][Sensor_Baro],
                                              BARO_DMA_BUF_SIZE);
        if (st != HAL_OK) {
          baro_cs_high();
          store(&in_progress[Sensor_Baro], 0, Rlx);
        }
      }
      break;
    }

    case GYRO_INT_PIN_1:
/*  case GYRO_INT_PIN_2:        unused for IREC '26 */
    {
      static const uint8_t tx_gyro[GYRO_DMA_BUF_SIZE] = {
        [0] = GYRO_TX_BYTE, [1 ... 7] = 0x00
      };

      if (cas_weak(&in_progress[Sensor_Gyro], &t, 1, Acq, Rlx))
      {
        i = load(&transfer_imu, Acq);
        gyro_cs_low();
  
        st = dma_spi_txrx(tx_gyro, (uint8_t *)rx[i][Sensor_Gyro],
                                              GYRO_DMA_BUF_SIZE);
        if (st == HAL_ERROR) {
          gyro_cs_high();
          store(&in_progress[Sensor_Gyro], 0, Rlx);
        }
        break;
      }
    }

    case ACCL_INT_PIN_1:
/*  case ACCL_INT_PIN_2:        unused for IREC '26 */
    {
      static const uint8_t tx_accl[ACCL_DMA_BUF_SIZE] = {
        [0] = ACCL_TX_BYTE, [1 ... 7] = 0x00
      };

      if (cas_weak(&in_progress[Sensor_Accl], &t, 1, Acq, Rlx))
      {
        i = load(&transfer_imu, Acq);
        accl_cs_low();
        
        st = dma_spi_txrx(tx_accl, (uint8_t *)rx[i][Sensor_Accl],
                                              ACCL_DMA_BUF_SIZE);
        if (st == HAL_ERROR) {
          accl_cs_high();
          store(&in_progress[Sensor_Accl], 0, Rlx);
        }
        break;
      }
    }

    default: break;
  }
}


/* ------ Public API ------ */


static atomic_uint_fast8_t baro_i = 1;

/*
 * Peeks at the barometer DMA buffer status,
 * and fetches the latest pair of measurements
 * (temperature, pressure), if available.
 * Returns 1 on successful fetch and 0 otherwise. 
 */
fu8 dma_fetch_baro(struct baro *buf)
{
  if (!buf) {
    return UINT_FAST8_MAX;
  }

  if (is_complete[baro_i][Sensor_Baro])
  {
    buf->p = U24(rx[baro_i][0][1], rx[baro_i][0][2], rx[baro_i][0][3]);
    buf->t = U24(rx[baro_i][0][4], rx[baro_i][0][5], rx[baro_i][0][6]);

    is_complete[baro_i][Sensor_Baro] = 0;

    baro_i = swap(&transfer_baro, baro_i, Rel);

    return 1;
  }

  baro_i = swap(&transfer_baro, baro_i, Rel);

  return 0;
}


static atomic_uint_fast8_t imu_i = 1;

/*
 * Peeks at the IMU DMA buffers' statuses, and
 * fetches the latest pair of measurements
 * (accelerometer, gyroscope), if available.
 * Returns 1 on successful fetch and 0 otherwise.
 */
fu8 dma_fetch_imu(struct coords *gyro, struct coords *accl)
{
  if (!gyro || !accl) {
    return UINT_FAST8_MAX;
  }

  if (is_complete[imu_i][Sensor_Gyro] &&
      is_complete[imu_i][Sensor_Accl])
  {
    gyro->x = F16(rx[imu_i][1][1], rx[imu_i][1][2]);
    gyro->y = F16(rx[imu_i][1][3], rx[imu_i][1][4]);
    gyro->z = F16(rx[imu_i][1][5], rx[imu_i][1][6]);

    accl->x = F16(rx[imu_i][2][2], rx[imu_i][2][3]);
    accl->y = F16(rx[imu_i][2][4], rx[imu_i][2][5]);
    accl->z = F16(rx[imu_i][2][6], rx[imu_i][2][7]);

    is_complete[imu_i][Sensor_Gyro] = 0;
    is_complete[imu_i][Sensor_Accl] = 0;

    imu_i = swap(&transfer_imu, imu_i, Rel);

    return 1;
  }

  imu_i = swap(&transfer_imu, imu_i, Rel);

  return 0;
}


/*
 * Fetches whatever is available in a free (tm) DMA
 * buffer. Returns a relevancy mask, which has bits
 * set for devices whose data was fetched.
 */
fu8 dma_fetch(struct measurement *buf, fu8 skip_mask)
{ 
  if (!buf) {
    return UINT_FAST8_MAX;
  }
  
  fu8 fetched = 0;

  if (!(skip_mask & RX_BARO) && is_complete[baro_i][Sensor_Baro])
  {
    buf->baro.p = U24(rx[baro_i][0][1], rx[baro_i][0][2], rx[baro_i][0][3]);
    buf->baro.t = U24(rx[baro_i][0][4], rx[baro_i][0][5], rx[baro_i][0][6]);

    is_complete[baro_i][Sensor_Baro] = 0;
    fetched |= RX_BARO;
  }

  if (!(skip_mask & RX_GYRO) && is_complete[imu_i][Sensor_Gyro])
  {
    buf->gyro.x = F16(rx[imu_i][1][1], rx[imu_i][1][2]);
    buf->gyro.y = F16(rx[imu_i][1][3], rx[imu_i][1][4]);
    buf->gyro.z = F16(rx[imu_i][1][5], rx[imu_i][1][6]);

    is_complete[imu_i][Sensor_Gyro] = 0;
    fetched |= RX_GYRO;
  }

  if (!(skip_mask & RX_ACCL) && is_complete[imu_i][Sensor_Accl])
  {
    buf->d.accl.x = F16(rx[imu_i][2][2], rx[imu_i][2][3]);
    buf->d.accl.y = F16(rx[imu_i][2][4], rx[imu_i][2][5]);
    buf->d.accl.z = F16(rx[imu_i][2][6], rx[imu_i][2][7]);

    is_complete[imu_i][Sensor_Accl] = 0;
    fetched |= RX_ACCL;
  }

  imu_i = swap(&transfer_imu, imu_i, Rel);

  baro_i = swap(&transfer_imu, baro_i, Rel);

  return fetched;
}
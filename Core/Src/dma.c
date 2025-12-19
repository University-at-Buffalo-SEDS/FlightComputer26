/*
 * Interrupt-based sensor data collection.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"
#include "dma.h"

/*
 * Control struct and tx/rx buffers.
 * Two shared rx bufs implement double buffering.
 */
static dma_t ctr = {0};

static uint8_t tx[3][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [1][0] = GYRO_TX_BYTE, [2][0] = ACCEL_TX_BYTE
};
static uint8_t rx[2][SENSOR_BUF_SIZE];

static inline uint_fast8_t index_from_ptr(uint8_t *p)
{
  return (p != NULL) ? ((p == rx[0]) ? 0 : 1) : 127;
}

static inline void switch_pull_cs_high(uint_fast8_t i)
{
  switch (ctr.type[i]) {
    case BAROMETER:
      PL_CS_BARO_HIGH(); break;
    case GYROSCOPE:
      PL_CS_GYRO_HIGH(); break;
    case ACCELEROMETER:
      PL_CS_ACCEL_HIGH(); break;
    case NONE: return;
  }
}

/*
 * TODO
 */
void HAL_SPI_TxRxCpltCallback(PL_SPI_Handle *hspi)
{
  uint_fast8_t a = index_from_ptr(hspi->pRxBuffPtr);
  if (a == 127) return;

  INVALIDATE_DCACHE_ADDR_INT(rx[a], SENSOR_BUF_SIZE);
  switch_pull_cs_high(a);
}

/*
 * TODO
 */
void HAL_SPI_ErrorCallback(PL_SPI_Handle *hspi)
{
  uint_fast8_t a = index_from_ptr(hspi->pRxBuffPtr);
  if (a == 127) return;

  CLEAN_DCACHE_ADDR_INT(rx[a], SENSOR_BUF_SIZE);
  switch_pull_cs_high(a);
}

/*
 * TODO
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint_fast8_t a = atomic_fetch_xor_explicit(&ctr.w, 1u, memory_order_acq_rel) & 1u;

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
      ctr.type[a] = BAROMETER;
      PL_CS_BARO_LOW();
      DMA_SPI_TXRX(tx[0], rx[a], SENSOR_BUF_SIZE);
      break;
    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
      ctr.type[a] = GYROSCOPE;
      PL_CS_GYRO_LOW();
      DMA_SPI_TXRX(tx[1], rx[a], SENSOR_BUF_SIZE);
      break;
    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
      ctr.type[a] = ACCELEROMETER;
      PL_CS_ACCEL_LOW();
      DMA_SPI_TXRX(tx[2], rx[a], SENSOR_BUF_SIZE);
      break;
    default: return;
  }
}

inline dma_e dma_read_latest(payload_t *buf)
{
  uint_fast8_t a = atomic_load_explicit(&ctr.w, memory_order_acquire);
  buf->type = ctr.type[a];
  
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
    case NONE: return DMA_EMPTY;
  }

  return DMA_OK;
}
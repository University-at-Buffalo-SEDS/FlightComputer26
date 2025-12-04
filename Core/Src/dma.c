/*
 * Interrupt-based sensor data collection.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "platform.h"
#include "dma.h"

/* 
 * DMA buffers for each sensor.
 * Note: HAL expects non-volatile buffers.
 * Tx: Producer: (final value). Consumer: DMA peripheral.
 * Rx: Producer: DMA peripheral. Consumer: ReceiveComplete callback.
 */
static uint8_t baro_dma_tx[BMP390_BUF_SIZE] = {[0] = BARO_TX_BYTE};
static uint8_t baro_dma_rx[BMP390_BUF_SIZE];
static uint8_t gyro_dma_tx[GYRO_BUF_SIZE] = {[0] = GYRO_TX_BYTE};
static uint8_t gyro_dma_rx[GYRO_BUF_SIZE];
static uint8_t accel_dma_tx[ACCEL_BUF_SIZE] = {[0] = ACCEL_TX_BYTE};
static uint8_t accel_dma_rx[ACCEL_BUF_SIZE];

/* 
 * Ring buffer. Producer: ReceiveComplete callback.
 * Consumers: Sensor and Kalman tasks.
 */
static volatile payload_t ring[RING_SIZE];
static volatile atomic_uint_fast16_t head = ATOMIC_VAR_INIT(0);
static volatile atomic_uint_fast16_t tail = ATOMIC_VAR_INIT(0);

/*
 * Device to expect next time ReceiveComplete is invoked by HAL.
 */
static volatile atomic_uint_fast8_t next = ATOMIC_VAR_INIT(NONE);
static uint_fast8_t expected = ATOMIC_VAR_INIT(NONE);

/*
 * Enqueues data and signals transfer completion.
 */
static inline void enqueue(const expected_e type)
{
  uint_fast16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint_fast16_t t = atomic_load_explicit(&tail, memory_order_acquire);
  uint_fast16_t i = h & RING_MASK;

  /* 
   * If the ring is full, overwrite oldest entry
   * to keep slow consumer updated.
   */
  if (((h + 1) & RING_MASK) == (t & RING_MASK))
    atomic_store_explicit(&tail, t + 1, memory_order_release);

  switch (type) {
    case BAROMETER:
    {
      INVALIDATE_DCACHE_ADDR_INT(baro_dma_rx, PL_BUF_SIZE_BARO);

      ring[i].type = BAROMETER;
      ring[i].data.baro[0] = U24(baro_dma_rx[1], baro_dma_rx[2], baro_dma_rx[3]);
      ring[i].data.baro[1] = U24(baro_dma_rx[4], baro_dma_rx[5], baro_dma_rx[6]);

      PL_CS_BARO_HIGH();
      break;
    }
    case GYROSCOPE:
    {
      INVALIDATE_DCACHE_ADDR_INT(gyro_dma_rx, PL_BUF_SIZE_GYRO); 

      ring[i].type = GYROSCOPE;
      ring[i].data.gyro[0] = I16(gyro_dma_rx[1], gyro_dma_rx[2]);
      ring[i].data.gyro[1] = I16(gyro_dma_rx[3], gyro_dma_rx[4]);
      ring[i].data.gyro[2] = I16(gyro_dma_rx[5], gyro_dma_rx[6]);

      PL_CS_GYRO_HIGH();
      break;
    }
    case ACCELEROMETER:
    {
      INVALIDATE_DCACHE_ADDR_INT(accel_dma_rx, PL_BUF_SIZE_ACCEL);

      ring[i].type = ACCELEROMETER;
      ring[i].data.accel[0] = F16(accel_dma_rx[2], accel_dma_rx[3]);
      ring[i].data.accel[1] = F16(accel_dma_rx[4], accel_dma_rx[5]);
      ring[i].data.accel[2] = F16(accel_dma_rx[6], accel_dma_rx[7]);

      PL_CS_ACCEL_HIGH();
      break;
    }
    case NONE: return;
  }

  atomic_store_explicit(&head, h + 1, memory_order_release);
}

/*
 * Copy the oldest raw element of the ring.
 */
inline bool dma_ring_copy_oldest(payload_t *buf)
{
  uint16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint16_t t = atomic_load_explicit(&tail, memory_order_relaxed);
  uint16_t i = t & RING_MASK;

  if ((h & RING_MASK) == i)
    return false;

  DISABLE_HAL_INTS();
  *buf = ring[i];
  ENABLE_HAL_INTS();

  return true;
}

/*
 * Dequeue the oldest raw element of the ring and advance tail.
 */
inline bool dma_ring_dequeue_oldest(payload_t *buf)
{
  uint16_t h = atomic_load_explicit(&head, memory_order_acquire);
  uint16_t t = atomic_load_explicit(&tail, memory_order_acquire);
  uint16_t i = t & RING_MASK;

  /*
   * Check if the ring is empty (head is on tail)
   */
  if ((h & RING_MASK) == i)
    return false;

  DISABLE_HAL_INTS();
  *buf = ring[i];
  ENABLE_HAL_INTS();

  atomic_store_explicit(&tail, t + 1, memory_order_release);
  return true;
}

/*
 * Enqueues data and signals transfer completion.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Lock == HAL_LOCKED)
    return;

  enqueue(atomic_load_explicit(&next, memory_order_acquire));
  atomic_store_explicit(&next, NONE, memory_order_release);
}

/*
 * Cleans cache for a device that produced an error.
 */
void HAL_SPI_ErrorCallback(PL_SPI_Handle *hspi)
{
  if (hspi->Lock == HAL_LOCKED)
    return;

  expected_e t = atomic_load_explicit(&next, memory_order_acquire);
  atomic_store_explicit(&next, NONE, memory_order_release);

  switch (t) {
    case BAROMETER:
    {
      CLEAN_DCACHE_ADDR_INT(baro_dma_rx, PL_BUF_SIZE_BARO);
      PL_CS_BARO_HIGH();
      break;
    }
    case GYROSCOPE:
    {
      CLEAN_DCACHE_ADDR_INT(gyro_dma_rx, PL_BUF_SIZE_GYRO);
      PL_CS_GYRO_HIGH();
      break;
    }
    case ACCELEROMETER:
    {
      CLEAN_DCACHE_ADDR_INT(accel_dma_rx, PL_BUF_SIZE_ACCEL);
      PL_CS_ACCEL_HIGH();
      break;
    }
    case NONE: return;
  }
}

/*
 * Interrupt used to trigger DMA transfer
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  PL_HAL_Handle st;

  switch (GPIO_Pin) {
    case BARO_INT_PIN:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, BAROMETER))
        return;

      PL_CS_BARO_LOW();
      st = DMA_SPI_TXRX(baro_dma_tx, baro_dma_rx, PL_BUF_SIZE_BARO);

      if (st != HAL_OK) {
        PL_CS_BARO_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    case GYRO_INT_PIN_1:
    case GYRO_INT_PIN_2:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, GYROSCOPE))
        return;

      PL_CS_GYRO_LOW();
      st = DMA_SPI_TXRX(gyro_dma_tx, gyro_dma_rx, PL_BUF_SIZE_GYRO);

      if (st != HAL_OK) {
        PL_CS_GYRO_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    case ACCEL_INT_PIN_1:
    case ACCEL_INT_PIN_2:
    {
      if (!atomic_compare_exchange_strong(&next, &expected, ACCELEROMETER))
        return;

      PL_CS_ACCEL_LOW();
      st = DMA_SPI_TXRX(accel_dma_tx, accel_dma_rx, PL_BUF_SIZE_ACCEL);

      if (st != HAL_OK) {
        PL_CS_ACCEL_HIGH();
        atomic_store_explicit(&next, NONE, memory_order_release);
      }
      break;
    }
    default: return;
  }
}
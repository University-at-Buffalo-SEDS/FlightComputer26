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


/* ------ Static storage ------ */

/* Each sensor data readiness flag for two buffers. */
static volatile fu8 is_complete[2][Sensors] = {0};

/* Each device's buffer accepting DMA transfers.
 * 1 - Barometer, 0 - IMU. */
static atomic_uint_fast8_t transfer[2] = {0};

/* Consumer-side flags for 'transfer' */
static atomic_uint_fast8_t imu_i = 1;
static atomic_uint_fast8_t baro_i = 1;

/* Per device flag for signaling ongoing DMA transfer. */
static atomic_uint_fast8_t in_progress[Sensors] = {0};

/* Double-buffered Rx for each sensor. */
static volatile uint8_t rx[2][Sensors][SENSOR_BUF_SIZE] = {0};

#ifdef DMA_BENCHMARK

/* HAL timestamp for consumer benchmark.
 * Stores relative time when RxCplt finishes transfer. */
static atomic_uint_fast32_t rxts[Sensors] = {0};

/* Stores time, in milliseconds, that ISR took to execute.
 * Updated inside ISR, reported inside thread context. */
static atomic_uint_fast32_t isr_dt = 0;

#endif // DMA_BENCHMARK

/* GPIO port and pin lookup based on sensor.
 * Stored consecutively for single cache load. */
static const struct gpio_lookup gpio = {
  .port = {CS_BARO_GPIO_Port, CS_GYRO_GPIO_Port, CS_ACCEL_GPIO_Port},
  .pin = {CS_BARO_Pin, CS_GYRO_Pin, CS_ACCEL_Pin}
};

/* Per-sensor buffer with transmit byte set */
static const uint8_t tx[Sensors][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [0][1 ... 7] = 0x00,
  [1][0] = GYRO_TX_BYTE, [1][1 ... 7] = 0x00,
  [2][0] = ACCL_TX_BYTE, [2][1 ... 7] = 0x00,
};

/* ------ Static storage ------ */


/* ------ Callbacks ------ */

/*
 * Finish transfer and publish device data flag.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (!hspi || !hspi->pRxBuffPtr) {
    terminate_transfers();
    return;
  }

  ptrdiff_t offset = hspi->pRxBuffPtr - &rx[0][0][0];

  if (offset < 0 || offset >= sizeof rx) {
    terminate_transfers();
    return;
  }

  /* >> 3 is division by 8 (i.e., SENSOR_BUF_SIZE) */
  fu8 idx = (fu8)(offset) >> 3;
  fu8 i = idx > 2;
  fu8 t = idx - (i ? 3 : 0);

  invalidate_dcache_addr_int(rx[i][t], SENSOR_BUF_SIZE);
  gpio_cs_high(t);

#ifdef DMA_BENCHMARK
  store(&rxts[t], now_ms(), Rel);
#endif

  is_complete[i][t] = 1;
  store(&in_progress[t], 0, Rel);
}

/*
 * Finish transfer but do not publish flag (drop sample).
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (!hspi || !hspi->pRxBuffPtr) {
    terminate_transfers();
    return;
  }

  ptrdiff_t offset = hspi->pRxBuffPtr - &rx[0][0][0];

  if (offset < 0 || offset >= sizeof rx) {
    terminate_transfers();
    return;
  }

  fu8 idx = (fu8)(offset) >> 3;
  fu8 t = idx - (idx > 2 ? 3 : 0);

  gpio_cs_high(t);
  store(&in_progress[t], 0, Rel);
}

/* ------ Callbacks ------ */


/* ------ DMA helper ------ */

/*
 * Attempts to begin a DMA transfer for the specified device.
 * Has no side effects if there is a transfer ongoing, and
 * pulls sensor's CS pin back high on DMA-side failures.
 */
static inline enum dma_status
start_dma_transfer(enum sensor k)
{
  HAL_StatusTypeDef st;
  fu8 desired = 0;

  if (cas_strong(&in_progress[k], &desired, 1, Acq, Rlx))
  {
    /* The point of !k is to map Gyro and Accl to the same index
     * and Baro to a different one. This works because Baro is 0. */
    fu32 i = load(&transfer[!k], Acq);
    gpio_cs_low(k);

    st = dma_spi_txrx(tx[k], (uint8_t *)rx[i][k], SENSOR_BUF_SIZE);

    if (st == HAL_OK)
    {
      return DMA_Ok;
    }
    else
    {
      gpio_cs_high(k);
      store(&in_progress[k], 0, Rlx);
      return DMA_Error;
    }
  }

  return DMA_Busy;
}

/* ------ DMA helper ------ */


/* ------ ISR ------ */

/*
 * Attempts to initialize DMA transfer.
 *
 * This ISR may be expensive depending on how often
 * the sensors will generate DRDY events. I includied
 * a microbenchmark to estimate the time this ISR and
 * DMA take to execute. The performance effect of this
 * ISR on other tasks will also be evident from tests.
 *
 * If this ISR proves to crowd out tasks, we generally
 * have 3 other options of using the benefits of DMA:
 *
 * 1. Tune IMU for higher precision. This may sound
 *    lazy and redundant, but it will relax the ISR
 *    and make Predict to Update ratio closer to 1.
 *
 * 2. Starting DMA transfer synchronously on demand.
 *    This will require:
 *    a) checking if there is new data (relatively
 *       cheap if we keep ISR to receive and register
 *       DRDY events in atomic flags),
 *    b) starting DMA transfer in advance to allow
 *       DMA arbiter time to write into memory (how
 *       we determine 'advance' for each sensor and
 *       KF scenario is another good question!),
 *    c) checking if the transfer completed by the
 *       time we need the new data.
 *
 * 3. Make consumer notify the ISR via atomic flag
 *    that it consumed whatever was in the buffers,
 *    and perform transfer on CAS. This will yield
 *    older data (which is ~ok?? for speedy consumers).
 *
 * 4. Not that I cannot count, but I don't think this
 *    is a viable option: begin transfer on TX timer.
 *    Because scheduling is not deterministic (tasks
 *    yield cooperatively and not all of them are RR,
 *    e.g. Recovery), this timer may end up playing
 *    blind baseball with Distribution task that may
 *    periodically wait on active transfers.
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
#ifdef DMA_BENCHMARK
  fu32 called = now_ms();
#endif

  start_dma_transfer(sensor_idx(GPIO_Pin));

#ifdef DMA_BENCHMARK
  store(&isr_dt, now_ms() - called, Rel);
#endif
}

/* ------ ISR ------ */


/* ------ Public API ------ */

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

  dma_bench_log_isr();

  if (is_complete[baro_i][Sensor_Baro])
  {
    buf->p = U24(rx[baro_i][0][1], rx[baro_i][0][2], rx[baro_i][0][3]);
    buf->t = U24(rx[baro_i][0][4], rx[baro_i][0][5], rx[baro_i][0][6]);

    is_complete[baro_i][Sensor_Baro] = 0;

    baro_i = swap(&transfer[1], baro_i, Rel);

    dma_bench_log_fetch(Sensor_Baro);

    return 1;
  }

  baro_i = swap(&transfer[1], baro_i, Rel);

  return 0;
}

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

  dma_bench_log_isr();

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

    imu_i = swap(&transfer[0], imu_i, Rel);

    dma_bench_log_fetch(Sensor_Gyro);
    dma_bench_log_fetch(Sensor_Accl);

    return 1;
  }

  imu_i = swap(&transfer[0], imu_i, Rel);

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

    dma_bench_log_fetch(Sensor_Baro);
  }

  if (!(skip_mask & RX_GYRO) && is_complete[imu_i][Sensor_Gyro])
  {
    buf->gyro.x = F16(rx[imu_i][1][1], rx[imu_i][1][2]);
    buf->gyro.y = F16(rx[imu_i][1][3], rx[imu_i][1][4]);
    buf->gyro.z = F16(rx[imu_i][1][5], rx[imu_i][1][6]);

    is_complete[imu_i][Sensor_Gyro] = 0;
    fetched |= RX_GYRO;

    dma_bench_log_fetch(Sensor_Gyro);
  }

  if (!(skip_mask & RX_ACCL) && is_complete[imu_i][Sensor_Accl])
  {
    buf->d.accl.x = F16(rx[imu_i][2][2], rx[imu_i][2][3]);
    buf->d.accl.y = F16(rx[imu_i][2][4], rx[imu_i][2][5]);
    buf->d.accl.z = F16(rx[imu_i][2][6], rx[imu_i][2][7]);

    is_complete[imu_i][Sensor_Accl] = 0;
    fetched |= RX_ACCL;

    dma_bench_log_fetch(Sensor_Accl);
  }

  imu_i = swap(&transfer[0], imu_i, Rel);

  baro_i = swap(&transfer[1], baro_i, Rel);

  dma_bench_log_isr();

  return fetched;
}

/* ------ Public API ------ */
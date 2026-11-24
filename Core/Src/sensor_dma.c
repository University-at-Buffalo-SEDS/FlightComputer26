/* Core/Src/sensor_dma.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "FC-Threads.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "ground_test.h"
#include <math.h>

#define id "DM "


/* Sensor acquisition, run by the distribution thread itself (it used to be
 * a separate DMA thread). A data-ready interrupt marks the sensor in
 * flags.drdy and wakes the thread through sensor_events; the thread reads
 * every ready sensor with one SPI1 DMA transfer each, waiting on
 * dma_completion, and keeps the bytes in taskrx[] until try_fetch_*()
 * consumes them. distribution_wake() ends a wait early for inputs that have
 * no data-ready line (GPS, commands, the evaluator's stage). The one-tick
 * timeout only bounds the barometer DRDY level recovery below and
 * configuration changes; it never delays a signalled sample. */
static TX_SEMAPHORE dma_completion;
static TX_EVENT_FLAGS_GROUP sensor_events;
#define SENSOR_IDLE_POLL_TICKS 1U
#define DIST_WAKE_OTHER 0x10000UL

#ifdef SEDS_FIRMWARE_SIM_TEST
volatile uint32_t g_sim_sensor_ready_irqs;
volatile uint32_t g_sim_sensor_dma_started;
volatile uint32_t g_sim_sensor_dma_completed;
volatile uint32_t g_sim_sensor_dma_delivered;
#define SENSOR_PROBE(counter) (++(counter))
#else
#define SENSOR_PROBE(counter) ((void)0)
#endif

static const gpio_map gpio = {
  .port   = {BARO_CS_PORT, GYRO_CS_PORT, ACCL_CS_PORT},
  .pin    = {BARO_CS_PIN, GYRO_CS_PIN, ACCL_CS_PIN},
  .drdy   = {DMA_BARO_MASK, DMA_GYRO_MASK, DMA_ACCL_MASK},
  .offset = {DMA_BARO_OFFSET, DMA_GYRO_OFFSET, DMA_BARO_OFFSET}
};

static const uint8_t tx[Sensors][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [0][1 ... 7] = 0x00,
  [1][0] = GYRO_TX_BYTE, [1][1 ... 7] = 0x00,
  [2][0] = ACCL_TX_BYTE, [2][1 ... 7] = 0x00,
};


volatile uncached uint8_t dmarx[SENSOR_BUF_SIZE];

/* Read but not yet consumed, per sensor (only the distribution thread). */
static uint8_t taskrx[Sensors][SENSOR_BUF_SIZE - 2] = {0};

static dmasel select = {Sensors, 0};

/* drdy: set by the data-ready interrupts. relv: read, not yet consumed. */
static fdma flags = {0, 0};

static bool baro_level_seen;


/* Consumer */

bool try_fetch_baro(baro *buf)
{
  fu16 rxdone = fetch_and(&flags.relv, ~DMA_BARO_MASK, Acq);

  if (!(rxdone & DMA_BARO_MASK))
  {
    return false;
  }

  fu32 pres, temp; // never gets here

  sweetbench_catch(0);

  pres = U24(taskrx[Sensor_Baro][0],
             taskrx[Sensor_Baro][1],
             taskrx[Sensor_Baro][2]);

  temp = U24(taskrx[Sensor_Baro][3],
             taskrx[Sensor_Baro][4],
             taskrx[Sensor_Baro][5]);


  buf->tmp = baro_compensate_temp(temp);
  buf->prs = baro_compensate_pres(pres);
  /* The ground reference tracks drift on the pad and freezes at Launch. */
  buf->alt = baro_track_alt(buf->prs, current() < Launch);

#ifdef SD_AVAILABLE
  const float raw_sample[] = {(float)pres, (float)temp};
  /* Converted order is pressure, temperature, relative altitude; the raw
   * pressure/temperature ADC codes are sufficient to recompute all three. */
  const float converted_sample[] = {buf->prs, buf->tmp, buf->alt};
  /* The third raw entry is absent in hardware, not a synthesized ADC code. */
  const float raw_codes[] = {raw_sample[0], raw_sample[1], NAN};
  sd_append_sensor(Sensor_Baro, raw_codes, converted_sample, 3U);
#endif

  // led_toggle(LED1_PORT, LED1_PIN);

  /* Ground test: the real conversion above is on the SD card; the replay
   * replaces what the estimator and flight logic see. */
#if FC_GROUND_TEST
  if (igt_sim_running()) igt_override_baro(&buf->alt, now_ms());
#endif

  sweetbench_start(0, 150);
  
  return true;
}

bool try_fetch_gyro(f_xyz *buf)
{
  fu16 rxdone = fetch_and(&flags.relv, ~DMA_GYRO_MASK, Acq);

  if (!(rxdone & DMA_GYRO_MASK))
  {
    return false;
  }

  fi16 gx, gy, gz;

  sweetbench_catch(1);

  gx = I16(taskrx[Sensor_Gyro][0], taskrx[Sensor_Gyro][1]);
  gy = I16(taskrx[Sensor_Gyro][2], taskrx[Sensor_Gyro][3]);
  gz = I16(taskrx[Sensor_Gyro][4], taskrx[Sensor_Gyro][5]);
  

  buf->x = gx * inv_sens[init_rng];
  buf->y = gy * inv_sens[init_rng];
  buf->z = gz * inv_sens[init_rng];
  baro_ref_gyro(buf->x, buf->y, buf->z, now_ms());

#ifdef SD_AVAILABLE
  const float raw_sample[] = {(float)gx, (float)gy, (float)gz};
  sd_append_sensor(Sensor_Gyro, raw_sample, &buf->x, 3U);
#endif

#if FC_GROUND_TEST
  if (igt_sim_running()) igt_override_gyro(&buf->x, now_ms());
#endif

  sweetbench_start(1, 150);

  return true;
}

bool try_fetch_accl(f_xyz *buf)
{
  fu16 rxdone = fetch_and(&flags.relv, ~DMA_ACCL_MASK, Acq);

  if (!(rxdone & DMA_ACCL_MASK))
  {
    return false;
  }

  fi16 ax, ay, az;

  sweetbench_catch(2);

  ax = I16(taskrx[Sensor_Accl][0], taskrx[Sensor_Accl][1]);
  ay = I16(taskrx[Sensor_Accl][2], taskrx[Sensor_Accl][3]);
  az = I16(taskrx[Sensor_Accl][4], taskrx[Sensor_Accl][5]);
  

  buf->x = ax * lsb_to_g;
  buf->y = ay * lsb_to_g;
  buf->z = az * lsb_to_g;
  baro_ref_accel(buf->x, buf->y, buf->z, now_ms());

#ifdef SD_AVAILABLE
  const float raw_sample[] = {(float)ax, (float)ay, (float)az};
  sd_append_sensor(Sensor_Accl, raw_sample, &buf->x, 3U);
#endif

#if FC_GROUND_TEST
  if (igt_sim_running()) igt_override_accl(&buf->x, now_ms());
#endif

  sweetbench_start(2, 150);

  return true;
}


/* Callbacks */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SENSOR_PROBE(g_sim_sensor_dma_completed);
	gpio_cs_high(select.next);
  select.valid = 1;
  tx_semaphore_put(&dma_completion);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	gpio_cs_high(select.next);
  select.valid = 0;
  tx_semaphore_put(&dma_completion);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  SENSOR_PROBE(g_sim_sensor_ready_irqs);
	(void) fetch_or(&flags.drdy, GPIO_Pin, Rel);
  /* After the flag: a thread that saw no flag still finds this event. */
  (void) tx_event_flags_set(&sensor_events, GPIO_Pin, TX_OR);
}


/* DMA routine */

static inline void propagate_rx(void)
{
  SENSOR_PROBE(g_sim_sensor_dma_delivered);
  const uint8_t *src = (const uint8_t *)(dmarx + gpio.offset[select.next]);
  uint8_t *dst = taskrx[select.next];

  size_t n = sizeof taskrx / Sensors;
  for (size_t i = 0; i < n; ++i) dst[i] = src[i];

  fetch_or(&flags.relv, gpio.drdy[select.next], Rel);
}

static inline void start_dma_transfer(void)
{
  SENSOR_PROBE(g_sim_sensor_dma_started);
  HAL_StatusTypeDef st;

  /* A completion may arrive before the thread blocks. A semaphore retains
   * that event; aborting a not-yet-sleeping thread loses the notification. */
  while (tx_semaphore_get(&dma_completion, TX_NO_WAIT) == TX_SUCCESS) {}
  select.valid = 0;

  /* Consume the selected ready event before starting SPI. A new sensor IRQ
   * can arrive inside HAL_SPI_TransmitReceive_DMA; clearing after that call
   * would erase it and can leave acquisition waiting forever. */
  fetch_and(&flags.drdy, ~gpio.drdy[select.next], Rlx);
  gpio_cs_low(select.next);

  st = dma_spi_txrx(tx[select.next], (uint8_t *)dmarx,
                                     SENSOR_BUF_SIZE);

  if (st != HAL_OK)
  {
    gpio_cs_high(select.next);
    fetch_or(&flags.drdy, gpio.drdy[select.next], Rel);
    tx_thread_sleep(1);
    return;
  }

  UINT completion = tx_semaphore_get(&dma_completion, DMA_TIMEOUT_MS);
  if (completion == TX_SUCCESS && select.valid)
  {
    propagate_rx();
  }
  else if (completion != TX_SUCCESS)
  {
    /* Stop the old transaction before another sensor can own the buffer. */
    HAL_SPI_Abort(&hspi1);
    gpio_cs_high(select.next);
    fetch_or(&flags.drdy, gpio.drdy[select.next], Rel);
  }
}

/* Read every sensor whose data is ready: gyro, accelerometer, barometer.
 * Before Apogee all three are sampled; the descent filter uses only the
 * barometer (the IMU is disabled then). Each sensor is read at most once per
 * call, so a failed transfer (flag restored) waits for the next call. */
static void acquire_ready_samples(void)
{
  /* Recover a high DRDY missed while EXTI was being enabled. Observe each
   * high period once: BMP390 non-latched DRDY stays high for 2.5ms even
   * after a data read, so repeatedly polling high would duplicate samples. */
  bool baro_high = HAL_GPIO_ReadPin(BARO_EXTI_GPIO_Port, BARO_EXTI_Pin) == GPIO_PIN_SET;
  if (baro_high && !baro_level_seen)
  {
    fetch_or(&flags.drdy, DMA_BARO_MASK, Rel);
  }
  baro_level_seen = baro_high;

  static const fu8 order[Sensors] = {Sensor_Gyro, Sensor_Accl, Sensor_Baro};
  const fu16 sampled = (load(&g_conf, Acq) & option(Using_Ascent_KF))
                     ? DMA_SENSOR_MASKS : DMA_BARO_MASK;
  const fu16 ready = load(&flags.drdy, Acq) & sampled;

  for (fu8 k = 0; k < Sensors; ++k)
  {
    if (ready & gpio.drdy[order[k]])
    {
      select.next = order[k];
      start_dma_transfer();
    }
  }
}

/* Idle wait */

void dma_wait_for_samples(fu16 wanted)
{
  ULONG seen;
  acquire_ready_samples();
  /* A sample already read is still pending in relv: never sleep on it. A
   * data-ready interrupt after the read above sets its event first, so the
   * wait below returns at once. */
  if (load(&flags.relv, Acq) & wanted)
  {
    return;
  }
  (void) tx_event_flags_get(&sensor_events, (ULONG)DMA_SENSOR_MASKS | DIST_WAKE_OTHER,
                            TX_OR_CLEAR, &seen, SENSOR_IDLE_POLL_TICKS);
  acquire_ready_samples();
}

void distribution_wake(void)
{
  (void) tx_event_flags_set(&sensor_events, DIST_WAKE_OTHER, TX_OR);
}

/* Created before the distribution thread starts (it owns acquisition). */
UINT dma_acquisition_init(void)
{
  if (tx_semaphore_create(&dma_completion, "Sensor DMA completion", 0) != TX_SUCCESS)
    return FC_INIT_FAILED;
  if (tx_event_flags_create(&sensor_events, id "R") != TX_SUCCESS)
  {
    tx_semaphore_delete(&dma_completion);
    return FC_INIT_FAILED;
  }
  return TX_SUCCESS;
}

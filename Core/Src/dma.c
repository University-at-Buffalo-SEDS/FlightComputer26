/*
 * Direct Memory Access implementation
 *
 * Sensors:  Barometer, Gyroscope, Accelerometer
 * Consumer: Distribution Task
 *
 * This module implements the HAL Interrupt Service
 * Routine, DMA transfer callbacks for completion and
 * error handling, logical sensor selector function,
 * DMA transfer routine, and public fetching functions.
 *
 * Compensation is performed in fetching functions and
 * is not needed to be done elsewhere.
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "DM "


TX_THREAD dma_task;

static const gpio_map gpio = {
  .port   = {BARO_CS_PORT, GYRO_CS_PORT, ACCL_CS_PORT},
  .pin    = {BARO_CS_PIN, GYRO_CS_PIN, ACCL_CS_PIN},
  .drdy   = {BARO_MASK, GYRO_MASK, ACCL_MASK},
  .offset = {0x2u, 0x1u, 0x2u}
};

static const uint8_t tx[Sensors][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [0][1 ... 7] = 0x00,
  [1][0] = GYRO_TX_BYTE, [1][1 ... 7] = 0x00,
  [2][0] = ACCL_TX_BYTE, [2][1 ... 7] = 0x00,
};

static volatile uint8_t dmarx[SENSOR_BUF_SIZE] = {0};

static uint8_t taskrx[Sensors][SENSOR_BUF_SIZE - 2] = {0};

static atomic_uint_fast8_t locks[Sensors] = {0};

static dmasel select = {Sensors, 0};

static fdma flags = {0, 0};


static inline void lock(sens k)
{
  fu8 unlocked = 0;

  /* Spinlock with immediate context switch? What the fuck?
   * In our case this makes sense because critical sections
   * are too tiny to use blocking primitives and regular spinlock
   * wouldn't concede the only physical core to a contender thread.
   */
  while (!cas_strong(&locks[k], &unlocked, 1, Acq, Rlx))
  {
    unlocked = 0;
    tx_thread_relinquish();
  }
}

static inline void unlock(sens k)
{
  store(&locks[k], 0, Rel);
}


/*
 * Tries to fetch barometer from shared buffer.
 * Returns true on success and false otherwise.
 */
bool fetch_baro(baro *buf)
{
  if (!(fetch_and(&flags.relv, ~BARO_MASK, Acq) & BARO_MASK))
  {
    return false;
  }

  fu32 pres, temp;

  sweetbench_catch(0);

  lock(Sensor_Baro);

  pres = U24(taskrx[Sensor_Baro][0],
             taskrx[Sensor_Baro][1],
             taskrx[Sensor_Baro][2]);

  temp = U24(taskrx[Sensor_Baro][3],
             taskrx[Sensor_Baro][4],
             taskrx[Sensor_Baro][5]);

  unlock(Sensor_Baro);

  buf->tmp = baro_compensate_temp(temp);
  buf->prs = baro_compensate_pres(pres);
  buf->alt = baro_relative_alt(buf->prs);

  sweetbench_start(0, 150);
  
  return true;
}

/*
 * Tries to fetch gyroscope from shared buffer.
 * Returns true on success and false otherwise.
 */
bool fetch_gyro(f_xyz *buf)
{
  if (!(fetch_and(&flags.relv, ~GYRO_MASK, Acq) & GYRO_MASK))
  {
    return false;
  }

  fi16 gx, gy, gz;

  sweetbench_catch(1);

  lock(Sensor_Gyro);

  gx = I16(taskrx[Sensor_Gyro][0], taskrx[Sensor_Gyro][1]);
  gy = I16(taskrx[Sensor_Gyro][2], taskrx[Sensor_Gyro][3]);
  gz = I16(taskrx[Sensor_Gyro][4], taskrx[Sensor_Gyro][5]);
  
  unlock(Sensor_Gyro);

  buf->x = gx * inv_sens[init_rng];
  buf->y = gy * inv_sens[init_rng];
  buf->z = gz * inv_sens[init_rng];

  sweetbench_start(1, 150);

  return true;
}

/*
 * Tries to fetch accelerometer from shared buffer.
 * Returns true on success and false otherwise.
 */
bool fetch_accl(f_xyz *buf)
{
  if (!(fetch_and(&flags.relv, ~ACCL_MASK, Acq) & ACCL_MASK))
  {
    return false;
  }

  fi16 ax, ay, az;

  sweetbench_catch(2);

  lock(Sensor_Accl);

  ax = I16(taskrx[Sensor_Accl][0], taskrx[Sensor_Accl][1]);
  ay = I16(taskrx[Sensor_Accl][2], taskrx[Sensor_Accl][3]);
  az = I16(taskrx[Sensor_Accl][4], taskrx[Sensor_Accl][5]);
  
  unlock(Sensor_Accl);

  buf->x = ax * lsb_to_g;
  buf->y = ay * lsb_to_g;
  buf->z = az * lsb_to_g;

  sweetbench_start(2, 150);

  return true;
}


/*
 * Success callback. Sets valid flag, invalidates data cache
 * for the DMA buffer and wakes DMA task.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	gpio_cs_high(select.next);
  invalidate_dcache_addr_int(dmarx, sizeof dmarx);
  select.valid = 1;
	tx_thread_wait_abort(&dma_task);
}

/*
 * Error callback. Sets invalid flag and wakes DMA task.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	gpio_cs_high(select.next);
  select.valid = 0;
	tx_thread_wait_abort(&dma_task);
}

/*
 * Sets data readiness flag for a sensor that rose SPI edge.
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	(void) fetch_or(&flags.drdy, GPIO_Pin, Rel);
}


/*
 * Propagates raw data from DMA buffer to one of the shared
 * buffers.
 */
static inline void propagate_rx(void)
{
  lock(select.next);

  memcpy(taskrx[select.next],
         (uint8_t *)(dmarx + gpio.offset[select.next]),
         sizeof taskrx / 3);

  unlock(select.next);

  /* Relevance flags use the same EXTI pin masks. */
  (void) fetch_or(&flags.relv, gpio.drdy[select.next], Rel);
}

/*
 * Begins DMA transfer and blocks for a timeout until it
 * elapses or until the task is woken by a callback.
 */
static inline void start_dma_transfer(void)
{
  HAL_StatusTypeDef st;

  gpio_cs_low(select.next);

  st = dma_spi_txrx(tx[select.next], (uint8_t *)dmarx, SENSOR_BUF_SIZE);

  if (st != HAL_OK)
  {
    gpio_cs_high(select.next);
    return;
  }

  /* Sensor's drdy flag is erased after starting DMA transfer
   * but before a callback fires. This is a compromise between 
   * a) erasing a flag despite possible SPI errors (see right
   * above) and b) almost certainly erasing a newer drdy flag.
   */
  (void) fetch_and(&flags.drdy, ~gpio.drdy[select.next], Rlx);

  /* As a lightweght substitute for a binary condvar,
   * this suspension should be aborted by a callback.
   */
  st = tx_thread_sleep(DMA_TIMEOUT_MS);

  if (st == TX_WAIT_ABORTED && select.valid)
  {
    propagate_rx();
  }
}


/*
 * Loops sensor selector and blocks on DMA transfers.
 * This IO task should be "futex"-ed most of the time.
 */
void dma_entry(ULONG input)
{
  (void) input;

  fu16 drdy_snapshot;
  fu16 relv_snapshot;

  task_loop (DO_NOT_EXIT)
  {
    drdy_snapshot = load(&flags.drdy, Acq);

    if (load(&g_conf, Acq) & option(Using_Ascent_KF))
    {
      if (drdy_snapshot == 0)
      {
        tx_thread_relinquish();
        continue;
      }

      bool dr_b = drdy_snapshot & BARO_MASK;
      bool dr_g = drdy_snapshot & GYRO_MASK;
      bool dr_a = drdy_snapshot & ACCL_MASK;

      relv_snapshot = load(&flags.relv, Acq);

      bool re_b = relv_snapshot & BARO_MASK;
      bool re_g = relv_snapshot & GYRO_MASK;
      bool re_a = relv_snapshot & ACCL_MASK;

      /* The selector is defined as a simplified boolean expression
       * of 6 inputs (and 2^6 - 6 possible outcomes).
       */
      if (dr_g && ((!dr_b && (re_g || !dr_a)) || (!re_g && (!dr_a ||
                                                    re_a || re_b))))
      {
        select.next = Sensor_Gyro;
      }
      else if (dr_a && (((!dr_g || re_g) && (!dr_b || !re_a)) ||
                                            (dr_g && re_b && re_g)))
      {
        select.next = Sensor_Accl;
      }
      else
      {
        select.next = Sensor_Baro;
      }
    }
    else
    {
      if (drdy_snapshot & BARO_MASK)
      {
        select.next = Sensor_Baro;
      }
      else
      {
        tx_thread_relinquish();
        continue;
      }
    }
    
    start_dma_transfer();
  }
}

/*
 * Creates a preemptive, cooperative DMA task with defined
 * parameters.
 */
UINT create_dma_task(TX_BYTE_POOL *byte_pool)
{
	UINT st;
  CHAR *pointer;

	const char *critical = "creation failure:";

  st = tx_byte_allocate(byte_pool, (VOID **)&pointer,
												DMA_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    log_die(id "pool %s %u", critical, st);
  }

  st = tx_thread_create(&dma_task,
                        "DMA Task",
                        dma_entry,
                        DMA_INPUT,
                        pointer,
                        DMA_STACK_BYTES,
                        DMA_PRIORITY,
                        /* No preemption threshold */
                        DMA_PRIORITY,
                        TX_NO_TIME_SLICE,
                        TX_AUTO_START);

	if (st != TX_SUCCESS)
  {
    log_die(id "task %s %u", critical, st);
  }

  return TX_SUCCESS;
}
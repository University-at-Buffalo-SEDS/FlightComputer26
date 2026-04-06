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
#include "sweetbench.h"
#include "recovery.h"
#include "dma.h"

#define id "DM "

TX_THREAD dma_task;


/* ------ Static storage ------ */

static const struct gpio_lookup gpio = {
  .port = {CS_BARO_GPIO_Port, CS_GYRO_GPIO_Port, CS_ACCEL_GPIO_Port},
  .pin  = {CS_BARO_Pin, CS_GYRO_Pin, CS_ACCEL_Pin},
  .drdy = {BARO_MASK, GYRO_MASK, ACCL_MASK}
};

static const uint8_t tx[Sensors][SENSOR_BUF_SIZE] = {
  [0][0] = BARO_TX_BYTE, [0][1 ... 7] = 0x00,
  [1][0] = GYRO_TX_BYTE, [1][1 ... 7] = 0x00,
  [2][0] = ACCL_TX_BYTE, [2][1 ... 7] = 0x00,
};

/* IREC 2027: include temperature (indices 3, 4, 5). */
static uint8_t baro_rx[6] = {0};
static uint8_t gyro_rx[6] = {0};
static uint8_t accl_rx[6] = {0};

static atomic_uint_fast8_t locks[Sensors] = {0};

static volatile uint8_t rx[SENSOR_BUF_SIZE] = {0};

static struct selector select = {Sensors, 0};

static struct dma_flags flags = {0, 0};

#ifdef DMA_BENCHMARK

/* HAL timestamp for consumer benchmark.
 * Stores relative time when RxCplt finishes transfer. */
static atomic_uint_fast32_t rxts[Sensors] = {0};

#endif // DMA_BENCHMARK

/* ------ Static storage ------ */


/* ------ Spinlock ------ */

static inline void lock(enum sensor k)
{
  fu8 unlocked = 0;

  /* Spinlock with context switch? What the fuck?
   * But in our case it makes sense because critical sections
   * are too tiny to use blocking primitives and regular spinlock
   * wouldn't concede the only physical core to a contender thread.
   */
  while (!cas_strong(&locks[k], &unlocked, 1, Acq, Rlx))
  {
    unlocked = 0;
    tx_thread_relinquish();
  }
}

static inline void unlock(enum sensor k)
{
  store(&locks[k], 0, Rel);
}

/* ------ Spinlock ------ */


/* ------ Public fetching functions ------ */

/*
 * Tries to fetches barometer data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_baro(struct baro *buf)
{
  if (!(fetch_and(&flags.relv, ~BARO_MASK, Acq) & BARO_MASK))
  {
    return false;
  }

  fu32 pres, temp;

  sweetbench_catch(0);

  lock(Sensor_Baro);

  pres = U24(baro_rx[0], baro_rx[1], baro_rx[2]);
  temp = U24(baro_rx[3], baro_rx[4], baro_rx[5]);

  unlock(Sensor_Baro);

  buf->t = baro_compensate_temp(temp);
  buf->p = baro_compensate_pres(pres);
  buf->alt = baro_relative_alt(buf->p);

  sweetbench_start(0, 10, false);
  
  return true;
}

/*
 * Tries to fetches gyroscope data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_gyro(struct coords *buf)
{
  if (!(fetch_and(&flags.relv, ~GYRO_MASK, Acq) & GYRO_MASK))
  {
    return false;
  }

  fi16 gx, gy, gz;

  sweetbench_catch(1);

  lock(Sensor_Gyro);

  gx = I16(gyro_rx[0], gyro_rx[1]);
  gy = I16(gyro_rx[2], gyro_rx[3]);
  gz = I16(gyro_rx[4], gyro_rx[5]);
  
  unlock(Sensor_Gyro);

  buf->x = gx * inv_sens[init_rng];
  buf->y = gy * inv_sens[init_rng];
  buf->z = gz * inv_sens[init_rng];

  sweetbench_start(1, 10, false);

  return true;
}

/*
 * Tries to fetches acceletometer data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_accl(struct coords *buf)
{
  if (!(fetch_and(&flags.relv, ~ACCL_MASK, Acq) & ACCL_MASK))
  {
    return false;
  }

  fi16 ax, ay, az;

  sweetbench_catch(2);

  lock(Sensor_Accl);

  ax = I16(accl_rx[0], accl_rx[1]);
  ay = I16(accl_rx[2], accl_rx[3]);
  az = I16(accl_rx[4], accl_rx[5]);
  
  unlock(Sensor_Accl);

  buf->x = ax * lsb_to_g;
  buf->y = ay * lsb_to_g;
  buf->z = az * lsb_to_g;

  sweetbench_start(2, 10, false);

  return true;
}

/* ------ Public fetching functions ------ */


/* ------ ISR and callbacks ------ */

/*
 * Sets valid flag and wakes DMA task.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	(void) hspi; /* We only set up SPI1 */

	gpio_cs_high(select.next);
  invalidate_dcache_addr_int(rx, sizeof rx);
  select.valid = 1;
	tx_thread_wait_abort(&dma_task);
}

/*
 * Sets invalid flag and wakes DMA task.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	(void) hspi; /* We only set up SPI1 */

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

/* ------ ISR and callbacks ------ */


/* ------ Transfer routine ------ */

/*
 * Propagates raw data from DMA buffer to one of the shared
 * buffers.
 */
static inline void propagate_rx(void)
{
  lock(select.next);

  switch (select.next)
  {
    case Sensor_Baro:
      memcpy(baro_rx, (uint8_t *)(rx + 2), sizeof baro_rx);
      break;

    case Sensor_Gyro:
      memcpy(gyro_rx, (uint8_t *)(rx + 1), sizeof gyro_rx);
      break;

    case Sensor_Accl:
      memcpy(accl_rx, (uint8_t *)(rx + 2), sizeof accl_rx);
      break;
  }

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

  st = dma_spi_txrx(tx[select.next], (uint8_t *)rx, SENSOR_BUF_SIZE);

  if (st != HAL_OK)
  {
    /* Immediate error while trying to start a transfer => 
     * => repeat the same transfer on the next iteration. */
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

/* ------ Transfer routine ------ */


/* ------ DMA task and sensor selector ------ */

/*
 * Beginning of the DMA task. This task should started once
 * and not blocked or reset, though it is technically AC-Safe.
 * It includes a minimized expression for a selector of 64
 * (2^6) possible states, where are encoded in a truth table
 * over 6 variables - data readiness and relevance flags for
 * each of 3 sensors. Data readiness is updated from and ISR,
 * and relevance is set when a buffer is produced and unset
 * when it is consumed.
 */
void dma_entry(ULONG input)
{
  (void) input;

  fu16 drdy_snapshot;
  fu16 relv_snapshot;

  task_loop (DO_NOT_EXIT)
  {
    drdy_snapshot = load(&flags.drdy, Acq);

    if (load(&config, Acq) & option(Using_Ascent_KF))
    {
      if (drdy_snapshot == 0)
      {
        tx_thread_relinquish();
      }

      /* Step 1. Calculate inputs to selection logic.
       */ 
      bool dr_b = drdy_snapshot & BARO_MASK;
      bool dr_g = drdy_snapshot & GYRO_MASK;
      bool dr_a = drdy_snapshot & ACCL_MASK;

      relv_snapshot = load(&flags.relv, Acq);

      bool re_b = relv_snapshot & BARO_MASK;
      bool re_g = relv_snapshot & GYRO_MASK;
      bool re_a = relv_snapshot & ACCL_MASK;

      /* Step 2. Select transfer target based on drdy and alternating
       * selectors p_dev and p_imu. The below expressions are results
       * of Quine-McCluskey minimization for drdy and relv flag sets, 
       * each consisting of 3 sensors. Since expressions are mutually
       * exclusive on the domain of 64 states, the most expensive one
       * (in our case - Barometer) is evaluated with wildcard 'else'.
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
      }
    }
    
    start_dma_transfer();
  }
}

/*
 * Creates a preemptive, cooperative DMA task with defined parameters.
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

/* ------ DMA task and sensor selector ------ */
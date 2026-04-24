/* Core/Inc/fcapi.h */

#ifndef FC_API
#define FC_API

#include "fctypes.h"
#include "platform.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fcconfig.h"
#include "sweetbench.h"


/* DMA */

bool try_fetch_baro(baro *);
bool try_fetch_gyro(f_xyz *);
bool try_fetch_accl(f_xyz *);


/* Kalman filter */

#ifdef PARALLEL_PREDICT_UPDATE
extern TX_BYTE_POOL kfpool;
#endif

extern quat qv;
extern kf_svec imedsv;
extern void *kfpool_buf;

void descent_predict(const float);
void descent_update(void);
void descent_initialize(void);

void ascent_predict(const float, fu32);
void ascent_update(void);
void ascent_initialize(fu32);

void accel_to_quaternion(const f_xyz *);


/* Evaluation */

extern TX_EVENT_FLAGS_GROUP eval_stage;

extern kf_svec sv[];
extern sv_meta sm;
extern measm meas;
extern spinlock meas_locks[];

void evaluate_rocket_state(fu32);

#ifdef TELEMETRY_ENABLED
SedsResult on_fc_packet(const SedsPacketView *, void *);
#endif

static inline void
log_metric(const char *msg, float metric, bool critical)
{
  char buf[MAX_METRIC_REPORT_SIZE];
  
  sprintf(buf, "%s: %.*g\n", msg,
          FLOAT_LOG_PRECISION, metric);

  critical ? log_critical(buf) : log_msg(buf);
}


/* LED */

static inline void blink(volatile fu32 count)
{
  volatile fu32 delay;

  do {
    toggle_green_led();
    delay = LED_BLOCKING_CYCLES;

    while (delay--)
    {
      __NOP();
    }
  } while (--count);
}


/* Recovery */

extern TX_QUEUE shared;
extern atomic_uint_fast32_t g_conf;

static inline void clear_spi1_irq(void)
{
  irq_off(SPI1_GLOBAL_IRQ);
  irq_off(DMA_RECEIVER_SPI1);

  irq_off(Baro_EXTI);
  irq_off(Gyro_EXTI_1);
/*irq_off(Gyro_EXTI_2);   not used for IREC 2026 */
  irq_off(Accl_EXTI_1);
/*irq_off(Accl_EXTI_2);   not used for IREC 2026 */
}

static inline void restore_spi1_irq(void)
{
  irq_on(SPI1_GLOBAL_IRQ);
  irq_on(DMA_RECEIVER_SPI1);

  irq_on(Baro_EXTI);
  irq_on(Gyro_EXTI_1);
/*irq_on(Gyro_EXTI_2);    not used for IREC 2026 */
  irq_on(Accl_EXTI_1);
/*irq_on(Accl_EXTI_2);    not used for IREC 2026 */
}


/* Ignition */

static inline SedsResult request_ignition(void)
{
  const fu8 igniter_seq = IGNITION_COMMAND;
  return log_valve_board_command(igniter_seq);
}


/* Timer */

extern volatile fu32 local_time[Time_Users];

static inline fu32 timer_exchange(timer u)
{
  fu32 prev = local_time[u];
  local_time[u] = now_ms();
  return local_time[u] - prev;
}

static inline void timer_update(timer u)
{
  local_time[u] = now_ms();
}

static inline fu32 timer_fetch(timer u)
{
  return now_ms() - local_time[u];
}


/* Flight state checking */

static inline state current(void)
{
  return load(&sm.flight, Acq);
}

static inline bool beyond(state bound)
{
#ifdef LUNATIC_STATE
  return true;
#else
  return current() > bound;
#endif
}


/* Deployment routines */

static inline bool release_parachute(void)
{
  if (!beyond(Launch))
  {
    log_err("PD drogue blocked, state %u", current());
    return false;
  }

  co2_high();
  log_metric("PD exact altitude", svec(0).alt, true);

  sweetbench_start(4, 1);

  timer_update(AssertCO2);
  fetch_or(&g_conf, option(Parachute_Deployed | CO2_Asserted), Rel);

  return true;
}

static inline bool expand_parachute(void)
{
  if (!beyond(Launch))
  {
    log_err("PR reef expansion, state %u", current());
    return false;
  }
  
  if (!(load(&g_conf, Acq) & option(Parachute_Deployed)))
  {
    log_err("PR reef blocked: not drogue");
    return false;
  }

  reef_high();
  log_metric("PR exact altitude", svec(0).alt, true);

  sweetbench_start(4, 1);

  timer_update(AssertREEF);
  fetch_or(&g_conf, option(Parachute_Expanded | REEF_Asserted), Rel);

  return true;
}


/* Spinlock */

static inline void fc_lock(spinlock *object)
{
  fu8 unlocked = 0;

  while (!cas_strong(&object->lock, &unlocked, 1, Acq, Rlx))
  {
    unlocked = 0;
    fetch_add(&object->waiters, 1, Rel);
    tx_thread_relinquish();
    fetch_sub(&object->waiters, 1, Rlx);
  }
}

static inline void fc_unlock(spinlock *object, bool yield)
{
  store(&object->lock, 0, Rel);

  /* Wakeyield: for single-core, signle data cache,
                with a FIFO queue for scheduler threadpool */

  if (yield && load(&object->waiters, Acq) > 0)
  {
    tx_thread_relinquish();
  }
}


#endif /* FC_API */
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


/* SD pipeline */

void sd_pipeline_task(void);
void sd_append_f32(SedsDataType, const float *, fu8);
void sd_append_string(SedsDataType, const char *);


/* Kalman filter */

#ifdef PARALLEL_PREDICT_UPDATE
extern TX_BYTE_POOL kfpool;
#endif

extern quat qv;
extern kf_svec imedsv;
extern void *kfpool_buf;

void descent_predict(const float);
void descent_update(void);
void descent_initialize(fu32);

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

void evaluate_rocket_state(fu32, float);

#ifdef TELEMETRY_ENABLED
SedsResult on_fc_packet(const SedsPacketView *, void *);
#endif


/* Recovery */

extern TX_QUEUE shared;
extern atomic_uint_fast32_t g_conf;

extern TX_BYTE_POOL *tx_app_shared;

extern volatile log_rates rates;

void try_allocate_reserve_pool(void);

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

#ifdef TELEMETRY_ENABLED

static inline SedsResult request_ignition(void)
{
  const fu8 igniter_seq = IGNITION_COMMAND;
  return log_valve_board_command(igniter_seq);
}

#else 

static inline fu8 request_ignition(void)
{
  log_critical("Debug ignition requested");
  return SEDS_OK;
}

#endif /* TELEMETRY_ENABLED */


/* Blocking indicator (panic, debug) */

extern const led_gpio light[];

static inline void blink(led kind, bool slow,
                         volatile fu32 count)
{
  volatile fu32 cycles;
  const fu32 delay = slow
                   ? LED_BLOCKING_CYCLES * 4
                   : LED_BLOCKING_CYCLES;

  while (count--)
  {
    led_on(light[kind].port, light[kind].pin);

    cycles = delay;
    while (--cycles) __NOP();

    led_off(light[kind].port, light[kind].pin);

    cycles = delay;
    while (--cycles) __NOP();
  }
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

static inline bool timer_probe(timer u, fu32 timeout)
{
  if (timer_fetch(u) >= timeout)
  {
    timer_update(u);
    return true;
  }
  
  return false;
}


/* Flight state helpers */

static inline state current(void)
{
  return load(&sm.flight, Acq);
}

static inline bool beyond(state bound)
{
  return current() > bound;
}

static inline gnd_state *to_global_state(state local)
{
  switch (local) {
    case Startup:   sm.global_state = G_Startup;    break;
    case Postinit:  sm.global_state = G_Postinit;   break;
    case Armed:     sm.global_state = G_Armed;      break;
    case Launch:    sm.global_state = G_Launch;     break;
    case Ascent:    sm.global_state = G_Ascent;     break;
    case Coast:     sm.global_state = G_Coast;      break;
    case Apogee:    sm.global_state = G_Apogee;     break;
    case Descent:   sm.global_state = G_Descent;    break;
    case Reefing:   sm.global_state = G_Reefing;    break;
    case Landed:    sm.global_state = G_Landed;     break;
    case Recovery:  sm.global_state = G_Recovery;   break;

    /* Something went terribly wrong */
    default:        sm.global_state = G_Aborted;
  }

  return &sm.global_state;
}


/* Single-threaded spinlock */

static inline void fc_lock(spinlock *object)
{
  fu8 unlocked = 0;

  while (!cas_weak(&object->lock, &unlocked, 1, Acq, Rlx))
  {
    unlocked = 0;
    fetch_add(&object->waiters, 1, Rel);
    tx_thread_relinquish();
    fetch_sub(&object->waiters, 1, Rlx);
  }
}

static inline bool fc_trylock(spinlock *object)
{
  fu8 unlocked = 0;
  return cas_strong(&object->lock, &unlocked, 1, Acq, Rlx);
}

static inline void fc_unlock(spinlock *object)
{
  store(&object->lock, 0, Rel);
}

static inline void fc_concede(spinlock *object)
{
  fc_unlock(object);

  if (load(&object->waiters, Acq) > 0)
  {
    tx_thread_relinquish();
  }
}


/* Logging wrappers */

static inline void message(const char *msg, bool critical)
{
#ifdef SD_AVAILABLE
  if (critical || timer_probe(MessageLocal, rates.sd))
  {
    SedsDataType ty = critical ? SEDS_DT_ORDERED_MESSAGE
                               : SEDS_DT_MESSAGE_DATA;
    sd_append_string(ty, msg);
  }
#endif

  if (critical)
  {
    log_critical(msg);
  }
  else if (timer_probe(MessageRemote, rates.gnd))
  {
    log_msg(msg);
  }
}

/* TODO logging errors without double vsnprintf */

static inline void
log_metric(const char *msg, fi32 metric, bool critical)
{
  char buf[MAX_METRIC_REPORT_SIZE];
  int n = snprintf(buf, sizeof buf, "%s: %d", msg, metric);

  if (n > 0 && n < sizeof buf)
  {
    message(buf, critical);
  }
}


/* Deployment routines */

static inline bool release_parachute(bool force)
{
  if (!force && !beyond(Armed))
  {
    log_err("PD drogue blocked, state %u", current());
    return false;
  }

  co2_high();
  log_metric("PD approx altitude", svec(0).alt, true);

  sweetbench_start(4, 1);

  timer_update(AssertCO2);
  fetch_or(&g_conf, option(Parachute_Deployed | CO2_Asserted), Rel);

  return true;
}

static inline bool expand_parachute(bool force)
{
  if (!force && !beyond(Launch))
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
  log_metric("PR approx altitude", svec(0).alt, true);

  sweetbench_start(4, 1);

  timer_update(AssertREEF);
  fetch_or(&g_conf, option(Parachute_Expanded | REEF_Asserted), Rel);

  return true;
}


/* Debug type descriptors */

#if !defined(TELEMETRY_ENABLED) && defined(USB_ENUMERATES)

static inline char *debug_descriptor(SedsDataType logged)
{
  switch (logged) {
    case SEDS_DT_BAROMETER_DATA:  return "BaroRemote";
    case SEDS_DT_GYRO_DATA:       return "GyroRemote";
    case SEDS_DT_ACCEL_DATA:      return "AccelRemote";
    case SEDS_DT_IMU_DATA:        return "IMURemote";
    case SEDS_DT_BAROMETER_LOCAL: return "BaroLocal";
    case SEDS_DT_GYRO_LOCAL:      return "GyroLocal";
    case SEDS_DT_ACCEL_LOCAL:     return "AccelLocal";
    case SEDS_DT_IMU_LOCAL:       return "IMULocal";
    case SEDS_DT_ASCENT_STATE:    return "AscentRemote";
    case SEDS_DT_DESCENT_STATE:   return "DescentRemote";
    case SEDS_DT_ASCENT_LOCAL:    return "AscentLocal";
    case SEDS_DT_DESCENT_LOCAL:   return "DescentLocal";
    case SEDS_DT_EULER_ANGLES:    return "EulerAngles";
    case SEDS_DT_MESSAGE_DATA:    return "MessageData";
    case SEDS_DT_ORDERED_MESSAGE: return "OrderedMessage";
    case SEDS_DT_GENERIC_ERROR:   return "GenericError";
    default:                      return "SomeRandomData";
  }
}

static inline char *debug_g_state(gnd_state state)
{
  switch (state) {
    case G_Startup:       return "Startup";
    case G_Idle:          return "Idle";
    case G_PreFill:       return "PreFill";
    case G_FillTest:      return "FillTest";
    case G_NitrogenFill:  return "NitrogenFill";
    case G_NitrousFill:   return "NitrousFill";
    case G_Postinit:      return "Postinit";
    case G_Armed:         return "Armed";
    case G_Launch:        return "Launch";
    case G_Ascent:        return "Ascent";
    case G_Coast:         return "Coast";
    case G_Apogee:        return "Apogee";
    case G_Descent:       return "Descent";
    case G_Reefing:       return "Reefing";
    case G_Landed:        return "Landed";
    case G_Recovery:      return "Recovery";
    case G_Aborted:       return "Aborted";
    default:              return "Unknown";
  }
}

#endif /* !TELEMETRY_ENABLED && USB_ENUMERATES */


#endif /* FC_API */
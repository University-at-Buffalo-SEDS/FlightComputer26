/*
 * Flight Computer internal API header.
 */

#ifndef FC_API
#define FC_API

#include "fctypes.h"
#include "platform.h"
#include "fcstructs.h"
#include "fccommon.h"


/* DMA */

bool fetch_baro(baro *);
bool fetch_gyro(f_xyz *);
bool fetch_accl(f_xyz *);


/* Kalman filter */

void descent_predict(const float);
void descent_update(const float);
void descent_initialize(void);

void ascent_predict(const float);
void ascent_update(const float);
void ascent_initialize(void);


/* Evaluation */

extern TX_SEMAPHORE eval_focus_mode;

extern kf_svec sv[];
extern sv_meta sm;
extern measm meas;

extern const char *trans[];
extern volatile state flight;

SedsResult on_fc_packet(const SedsPacketView *, void *);

void evaluate_rocket_state(fu32);

static inline void
log_transition(const char *task, float metric)
{
  char buf[MAX_REPORT_SIZE];
  
  snprintf(buf,
           8 + sizeof(trans[flight]) + FLOAT_LOG_PRECISION,
           "%s%s %.*g\n",
           task, trans[flight],
           FLOAT_LOG_PRECISION,
           metric);

  log_msg(buf);
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


/* Deployment routines */

static inline bool release_parachute(void)
{
  if (flight < Ascent)
  {
    log_err("SE blocked deployment, state %u", flight);
    return false;
  }

  co2_high();

  timer_update(AssertCO2);
  fetch_or(&g_conf, option(Parachute_Deployed | CO2_Asserted), Rel);

  return true;
}

static inline bool expand_parachute(void)
{
  if (flight < Ascent)
  {
    log_err("ND blocked expansion, state %u", flight);
    return false;
  }
  
  if (!(load(&g_conf, Acq) & option(Parachute_Deployed)))
  {
    log_err("ND blocked expansion: no deployment");
    return false;
  }

  reef_high();

  timer_update(AssertREEF);
  fetch_or(&g_conf, option(Parachute_Expanded | REEF_Asserted), Rel);

  return true;
}


#endif /* FC_API */
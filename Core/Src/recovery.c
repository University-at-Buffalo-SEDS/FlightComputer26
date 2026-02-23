/*
 * Recovery Task
 *
 * This task suspends on a blocking message queue
 * and when resumed, drains the message queue. The
 * messages are then interpreted depending on the
 * MSB designating the endpoint - FC or GND. When
 * a message appears in the queue, this thread is 
 * _immediately_ woken by ThreadX and processing begins.
 *
 * This task provides for various recovery and abortion
 * scenarios, and also logs all important actions. This
 * task as a sole command and decision center of the
 * flight computer, and therefore cannot be preempted
 * while in execution. As such, utlizing the fact that
 * the target MCU is single core and thus has only one
 * data cache unit, this task can perform non-atomic
 * reads and writes across its allotted RAM region. 
 *
 * This task also includes a timer callback that
 * checks for timeout of FC, RF and GND. If timeout
 * occurs (i.e., this task did not receive a signal)
 * for 4 seconds, a handler is invoked that changes
 * global runtime configuration, which other tasks
 * read and locally update on every iteration. The
 * ThreadX timer is set to expire every second.
 *
 * To change default configuration applied during boot,
 * refer to recovery.h -> DEFAULT_OPTIONS.
 *
 * This task defines the local timer, which informs its
 * users of time elapsed since last call for each user.
 * This timer is independent of the ThreadX timer (above),
 * and relies on HAL tick that is incremented every 1 ms.
 */

#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "kalman.h"
#include "dma.h"

TX_QUEUE shared;
TX_THREAD recovery_task;
ULONG recovery_stack[RECV_STACK_ULONG];


/* ------ Global and static storage ------ */

#define id "FC:RECV: "

/* Last recorded time for each timer user. */
volatile fu32 local_time[Time_Users] = {0};

/* Run time configuration mask. */
atomic_uint_fast32_t config = DEFAULT_OPTIONS;

/* Recovery queue pool */
#define RECVQ_SIZE 64
static tx_align enum message recvq_pool[RECVQ_SIZE] = {0};

/* Monitors timeouts and deployment events */
static TX_TIMER monotonic_checks;

/* Static raw data status counters */
static volatile fu16 failures = 0;

static fu16 to_abort  = TO_ABORT;
static fu16 to_reinit = TO_REINIT;

static fu16 gps_delay_count = 0;
static fu16 gps_malform_count = 0;

/* Sensor configurations,
 * modified and passed to init at runtime */

static struct baro_config baro_conf = {
  .osr_t = BARO_OSR_X1,
  .osr_p = BARO_OSR_X2,
  .odr = BARO_DEFAULT_ODR_SEL,
  .iir_coef = BARO_IIR_COEF_0,
};

static struct gyro_config gyro_conf = {
  .rng = Gyro_Range_2000Dps,
  .bw  = Gyro_532Hz_ODR_2000Hz,
};

static struct accl_config accl_conf = {
  .mode = Normal_1600Hz,
  .rng  = Accl_Range_24g,
};

/* ------ Global and static storage ------ */


/* ------ Recovery logic ------ */

/*
 * (Re)initializes indicated sensor(s). Disables
 * DMA interrupt for the duration of reinitialization
 * of concerned sensor. This is the place to perform
 * sensitive initializations, because Recovery cannot
 * be preempted by other tasks. As the driver init
 * functions are idempotent, so is this function.
 */
static inline void
initialize_sensors(enum sensor_mask sensor)
{
  enum sensor_mask fails = 0;

  if (sensor & Init_Baro)
  {
    __NVIC_DisableIRQ(Baro_EXTI);
    reinit(init_baro(&baro_conf), fails, Init_Baro);
    __NVIC_EnableIRQ(Baro_EXTI);
  }

  if (sensor & Init_Gyro)
  {
    __NVIC_DisableIRQ(Gyro_EXTI_1);
    __NVIC_DisableIRQ(Gyro_EXTI_2);
    reinit(init_gyro(&gyro_conf), fails, Init_Gyro);
    __NVIC_EnableIRQ(Gyro_EXTI_1);
    __NVIC_EnableIRQ(Gyro_EXTI_2);
  }

  if (sensor & Init_Accl)
  {
    __NVIC_DisableIRQ(Accl_EXTI_1);
    __NVIC_DisableIRQ(Accl_EXTI_2);
    reinit(init_accl(&accl_conf), fails, Init_Accl);
    __NVIC_EnableIRQ(Accl_EXTI_1);
    __NVIC_EnableIRQ(Accl_EXTI_2);
  }
  
  if (fails != 0) {
    log_err(id "some of requested reinit failed: %u", fails);
  }

  config |= option(Reinit_Attempted);
}

/*
 * Reset Distribution and Evaluation tasks. Grants Telemetry
 * task full scheduling time. If launch signal is sent after
 * abortion, the FC will attempt to resume normal opertaion.
 */
static inline void auto_abort(void)
{
  tx_thread_reset(&evaluation_task);
  tx_thread_reset(&distribution_task);

  gps_delay_count = 0;
  gps_malform_count = 0;

  if (config & option(Lost_GroundStation))
  {
    /* Nowhere to expect commands from! Reinitialize. */
    to_abort = TO_ABORT * 10;

    enum message cmd = fc_mask(Launch_Signal);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }
  else
  {
    log_msg(id "aborted! Expecting commands "
               "or launch signal.", mlen(45));

    config |= option(In_Aborted_State);
  }
}

/*
 * Signals other tasks to not fetch or use GPS data.
 * Reinitializes barometer with larger oversampling rates.
 */
static inline void barometer_fallback(void)
{
  config &= ~option(GPS_Available);

  baro_conf.osr_p = BARO_OSR_X8;
  baro_conf.iir_coef = BARO_IIR_COEF_15;

  initialize_sensors(Init_Baro);
  
  if (!(config & option(GPS_Available)))
  {
    config |= option(Monitor_Altitude);
    log_msg(id "Entered vigilant mode", mlen(26));
  }
}

/*
 * Process general command from either endpoint.
 */
static inline void process_action(enum message cmd)
{
  UINT eval_old_pt;

  switch (cmd) {
    case Deploy_Parachute:
      co2_high(&config);
      return;

    case Expand_Parachute:
      if (config & option(Parachute_Deployed)) {
        reef_high(&config);
      } else {
        log_err(id "rejected REEF before PYRO");
      }
      return;

    case Reinit_Sensors:
      initialize_sensors(Init_All);
      return;

    case Launch_Signal:
      if (gps_delay_count || gps_malform_count)
      {
        log_err(id "GPS: %u delayed and %u malformed "
                "during pre-launch. Counters are reset.",
                gps_delay_count, gps_malform_count);

        gps_delay_count = 0;
        gps_malform_count = 0;
      }

      if (config & option(In_Aborted_State))
      {
        tx_thread_resume(&distribution_task);
        config &= ~option(In_Aborted_State);
      }

      /* Start Evaluation and begin rocket launch chain. */
      tx_thread_resume(&evaluation_task);
      return;

    case Evaluation_Relax:
      /* Allow preemption of Evaluation task inside KF. */
      config &= ~option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PRIORITY,
                                  &eval_old_pt);
      return;

    case Evaluation_Focus:
      /* Restrict preemption of Evaluation task inside KF. */
      config |= option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PREEMPT_THRESHOLD,
                                  &eval_old_pt);
      return;

    case Evaluation_Abort:
      config |= option(Eval_Abort_Flag);
      tx_thread_reset(&evaluation_task);
      return;

    case Reinit_Barometer:
      barometer_fallback();
      return;

    default: break;
  }
}

/*
 * Applies or revokes passed runtime configuration option.
 */
static inline void process_config(enum message code)
{
  if (code & KF_Operation_Mode)
  {
    renorm_step_mask = code & ~KF_Operation_Mode;
  }
  else if (code & Abortion_Thresholds)
  {
    to_abort = code & ~Abortion_Thresholds;
  }
  else if (code & Reinit_Thresholds)
  {
    to_reinit = code & ~Reinit_Thresholds;
  }
  else if (code & Revoke_Option)
  {
    config &= ~(code & ~Revoke_Option);
  }
  /* If nothing else matched, must have exactly 1 bit set
   * (i.e., a power of 2) to be a valid config option. */
  else if (!(code & (code - 1)))
  {
    config |= code;
  }
  else
  {
    log_err(id "invalid config option: %u", (unsigned)code);
  }
}

/*
 * Checks whether raw data report is OK.
 */
static inline void process_report(enum message code)
{
  if (code != Sensor_Measm_Code)
  {
    ++failures;
    log_err(id "dirty data report (%u)", (unsigned)code);

    if (failures >= to_abort) {
      auto_abort();
    } else if (failures >= to_reinit) {
      initialize_sensors(Init_All);
    }
  }
  else if (config & option(Reset_Failures))
  {
    failures = 0;
  }
}

/*
 * Handles delayed or malformed GPS data report.
 */
static inline void process_gps_code(enum message code)
{
  switch (code) {
    case GPS_Delayed:
      ++gps_delay_count;

      if (config & option(Launch_Triggered)) {
        log_err(id "delayed GPS packet (%u)", gps_delay_count);
        return;
      }

      if (gps_delay_count >= MAX_GPS_DELAYS) {
        barometer_fallback();
        return;
      }
      break;

    case GPS_Malformed:
      ++gps_malform_count;

      if (config & option(Launch_Triggered)) {
        log_err(id "malformed GPS packet (%u)", gps_malform_count);
        return;
      }

      if (gps_malform_count >= GPS_MAX_MALFORMED) {
        barometer_fallback();
        return;
      }
      break;

    default: break;
  }
}

/*
 * Decodes universal FC message from an endpoint {FC, RF, GND}.
 */
static inline void decode_message(enum message msg)
{
  if (msg & GroundStation_Heartbeat)
  {
    timer_update(HeartbeatGND);
  }
  else if (msg & Runtime_Configuration)
  {
    process_config(option(fc_unmask(msg)));
  }
  else if (msg & GPS_Data_Code)
  {
    process_gps_code(fc_unmask(msg));
  }
  else if (msg & Actionable_Decrees)
  {
    process_action(fc_unmask(msg));
  }
  else if (msg & Spurious_Confirmations)
  {
    msg &= ~(FC_Identifier | Spurious_Confirmations);
    log_err(id "unconfirmed transition: %u", (unsigned)msg);
  }
  else if (fc_unmask(msg))
  {
    process_report(fc_unmask(msg));
  }
}

/* ------ Recovery logic ------ */


/* ------ Timer ------ */

/*
 * This routine is called from ThreadX interrupt
 * every 0.5 seconds. It monitors timeout for the
 * endpoints flight computer depends on {FC, RF, GND},
 * and if timeout has occured, signals tasks to not
 * depend on the endpoint which has timed out. In case
 * of the FC itself, it tries to reinitialize vital
 * tasks 3 times and them aborts.
 *
 * This timer is also responsible for pulling CO2 and
 * REEF pins back low after either has been asserted.
 */
static void fc_timer_routine(ULONG timer_id)
{
  (void) timer_id;

  if (config & option(CO2_Asserted) &&
      timer_fetch(AssertCO2) >= CO2_ASSERT_INTERVAL)
  {
    co2_low();
    config &= ~option(CO2_Asserted);
  }

  if (config & option(REEF_Asserted) &&
      timer_fetch(AssertREEF) >= REEF_ASSERT_INTERVAL)
  {
    reef_low();
    config &= ~option(REEF_Asserted);
  }
  
  if (timer_fetch(HeartbeatGND) > GND_TIMEOUT)
  {
#ifdef TELEMETRY_ENABLED
    /* Vigilant mode */
    config |= option(Lost_GroundStation);
    config |= option(Monitor_Altitude);
    config |= option(Reset_Failures);
    renorm_step_mask = RENORM_STEP;
    failures = 0;

#else /* FOR TESTS ONLY */
    /* Auto "launch" on first timeout */
    static fu8 test_launched = 0;

    if (!test_launched) {
      tx_thread_resume(&evaluation_task);
      test_launched = 1;
    }

#endif // TELEMETRY_ENABLED
  }

  if (config & option(GPS_Available))
  {
    if (timer_fetch(HeartbeatRF) > GPS_DELAY_MS)
    {
      enum message cmd = fc_mask(GPS_Delayed);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }
  }
  else
  {
    if (timer_fetch(HeartbeatRF) < TX_TIMER_TICKS)
    {
      /* GPS became alive, announce */
      config |= option(GPS_Available);
    }
  }
}

/* ------ Timer ------ */


/* ------ Recovery task ------ */

/*
 * Upon entry, finishes Flight Computer boot sequence.
 * Suspends on blocking queue until a new message arrives.
 * Not idempotent. This task must be start first and never
 * reset until the device is powered off or rebooted.
 */
void recovery_entry(ULONG input)
{
  (void) input;

  UINT st;

  initialize_sensors(Init_All);

  for (enum fc_timer k = 0; k < Time_Users; ++k) {
    timer_update(k);
  }

  tx_timer_activate(&monotonic_checks);

  task_loop (DO_NOT_EXIT)
  {
    enum message msg;

    /* Thread suspension */
    st = tx_queue_receive(&shared, &msg, TX_WAIT_FOREVER);

    if (st != TX_SUCCESS) {
      continue;
    }

    decode_message(msg);
  }
}

/*
 * Creates a non-preemptive Recovery Task with defined parameters.
 */
void create_recovery_task(void)
{
  UINT st = tx_thread_create(&recovery_task,
                             "Recovery Task",
                             recovery_entry,
                             RECV_INPUT,
                             recovery_stack,
                             RECV_STACK_BYTES,
                             RECV_PRIORITY,
                             /* No preemption threshold */
                             RECV_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  const char *critical = "creation failure:";

  if (st != TX_SUCCESS) {
    log_die(id "task %s %u", critical, st);
  }

  st = tx_queue_create(&shared, "RECVQ", 1, &recvq_pool,
                                     sizeof recvq_pool);

  if (st != TX_SUCCESS) {
    log_die(id "queue %s %u", critical, st);
  }

  st = tx_timer_create(&monotonic_checks, "RECVT",
                       fc_timer_routine, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_NO_ACTIVATE);

  if (st != TX_SUCCESS) {
    log_die(id "timer %s %u", critical, st);
  }
}

/* ------ Recovery task ------ */
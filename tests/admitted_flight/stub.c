#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <setjmp.h>
#include "fc_recovery_owner.h"
#include "fc_sd_events.h"
/*ACTUAL_ENUM*/
#define option(x) ((x)&~Runtime_Configuration)
#define TX_TIMER_TICKS_PER_SECOND 1000
#define CO2_ASSERT_INTERVAL 500
#define REEF_ASSERT_INTERVAL 500
#define CO2_PIN 1U
#define REEF_PIN 2U
#define Acq memory_order_acquire
#define load(p,o) atomic_load_explicit((p),(o))
#define fetch_or(p,v,o) atomic_fetch_or_explicit((p),(v),(o))
#define Rel memory_order_release
#define TX_SUCCESS 0U
#define TX_SUSPENDED 1U
#define TX_RUNNING 2U
#define TX_QUEUE_SUSP 3U
#define TX_TERMINATED 4U
#define TX_COMPLETED 5U
#define TX_OR 1U
#define TO_ABORT 120U
#define STATE_HISTORY_MASK 7U
#define GPS_SUS_DELAYS 2U
#define GPS_SUS_MALFORM 2U
#define msmcode(code) ((fu32)((code)&~Sensor_Measm_Code))
#define id "RE "
typedef uint32_t fu32;
typedef int32_t fi32;
typedef unsigned UINT;
typedef uint32_t ULONG;
enum state {Startup,Postinit,Armed,Launch,Ascent,Coast,Apogee,Descent,Reefing,Landed,Recovery};
enum {G_Startup,IMU=0,Baro=1,Baro_Mask=1,Gyro_Mask=2,Accl_Mask=4,Wild_Mask=7};
static struct {enum state flight;unsigned global_state,confidence,kf_deviations,idx;} sm={.flight=Armed};
static enum state current(void){return sm.flight;}
static bool beyond(enum state s){return current()>s;}
static struct {unsigned rezero;} baro_conf;
static struct {unsigned gps_delayed,gps_malform,to_abort,failures,to_reinit;} smon;
static unsigned devctr_if[2];
static struct {atomic_uint lock;} meas_locks[2];
static struct task {UINT tx_thread_state;void *tx_thread_suspend_control_block;} evaluation_task={.tx_thread_state=TX_SUSPENDED},recovery_task,distribution_task;
static unsigned eval_stage,seds_syscall;
static volatile uint8_t evaluation_quiescent=1U,g_recovery_receive_loop_ready=1U;
static unsigned irq,seal,wakes,ascent_initializations,messages,waits,suspends,resumes,terminations,resets,commits;
static bool evaluator_context,store_ready=true,durable,g_fc_watchdog_resumed;
static jmp_buf scheduler;
atomic_uint_fast32_t g_conf;
static struct {uint32_t ODR;} gpio;
#define PYRO_PORT (&gpio)
static uint8_t board_recovery_manual_time_retired(uint64_t source){(void)source;return 0;}
static bool persistent_store_controls_ready(void){return store_ready;}
static bool persistent_store_publish_commission_complete(void){return store_ready;}
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t value){irq=value;}
static void __DMB(void){}
static void co2_high(void){assert(irq);gpio.ODR|=1;}
static void reef_high(void){assert(irq);gpio.ODR|=2;}
static void co2_low(void){gpio.ODR&=~1U;}
static void reef_low(void){gpio.ODR&=~2U;}
static void HAL_SuspendTick(void){}
static void HAL_IncTick(void){}
static uint32_t HAL_GetTick(void){return 100;}
static bool fc_watchdog_can_actuate(void){return seal==0;}
__attribute__((unused)) static bool fc_watchdog_can_actuate_manual(void){return seal==0;}
static bool fc_watchdog_record_deployment(bool reef,bool manual){(void)reef;(void)manual;return true;}
static bool fc_watchdog_hold_for_operator_reset(void){assert(irq);seal=1;return true;}
static bool fc_watchdog_begin_new_flight(void){assert(irq);seal=0;return true;}
static bool fc_watchdog_startup_reset_ready(void){return durable;}
int32_t flight_state_cache_reset_startup(void){assert(!irq);++commits;durable=true;return 0;}
static UINT tx_thread_suspend(struct task *t){++suspends;t->tx_thread_state=TX_SUSPENDED;if(evaluator_context)longjmp(scheduler,1);return TX_SUCCESS;}
static UINT tx_thread_resume(struct task *t){++resumes;t->tx_thread_state=TX_RUNNING;return TX_SUCCESS;}
static UINT tx_thread_terminate(struct task *t){++terminations;t->tx_thread_state=TX_TERMINATED;return TX_SUCCESS;}
static UINT tx_thread_reset(struct task *t){++resets;t->tx_thread_state=TX_SUSPENDED;return TX_SUCCESS;}
static UINT tx_thread_wait_abort(struct task *t){(void)t;++wakes;return TX_SUCCESS;}
static UINT tx_queue_flush(void *q){(void)q;return TX_SUCCESS;}
static UINT tx_event_flags_set(unsigned *group,unsigned mask,unsigned mode){(void)group;(void)mask;(void)mode;return TX_SUCCESS;}
static void tx_thread_sleep(unsigned ticks){(void)ticks;++waits;}
static void ascent_initialize(uint32_t conf){(void)conf;++ascent_initializations;assert(current()==Armed);}
static void message(const char *text,bool important){(void)text;(void)important;++messages;}

static void log_metric(const char *text,fi32 value,bool important){(void)text;(void)value;(void)important;}

static void sensor_init_supervised(uint32_t mask){(void)mask;assert(!"threshold should take abort handler");}

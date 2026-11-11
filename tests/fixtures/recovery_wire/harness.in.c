#include <stdlib.h>
#include "board_recovery_floor.c"
#include "main.h"
#include "tx_api.h"
#include "persistent_store.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
#include "board_recovery_transport.h"
#include "board_recovery_floor.h"
#include "board_recovery_wire.h"
#include "flight_state_cache.h"
#include "launchcore/storage.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint8_t disk[4096],response[40];
static bool runtime,fixture=true,fail_lock,fail_unlock,fail_read,owner_fail;static uint32_t fail_key;
static TX_MUTEX*physical;
static unsigned media_reads,media_writes,app_replies;
static uint64_t clock_ms=10000;
volatile uint32_t g_telemetry_discovery_seen;
struct RccFake test_rcc;struct PwrFake test_pwr;struct TampFake test_tamp;
void test_dsb(void){}
uint32_t HAL_GetTick(void){return 123;}
void*tx_thread_identify(void){return runtime?(void*)1:NULL;}
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT p){(void)n;(void)p;physical=m;m->locked=0;return fail_unlock?TX_MUTEX_ERROR:TX_SUCCESS;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG wait){(void)wait;if(fail_lock)return TX_MUTEX_ERROR;assert(!m->locked);m->locked=1;return TX_SUCCESS;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return fail_unlock?TX_MUTEX_ERROR:TX_SUCCESS;}
static const launchcore_storage_layout_t layout={.persistent_data_base=0,.persistent_data_size=4096,.persistent_data_erase_size=2048,.persistent_data_write_size=8};
static void check_owner(void){assert(fixture||!runtime||(physical&&physical->locked));}
static launchcore_storage_status_t init_media(void){return LAUNCHCORE_STORAGE_OK;}
static const launchcore_storage_layout_t*get_layout(void){return &layout;}
static launchcore_storage_status_t read_media(uint32_t a,void*p,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_reads++;if(fail_read){fail_read=false;return LAUNCHCORE_STORAGE_ERR_READ;}memcpy(p,disk+a,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t erase_media(uint32_t a,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_writes++;memset(disk+a,255,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t write_media(uint32_t a,const void*p,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_writes++;uint32_t key=0;if(n>=24)memcpy(&key,(const uint8_t*)p+4,4);if(key&&key==fail_key)return LAUNCHCORE_STORAGE_ERR_WRITE;for(uint32_t i=0;i<n;i++){assert((disk[a+i]&((const uint8_t*)p)[i])==((const uint8_t*)p)[i]);disk[a+i]=((const uint8_t*)p)[i];}return LAUNCHCORE_STORAGE_OK;}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=init_media,.read=read_media,.erase=erase_media,.write=write_media,.layout=get_layout};
static void sdk_unlocked(void){assert(!physical||!physical->locked);}
bool seds_router_sender_matches_hostname(const SedsRouter*r,const char*s,size_t n,const char*name,size_t len){sdk_unlocked();assert(r&&((len==2&&!memcmp(name,"GS",2))||(len==4&&!memcmp(name,"RRLS",4))));return s&&((n==len&&!memcmp(s,name,len))||(len==2&&n==10&&!memcmp(s,"@addr:1234",10)));}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType type,const void*p,size_t count,size_t width,SedsElemKind kind){sdk_unlocked();assert(r&&type==201&&count==40&&width==1&&kind==SEDS_EK_UNSIGNED);memcpy(response,p,40);assert((response[3]&32U)==EXPECTED_CAP);assert(response[3]&0x80U); /* RESET_ANY_STATE on every FC status */app_replies++;return SEDS_OK;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,SedsDataType type,bool read,bool write){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE&&read&&!write);return SEDS_OK;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,SedsDataType type,SedsResult(*cb)(const SedsPacketView*,void*),void*user){sdk_unlocked();(void)r;(void)cb;assert(type==SEDS_DT_FLIGHT_STATE&&!user);return SEDS_OK;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,SedsDataType type,bool*out){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE&&!out);return owner_fail?SEDS_IO:SEDS_OK;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,SedsDataType type){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE);return SEDS_OK;}

static void put64(uint8_t*p,uint64_t value){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(value>>(8*i));}

static uint64_t get64(const uint8_t*p){uint64_t v=0;for(unsigned i=0;i<8;i++)v|=(uint64_t)p[i]<<(8*i);return v;}
static void send(SedsRouter*r,SedsPacketView*p){unsigned n=app_replies;assert(board_recovery_handle(r,p,SEDS_EP_BOARD_RECOVERY,123)==0&&app_replies==n+1);}
static uint32_t response_epoch(void){uint32_t n;memcpy(&n,response+20,4);return n;}

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <assert.h>
#include <string.h>
#include "fc_recovery_owner.h"
#include "fc_sd_events.h"
#define CO2_ASSERT_INTERVAL 500
#define REEF_ASSERT_INTERVAL 500
#define Parachute_Deployed 1U
#define Parachute_Expanded 2U
#define CO2_Asserted 4U
#define REEF_Asserted 8U
#define CO2_PIN 1U
#define REEF_PIN 2U
#define option(x) (x)
atomic_uint_fast32_t g_conf;
static struct {uint32_t ODR;} gpio;
#define PYRO_PORT (&gpio)
static unsigned irq,highs,lows,commits,finals,wakes,arms,historical;
static bool quiet,ready=true,durable,io_error,stop_in_io,arm_ready,seal_error;
static struct{uint8_t flight;int global_state,confidence,kf_deviations;}sm={.flight=5};
#define current() sm.flight
static uint32_t seal;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t n){irq=n;}
/* This wire/owner fixture has no OTA job or telemetry reset worker. Its
 * actual recovery/pulse callers still hold their real modeled IRQ owner. */
static void ota_stream_retire_reset_locked(void){assert(irq);}
static bool ota_stream_take_reset_locked(uint32_t epoch){(void)epoch;assert(irq);return false;}
bool fc_recovery_ota_reset_boundary_ready(void){assert(irq);return false;}
static void NVIC_SystemReset(void){assert(!"wire fixture must never reset MCU");}

static void co2_high(void){assert(irq);gpio.ODR|=1;++highs;}
static void reef_high(void){assert(irq);gpio.ODR|=2;++highs;}
static void co2_low(void){gpio.ODR&=~1U;++lows;}
static void reef_low(void){gpio.ODR&=~2U;++lows;}
static void HAL_SuspendTick(void){}
static void HAL_IncTick(void){}
static bool fc_watchdog_can_actuate(void){return seal==0;}
__attribute__((unused)) static bool fc_watchdog_can_actuate_manual(void){return seal==0;}
static bool fc_watchdog_record_deployment(bool reef,bool manual){(void)manual;historical|=reef?2:1;return true;}
static bool fc_watchdog_hold_for_operator_reset(void){assert(irq);if(seal_error)return false;seal=1;return true;}
static bool fc_watchdog_begin_new_flight(void){assert(irq);seal=0;++arms;return true;}
bool fc_recovery_control_new_flight_ready(void){assert(irq);return arm_ready;}
void fc_recovery_control_new_flight(void){assert(irq);g_conf&=~(Parachute_Deployed|Parachute_Expanded);}

#include <stdio.h>
#define Startup 0
#define G_Startup 0
#define Eval_Abort_Flag 256U
#define Postinit_Requested 512U
#define Launch_Requested 1024U
#define In_Aborted_State 2048U
#define Graceful_Reset 4096U
#define Ascent_KF_Staged 8192U
#define Defer_Baro_Fallback 16384U
#define Using_Ascent_KF 32768U
typedef unsigned UINT;
#define TX_SUSPENDED 1
#define TX_COMPLETED 2
#define TX_TERMINATED 3
#define TX_QUEUE_SUSP 4
#define TX_OR 1
#define Baro_Mask 1
#define Gyro_Mask 2
#define Accl_Mask 4
#define IMU 0
#define Baro 1
#define Acq memory_order_acquire
#define load(p,o) atomic_load_explicit(p,o)
static struct{_Atomic int lock;}meas_locks[2];
static struct{int rezero;}baro_conf;
static uint8_t g_recovery_receive_loop_ready=1;
static struct Thread{int tx_thread_state;void*tx_thread_suspend_control_block;}evaluation_task,recovery_task;
static int seds_syscall,eval_stage,g_last_network_flight_command_msg;
#define Invalid_Message 255
static unsigned resumes,suspends,terminations,resets,signals,flushes;
static bool terminate_error,reset_error;
static void __DMB(void){}
static int tx_thread_suspend(struct Thread*t){++suspends;t->tx_thread_state=TX_SUSPENDED;return 0;}
static int tx_thread_resume(struct Thread*t){++resumes;t->tx_thread_state=0;return 0;}
static int tx_thread_terminate(struct Thread*t){++terminations;if(terminate_error)return 1;t->tx_thread_state=TX_TERMINATED;return 0;}
static int tx_thread_reset(struct Thread*t){++resets;if(reset_error)return 1;t->tx_thread_state=TX_SUSPENDED;return 0;}
static int tx_thread_wait_abort(struct Thread*t){(void)t;++wakes;return 0;}
static int tx_event_flags_set(int*p,int f,int op){(void)p;assert(f==7&&op==TX_OR);++signals;return 0;}
static int tx_queue_flush(int*p){assert(p==&seds_syscall);++flushes;return 0;}
static bool fc_watchdog_startup_reset_ready(void){return seal&&flight_state_cache_has_value()&&flight_state_cache_value()==0;}
@ACTUAL_FC_OWNER@

@ACTUAL_PULSE@
static volatile uint8_t evaluation_quiescent = 1U;
bool fc_recovery_control_ready(void) { return g_recovery_receive_loop_ready != 0U; }
bool fc_recovery_control_is_startup(void) { return current() == Startup; }
/* This transport/owner fixture does not authorize bench entry. */
bool fc_recovery_control_monitor_preflight(void) { return false; }
void fc_recovery_evaluation_running(void) { evaluation_quiescent = 0U; }
void fc_recovery_evaluation_quiescent(void)
{
  evaluation_quiescent = 1U;
  __DMB();
  (void)tx_thread_suspend(&evaluation_task);
}
void fc_recovery_control_wake(void)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  /* Do not abort a sensor-init sleep or any unrelated wait. */
  if (g_recovery_receive_loop_ready &&
      recovery_task.tx_thread_state == TX_QUEUE_SUSP &&
      recovery_task.tx_thread_suspend_control_block == &seds_syscall)
    (void)tx_thread_wait_abort(&recovery_task);
  __set_PRIMASK(mask);
}
bool fc_recovery_control_quiesce(void)
{
  const UINT state = evaluation_task.tx_thread_state;
  if (state == TX_SUSPENDED && !evaluation_quiescent) {
    /* A previously reset thread enters its barrier before flight-mode setup. */
    (void)tx_thread_resume(&evaluation_task);
    return false;
  }
  if (state != TX_COMPLETED && state != TX_TERMINATED &&
      !(state == TX_SUSPENDED && evaluation_quiescent)) {
    (void)tx_event_flags_set(&eval_stage, Baro_Mask | Gyro_Mask | Accl_Mask, TX_OR);
    return false;
  }
  if (load(&meas_locks[IMU].lock, Acq) || load(&meas_locks[Baro].lock, Acq))
    return false;
  if (state != TX_TERMINATED && tx_thread_terminate(&evaluation_task) != TX_SUCCESS)
    return false;
  if (tx_thread_reset(&evaluation_task) != TX_SUCCESS) return false;
  evaluation_quiescent = 1U;
  if (fc_recovery_holds_sequence()) {
    (void)tx_queue_flush(&seds_syscall);
#if defined(TELEMETRY_ENABLED) || defined(FAKESTATION)
    g_last_network_flight_command_msg = Invalid_Message;
#endif
  }
  return true;
}
void fc_recovery_control_mark_eval_stopped(void) { g_conf |= option(Eval_Abort_Flag); }
bool fc_recovery_control_finalize_startup(void)
{
  if (!evaluation_quiescent || evaluation_task.tx_thread_state != TX_SUSPENDED ||
      !fc_watchdog_startup_reset_ready()) return false;
  /* Preserve calibration/biases, samples, clocks, rates, recording shutdown,
   * failure thresholds and historical deployment bits until a NEW flight. */
  g_conf &= ~option(Postinit_Requested | Launch_Requested | In_Aborted_State |
                    Graceful_Reset | Ascent_KF_Staged | Defer_Baro_Fallback |
                    Eval_Abort_Flag);
  g_conf |= option(Using_Ascent_KF);
  sm.flight = Startup;
  sm.global_state = G_Startup;
  sm.confidence = 0;
  sm.kf_deviations = 0;
  baro_conf.rezero = 0;
  return true;
}


enum{Postinit=1,Armed=2,Launch=3,Ascent=4,Coast=5,Apogee=6,Descent=7,Reefing=8,Landed=9,Recovery=10};
enum{G_Armed=6,G_Launch=7,G_Ascent=8,G_Coast=9,G_Apogee=10,G_Descent=11,G_Reefing=12,G_Landed=13,G_Recovery=14,G_Aborted=15}; /* fcstructs.h numbering */
/* Pure mapping of the ACTUAL evaluator, not the independently managed cache.
 * No write to sm/global state or deployment GPIO is performed by QUERY. */
static uint8_t evaluator_global_state(void) {
 switch (current()) {
  case Startup: return G_Startup;
  case Postinit: case Armed: return G_Armed;
  case Launch: return G_Launch;
  case Ascent: return G_Ascent;
  case Coast: return G_Coast;
  case Apogee: return G_Apogee;
  case Descent: return G_Descent;
  case Reefing: return G_Reefing;
  case Landed: return G_Landed;
  case Recovery: return G_Recovery;
  default: return G_Aborted;
 }
}
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out)
{
 if (out == NULL) return -10;
 const uint32_t mask = __get_PRIMASK(); __disable_irq();
 *out = (BoardRecoverySnapshot){.actual_state=evaluator_global_state(),
     .flags=BOARD_RECOVERY_STATE_VALID,.abort_epoch=fc_recovery_epoch()};
 if (fc_recovery_control_ready()) {
  out->flags |= BOARD_RECOVERY_SUPPORTED | BOARD_RECOVERY_OUTPUTS_VALID;
  if (deployment_pulse_safe_outputs_locked(&out->actual_outputs_mask))
   out->flags |= BOARD_RECOVERY_SAFE_OUTPUTS;
 }
 if (fc_recovery_abort_latched() || (g_conf & option(In_Aborted_State)))
  out->flags |= BOARD_RECOVERY_ABORT_LATCHED;
 __set_PRIMASK(mask);
 return 0;
}

int32_t board_recovery_apply_safe(uint8_t desired,uint32_t expected,BoardRecoverySnapshot *out)
{
 const int32_t result = desired == 0U ? fc_recovery_request_startup(expected) : -10;
 (void)board_recovery_snapshot(out);
 return result;
}
#include "board_commissioning.h"
#include "commission_record.h"
ULONG tx_time_get(void){return 1;}
static bool first_service=true;
static void request(SedsRouter*r,SedsPacketView*p,bool service){
 unsigned before=app_replies;assert(board_recovery_handle(r,p,SEDS_EP_BOARD_RECOVERY,123)==0);
 if(service){
  if(first_service){first_service=false;unsigned writes=media_writes;board_commissioning_service();fc_recovery_control_service();board_commissioning_service();assert(media_writes==writes&&app_replies==before&&!board_recovery_commission_held(1)&&!gpio.ODR);fc_recovery_evaluation_quiescent();fc_recovery_control_service();assert(board_recovery_commission_held(1));}
  for(unsigned i=0;i<8&&(app_replies==before||response[1]==BOARD_RECOVERY_RESULT_PENDING);i++){board_commissioning_service();fc_recovery_control_service();fc_recovery_network_service();fc_recovery_control_service();}
 }
 assert(app_replies>=before+1);
}
int main(int argc,char**argv){assert(argc==2);(void)send;(void)response_epoch;(void)quiet;(void)ready;(void)durable;(void)io_error;(void)stop_in_io;(void)commits;(void)finals;sm.flight=Recovery;evaluation_task.tx_thread_state=0;fc_recovery_evaluation_running();
 bool virgin=!strcmp(argv[1],"virgin");bool healthy=strcmp(argv[1],"virgin")&&strcmp(argv[1],"floor");memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);launchcore_persist_strict_info_t info;uint64_t prior=41;uint8_t original=1;
 if(!virgin){assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);}
 if(healthy){uint64_t floor[2]={77,100};assert(launchcore_persist_set(0x4252464cU,floor,16)==0);}
 if(!strcmp(argv[1],"corrupt_boot"))disk[0]=0;
 if(!strcmp(argv[1],"deleted_identity"))assert(launchcore_persist_delete_strict(0x42524745U,&info)==0);
 if(!strcmp(argv[1],"deleted_floor"))assert(launchcore_persist_delete_strict(0x4252464cU,&info)==0);
 healthy=true; /* Normal virgin/ABSENT bookkeeping is initialized at boot. */
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();if(!strcmp(argv[1],"query_fault"))fail_read=true;(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();if(board_commissioning_pending())fc_recovery_restore_commissioning_hold();flight_state_cache_restore();if(!strcmp(argv[1],"corrupt_boot")||!strcmp(argv[1],"deleted_identity")||!strcmp(argv[1],"deleted_floor")){assert(!persistent_store_controls_ready());return 0;}assert(persistent_store_runtime_start()==0);runtime=true;if(!healthy||!strcmp(argv[1],"query_fault")){assert(!persistent_store_controls_ready());}else{assert(persistent_store_controls_ready());}
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;uint8_t body[32]={1,0};put64(body+4,1);put64(body+24,77);SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32,.timestamp=10000};
 if(healthy){
  if(strcmp(argv[1],"query_fault")){assert(generation&&generation!=UINT64_MAX);assert(board_recovery_floor_is_healthy());assert(board_recovery_command_time_allowed(10000));}

  if(!strcmp(argv[1],"query_fault")){unsigned reads=media_reads,writes=media_writes;request(r,&packet,false);assert(response[1]==7&&reads==media_reads&&writes==media_writes&&!persistent_store_controls_ready());puts("strict boot-prime fault, RAM-only QUERY IO, no mutation PASS");return 0;}
  unsigned rd=media_reads,wr=media_writes;request(r,&packet,false);assert(response[1]==0&&rd==media_reads&&wr==media_writes);
  body[1]=1;put64(body+4,2);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10001;
  /* Op 3 RESET_STARTUP_ANY: the RECOVER owner path, after a pyro check that never cuts a pulse. */
  if(!strncmp(argv[1],"reset_any",9))body[1]=3;
  if(!strcmp(argv[1],"reset_any_desired")){body[2]=1;request(r,&packet,false);assert(response[1]==6&&response_epoch()==1&&!highs);puts("op 3 with desired state 1 is INVALID PASS");return 0;}
  if(!strcmp(argv[1],"reset_any_unsafe")){
   deployment_pulse_start(false,true);assert(gpio.ODR==1&&highs==1);unsigned w=media_writes;
   request(r,&packet,false);assert(response[1]==5&&response_epoch()==1&&gpio.ODR==1&&highs==1&&media_writes==w);
   puts("op 3 during a pulse is UNSAFE: no seal, no cancel, no epoch PASS");return 0;}
  if(!strcmp(argv[1],"reset_any_ascent")){
   sm.flight=Ascent;
#if FC_RESET_ANY_HOLDS_UNDEPLOYED_FLIGHT
   unsigned w=media_writes,before=app_replies;assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK);assert(app_replies==before+1&&response[1]==5&&response_epoch()==1&&media_writes==w);puts("policy: op 3 refused in Ascent before any deploy PASS");return 0;
#endif
  }

  unsigned before_reads=media_reads,before_writes=media_writes,before_replies=app_replies;
  if(!strcmp(argv[1],"authority")){
   packet.sender="PB";assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==before_replies);packet.sender="@addr:1234";packet.sender_len=10;
  }
  if(!strcmp(argv[1],"unsupported_idle")){body[2]=1;request(r,&packet,false);assert(response[1]==6&&media_reads==before_reads&&media_writes==before_writes&&response_epoch()==1&&response[2]==G_Recovery&&(response[3]&18U)==18U);body[2]=0;}
  if(!strcmp(argv[1],"stale")){body[20]=2;request(r,&packet,false);assert(response[1]==4&&media_reads==before_reads&&media_writes==before_writes&&response_epoch()==1&&response[2]==G_Recovery);return 0;}
  if(!strcmp(argv[1],"io"))fail_key=0x4252464cU;
  if(!strcmp(argv[1],"pending")){unsigned replies=app_replies;assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==replies);assert(media_reads==before_reads&&media_writes==before_writes&&!persistent_store_controls_ready());BoardRecoverySnapshot held={0};assert(board_recovery_snapshot(&held)==0&&held.abort_epoch==1&&held.actual_state==G_Recovery&&(held.flags&1U)&&(held.flags&18U)==18U);assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==replies);}
  request(r,&packet,true);assert(!gpio.ODR&&!highs&&irq==0);
  if(!strcmp(argv[1],"io")){assert(response[1]==7&&response[3]&1U&&(response[3]&18U)==18U&&!persistent_store_controls_ready()&&response_epoch()==1&&response[2]==G_Recovery);unsigned reads=media_reads,writes=media_writes;fail_key=0;request(r,&packet,false);assert(response[1]==7&&reads==media_reads&&writes==media_writes);puts("actual NVM IO stays held, exact duplicate performs no IO PASS");return 0;}

  assert(response[1]==0&&response[2]==0&&response_epoch()==2&&!(response[3]&1U)&&(response[3]&18U)==18U&&persistent_store_controls_ready()&&!persistent_store_commission_pending());
  if(!strncmp(argv[1],"reset_any",9)){assert(!highs&&!gpio.ODR&&fc_recovery_requires_new_flight()); /* no relaunch from a cached state */puts("op 3 reset to Startup from any state PASS");return 0;}
  uint8_t marker[40];size_t size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());CommissionRecord record;assert(commission_record_decode(marker,size,&record)&&record.kind==1&&record.state==0&&record.generation==generation&&record.nonce==2&&record.source==10001);
  uint8_t original_request[32];memcpy(original_request,body,32);body[1]=0;put64(body+4,3);put64(body+12,0);memset(body+20,0,4);packet.timestamp=clock_ms=10002;unsigned query_reads=media_reads,query_writes=media_writes;request(r,&packet,false);assert(response[1]==0&&response[2]==0&&response_epoch()==2&&!(response[3]&1U)&&get64(response+12)==generation&&media_reads==query_reads&&media_writes==query_writes);memcpy(body,original_request,32);packet.timestamp=10001;
  wr=media_writes;request(r,&packet,false);assert(response[1]==0&&media_writes==wr);packet.timestamp=10002;request(r,&packet,false);assert(response[1]==6&&media_writes==wr);

  puts("actual FC normal one-request durable-owner COMPLETE PASS");return 0;

 }
 unsigned reads=media_reads,writes=media_writes;request(r,&packet,false);assert(reads==media_reads&&writes==media_writes&&response[1]==7&&!(response[3]&1U)&&(response[3]&18U)==18U);uint64_t expected=virgin?UINT64_MAX:generation;assert(get64(response+12)==expected);
 body[1]=1;put64(body+4,2);put64(body+12,expected);body[20]=1;packet.timestamp=clock_ms=10001;request(r,&packet,true);assert(response[1]==7&&response[3]&1U&&(response[3]&18U)==18U&&!persistent_store_controls_ready()&&persistent_store_commission_pending());generation=get64(response+12);assert(generation==(virgin?1:42));assert(flight_state_cache_has_value()==!virgin);if(!virgin)assert(flight_state_cache_value()==1);
 uint8_t marker[40];size_t size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());CommissionRecord history;assert(commission_record_decode(marker,size,&history)&&history.state==1&&history.generation==expected&&history.source==10001&&history.nonce==2&&history.issuer==77);
 writes=media_writes;body[1]=0;put64(body+4,3);put64(body+12,0);body[20]=0;packet.timestamp=10000;request(r,&packet,false);assert(response[1]==7&&media_writes==writes);body[1]=1;put64(body+4,4);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10002;request(r,&packet,false);assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&!persistent_store_controls_ready());
 body[1]=0;put64(body+4,5);put64(body+12,0);body[20]=0;packet.timestamp=clock_ms=10003;reads=media_reads;request(r,&packet,false);assert(response[1]==0&&response[3]&1U&&reads==media_reads&&writes==media_writes);
 body[1]=1;put64(body+4,6);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10004;request(r,&packet,true);assert(response[1]==0&&!(response[3]&1U)&&(response[3]&18U)==18U&&response[2]==0&&response_epoch()==2&&persistent_store_controls_ready()&&!persistent_store_commission_pending());
 size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());assert(commission_record_decode(marker,size,&history)&&history.state==0&&history.generation==expected&&history.source==10001);
 writes=media_writes;request(r,&packet,false);assert(response[1]==0&&media_writes==writes);packet.timestamp=10005;request(r,&packet,false);assert(response[1]==6&&media_writes==writes);
 assert(!gpio.ODR&&!highs&&fc_recovery_requires_new_flight());puts("actual FC wire->priority0owner/pulse/evaluatorhooks->realjournal->IO/freshsecondleg COMPLETE PASS");return 0;
}

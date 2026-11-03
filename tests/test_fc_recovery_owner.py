"""Actual bounded owner + actual pulse code, deterministic control/IO interleavings."""
from pathlib import Path
import subprocess,tempfile,unittest,os
ROOT=Path(os.environ.get("FC_OWNER_SOURCE_DIR",str(Path(__file__).resolve().parents[1])))
STUB=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <assert.h>
#include <string.h>
#include "fc_recovery_owner.h"
#include "fc_sd_events.h"
static uint8_t board_recovery_manual_time_retired(uint64_t source){(void)source;return 0;}
#define TX_TIMER_TICKS_PER_SECOND 1000
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
static bool store_ready=true;
static bool persistent_store_controls_ready(void){return store_ready;}
static bool persistent_store_publish_commission_complete(void){return store_ready;}
static uint8_t actual=5;
static uint32_t seal;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t n){irq=n;}
static void co2_high(void){assert(irq);gpio.ODR|=1;++highs;}
static void reef_high(void){assert(irq);gpio.ODR|=2;++highs;}
static void co2_low(void){gpio.ODR&=~1U;++lows;}
static void reef_low(void){gpio.ODR&=~2U;++lows;}
static void HAL_SuspendTick(void){}
static void HAL_IncTick(void){}
static uint32_t HAL_GetTick(void){return 100;}
static bool fc_watchdog_can_actuate(void){return seal==0;}
static bool fc_watchdog_record_deployment(bool reef){historical|=reef?2:1;return true;}
static bool fc_watchdog_hold_for_operator_reset(void){assert(irq);if(seal_error)return false;seal=1;return true;}
static bool fc_watchdog_begin_new_flight(void){assert(irq);seal=0;++arms;return true;}
bool fc_recovery_control_ready(void){return ready;}
bool fc_recovery_control_is_startup(void){return actual==0;}
void fc_recovery_control_wake(void){++wakes;}
bool fc_recovery_control_quiesce(void){assert(!irq);return quiet;}
void fc_recovery_control_mark_eval_stopped(void){}
bool fc_recovery_control_finalize_startup(void){assert(irq&&quiet&&durable);actual=0;++finals;return true;}
bool fc_recovery_control_new_flight_ready(void){assert(irq);return arm_ready;}
void fc_recovery_control_new_flight(void){assert(irq);g_conf&=~(Parachute_Deployed|Parachute_Expanded);}
int32_t flight_state_cache_reset_startup(void){assert(!irq);++commits;if(stop_in_io)fc_recovery_manual_stop();durable=!io_error;return io_error?-9:0;}
'''
MAIN=r'''
static void reset_complete(void){
 uint32_t epoch=fc_recovery_epoch();assert(fc_recovery_request_startup(epoch)==-14);
 fc_recovery_control_service();assert(commits==0&&finals==0); /* no quiescent ACK */
 fc_recovery_network_service();assert(commits==0);
 quiet=true;fc_recovery_control_service();assert(finals==0);
 fc_recovery_network_service();assert(commits==1&&finals==0);
 fc_recovery_control_service();assert(finals==1&&actual==0&&fc_recovery_epoch()==epoch+1);
 assert(fc_recovery_request_startup(epoch)==0&&commits==1&&finals==1);
}
int main(int argc,char**argv){assert(argc==2);
 if(!strcmp(argv[1],"store_unready")){
  store_ready=false;
  assert(fc_recovery_holds_sequence()&&fc_recovery_inhibits_pyro());
  deployment_pulse_start(false);deployment_pulse_start(true);assert(highs==0&&!gpio.ODR);
  assert(!fc_recovery_abort_latched()); /* Inhibit does not invent automatic Abort. */
  fc_recovery_manual_stop();assert(fc_recovery_abort_latched()&&seal&&!gpio.ODR);
 }else if(!strcmp(argv[1],"success")){
  deployment_pulse_start(false);assert(highs==1&&gpio.ODR==1);
  reset_complete();assert(!gpio.ODR&&!co2_ticks&&!reef_ticks);
  assert(fc_recovery_inhibits_pyro()&&seal&&historical==1);
  deployment_pulse_start(false);assert(highs==1); /* Startup never rearms */
  assert(!fc_recovery_admit_new_flight(true)&&arms==0);
  assert(!fc_recovery_admit_new_flight(false)&&arms==0);
  arm_ready=true;assert(fc_recovery_admit_new_flight(false)&&arms==1&&fc_recovery_epoch()==3);
  deployment_pulse_start(false);assert(highs==2);
  assert(fc_recovery_request_startup(1)==-10); /* old ticket crosses new flight */
 }else if(!strcmp(argv[1],"stop_before_high")){
  fc_recovery_manual_stop();deployment_pulse_start(false);deployment_pulse_start(true);
  assert(highs==0&&gpio.ODR==0&&seal&&fc_recovery_abort_latched());
 }else if(!strcmp(argv[1],"stop_after_high")){
  deployment_pulse_start(false);deployment_pulse_start(true);assert(highs==2);
  fc_recovery_manual_stop();assert(!gpio.ODR&&!co2_ticks&&!reef_ticks&&seal);
  unsigned stopped=lows;
  for(unsigned i=0;i<1000;i++)fc_runtime_tick();
  deployment_pulse_start(false);assert(highs==2&&lows==stopped);
 }else if(!strcmp(argv[1],"stop_during_io")){
  assert(fc_recovery_request_startup(1)==-14);quiet=true;fc_recovery_control_service();
  stop_in_io=true;fc_recovery_network_service();fc_recovery_control_service();
  assert(commits==1&&finals==0&&actual==5&&fc_recovery_epoch()==2);
  assert(fc_recovery_abort_latched()&&seal&&fc_recovery_request_startup(1)==-10);
 }else if(!strcmp(argv[1],"io_retry")){
  assert(fc_recovery_request_startup(1)==-14);quiet=true;fc_recovery_control_service();
  io_error=true;fc_recovery_network_service();fc_recovery_control_service();
  assert(fc_recovery_reset_failed()&&finals==0&&actual==5&&seal);
  io_error=false;assert(fc_recovery_request_startup(1)==-9&&commits==1);
  assert(fc_recovery_prepare_startup(1)); /* explicit new operator intent */
  assert(fc_recovery_request_startup(1)==-14);
  fc_recovery_control_service();fc_recovery_network_service();fc_recovery_control_service();
  assert(commits==2&&finals==1&&actual==0&&fc_recovery_epoch()==2);
 }else if(!strcmp(argv[1],"legacy_abort_cancels_reset")){
  assert(fc_recovery_request_startup(1)==-14);fc_recovery_request_eval_stop();
  quiet=true;fc_recovery_control_service();fc_recovery_network_service();
  assert(commits==0&&finals==0&&fc_recovery_epoch()==2&&seal);
 }else if(!strcmp(argv[1],"sensor_recovery")){
  assert(fc_recovery_request_sensor_restart());
  assert(fc_recovery_eval_pause_requested()&&!fc_recovery_inhibits_pyro()&&!seal);
  fc_recovery_control_service();assert(!fc_recovery_take_sensor_restart());
  quiet=true;fc_recovery_control_service();
  assert(fc_recovery_take_sensor_restart()&&!fc_recovery_take_sensor_restart());
  assert(!seal&&fc_recovery_epoch()==1&&commits==0&&arms==0);
 }else if(!strcmp(argv[1],"sensor_stop_race")){
  assert(fc_recovery_request_sensor_restart());fc_recovery_manual_stop();
  quiet=true;fc_recovery_control_service();
  assert(!fc_recovery_take_sensor_restart()&&!fc_recovery_admit_new_flight(true));
  assert(seal&&commits==0&&arms==0&&fc_recovery_epoch()==2);
 }else if(!strcmp(argv[1],"legacy_sensor_cancel")){
  assert(fc_recovery_request_sensor_restart());quiet=true;fc_recovery_control_service();
  fc_recovery_request_eval_stop();fc_recovery_control_service();
  assert(!fc_recovery_take_sensor_restart()&&!seal&&commits==0&&fc_recovery_epoch()==2);
 }else if(!strcmp(argv[1],"manual_twice")){
  fc_recovery_manual_stop();uint32_t ticket=fc_recovery_epoch();assert(ticket==2);
  fc_recovery_manual_stop();assert(fc_recovery_epoch()==3&&!fc_recovery_prepare_startup(ticket));
  assert(fc_recovery_request_startup(ticket)==-10&&!commits&&!gpio.ODR);
 }else if(!strncmp(argv[1],"stop_stage_",11)){
  fc_recovery_manual_stop();const uint32_t ticket=fc_recovery_epoch();
  assert(ticket==2&&fc_recovery_request_startup(ticket)==-14);
  if(strcmp(argv[1],"stop_stage_requested"))fc_recovery_control_service();
  if(!strcmp(argv[1],"stop_stage_network")||!strcmp(argv[1],"stop_stage_final")){
   quiet=true;fc_recovery_control_service();assert(owner.phase==RESET_NETWORK);
  }
  if(!strcmp(argv[1],"stop_stage_final")){fc_recovery_network_service();assert(owner.phase==RESET_FINAL);}
  fc_recovery_manual_stop();assert(fc_recovery_epoch()==ticket+1&&owner.phase==RESET_FAILED);
  unsigned before=commits;quiet=true;fc_recovery_control_service();fc_recovery_network_service();
  assert(commits==before&&!finals&&actual==5&&!gpio.ODR&&seal);
  assert(fc_recovery_request_startup(ticket)==-10);
 }else if(!strcmp(argv[1],"epoch_wrap")){
  owner.epoch=UINT32_MAX;assert(fc_recovery_request_startup(UINT32_MAX)==-10);
  fc_recovery_manual_stop();fc_recovery_manual_stop();assert(fc_recovery_epoch()==UINT32_MAX&&seal&&!gpio.ODR);
 }else if(!strcmp(argv[1],"manual_barrier_failure")){
  deployment_pulse_start(false);seal_error=true;fc_recovery_manual_stop();
  assert(!gpio.ODR&&fc_recovery_inhibits_pyro()&&fc_recovery_reset_failed()&&fc_recovery_abort_latched());
  quiet=true;fc_recovery_control_service();fc_recovery_network_service();assert(!commits&&!finals);
 }else if(!strcmp(argv[1],"barrier_failure")){
  deployment_pulse_start(false);seal_error=true;
  assert(!fc_recovery_prepare_startup(1));
  assert(!gpio.ODR&&fc_recovery_inhibits_pyro()&&fc_recovery_reset_failed());
  assert(fc_recovery_request_startup(1)==-9);
  quiet=true;fc_recovery_control_service();fc_recovery_network_service();
  assert(commits==0&&finals==0);
 }else if(!strcmp(argv[1],"masked_caller")){
  irq=1;fc_recovery_manual_stop();assert(irq==1&&seal&&!gpio.ODR);
 }else assert(!"unknown case");
 return 0;}
'''
class FCRecoveryOwnerTests(unittest.TestCase):
 def test_actual_owner_and_pulse_interleavings(self):
  owner=(ROOT/'Core/Src/fc_recovery_owner.c').read_text();pulse=(ROOT/'Core/Src/deployment_pulse.c').read_text()
  actual='\n'.join(l for l in (owner+'\n'+pulse).splitlines()if not l.startswith('#include'))
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);src=p/'test.c';src.write_text(STUB+actual+MAIN);exe=p/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),str(src),'-o',str(exe)],check=True)
   for case in ['store_unready','success','stop_before_high','stop_after_high','stop_during_io','io_retry','legacy_abort_cancels_reset','epoch_wrap','barrier_failure','manual_barrier_failure','masked_caller','sensor_recovery','sensor_stop_race','legacy_sensor_cancel','manual_twice','stop_stage_requested','stop_stage_wait','stop_stage_network','stop_stage_final']:
    with self.subTest(case=case):subprocess.run([str(exe),case],check=True)

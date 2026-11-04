"""Actual monitor/owner/pulse boundaries; old-source causal mode is read-only."""
from pathlib import Path
import importlib.util
import os
import re
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path(os.environ.get('FC_MONITOR_CAUSAL_SOURCE',str(ROOT)))
spec=importlib.util.spec_from_file_location('monitor_owner_fixture',ROOT/'tests/test_fc_recovery_owner.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)

class MonitorAdmissionTests(unittest.TestCase):
    def test_actual_owner_mailbox_and_pulse_boundaries(self):
        # Compile the candidate's actual RAM-only FSM helper in both controls.
        # The old owner never calls it; both runs see the same enum/flag inputs.
        recovery=(ROOT/'Core/Src/recovery.c').read_text()
        helper=recovery[recovery.index('bool fc_recovery_control_monitor_preflight(void)'):recovery.index('void fc_recovery_evaluation_running(void)')]
        states=(ROOT/'Core/Inc/fcstructs.h').read_text()
        enum=re.search(r'typedef enum flight_state : fu8 \{(.*?)\} state;',states,re.S).group(1)
        stub=fixture.STUB.replace('#define TX_TIMER_TICKS_PER_SECOND', '#define TELEMETRY_ENABLED\n#define FC_ACTUAL_MONITOR_PREFLIGHT\n#define TX_TIMER_TICKS_PER_SECOND',1)
        stub=stub.replace('static uint32_t HAL_GetTick(void){return 100;}','static uint32_t monitor_time=100; static uint32_t HAL_GetTick(void){return monitor_time;}')
        stub+='''
enum { '''+enum+''' };
#define current() actual
#define Launch_Requested (1U<<20)
#define Lost_GroundStation (1U<<21)
#define load(address,order) atomic_load(address)
static unsigned ota_retires,system_resets,estimator_initializations;
static void ota_stream_retire_reset_locked(void){assert(irq);++ota_retires;}
static bool ota_stream_take_reset_locked(uint32_t epoch){(void)epoch;return false;}
bool fc_recovery_ota_reset_boundary_ready(void){return false;}
static void NVIC_SystemReset(void){++system_resets;}
static void __DMB(void){}
#define Acq 0
#include <stdio.h>
'''+helper
        actual='\n'.join(line for file in ['fc_recovery_owner.c','deployment_pulse.c'] for line in (SOURCE/'Core/Src'/file).read_text().splitlines() if not line.startswith('#include'))
        monitor=(SOURCE/'Core/Src/fc_monitor_bench.c').read_text();monitor='\n'.join(line for line in monitor.splitlines() if not line.startswith('#include'))
        # Sensor/math plumbing is outside this control test; no fake success
        # qualifies numeric behavior. A rejected poll must never invoke it.
        sensors='''
#define MAX_DPS 2000
#define MAX_ACC 1000
#define MAX_ALT 100000
#define STATE_HISTORY_MASK 7
static struct {struct {float x,y,z;} gyro,accl;struct {float alt;} baro;} meas;
static struct {float alt,vel;} sv[8],imedsv;
static struct {unsigned idx;} sm;
static struct {float q0,rho1,rho2,rho3;} qv;
#define svec(k) sv[(sm.idx-(k))&7]
static bool kalman_monitor_initialize(void){++estimator_initializations;return true;}
static void ascent_predict(float dt,uint32_t conf){(void)dt;(void)conf;}
static void ascent_update(void){}
'''
        main=r'''
static void refused(void){
 const uint32_t epoch=owner.epoch,flags=atomic_load(&g_conf),before_seal=seal;
 const unsigned before_highs=highs,before_lows=lows,before_ota=ota_retires;
 const bool admitted=owner.admitted_flight;const uint32_t outputs=gpio.ODR;
 irq=1; /* Match the original production poll caller in the old control. */
 const bool result=fc_recovery_monitor_enter();assert(irq==1);irq=0;
 fprintf(stderr,"MONITOR_REFUSAL actual=%u admitted=%u result=%u output_before=%u output_after=%u low_before=%u low_after=%u ota_before=%u ota_after=%u epoch_before=%u epoch_after=%u\n",actual,admitted,result,outputs,gpio.ODR,before_lows,lows,before_ota,ota_retires,epoch,owner.epoch);
 assert(!result);
 assert(!irq&&!owner.monitor_mode&&!owner.monitor_ready);
 assert(owner.epoch==epoch&&owner.admitted_flight==admitted&&atomic_load(&g_conf)==flags&&seal==before_seal);
 assert(highs==before_highs&&lows==before_lows&&ota_retires==before_ota&&gpio.ODR==outputs);
 irq=1;assert(!fc_recovery_monitor_enter()&&irq==1);irq=0;
 /* The actual mailbox/poll must not turn rejection into lease ownership. */
 g_fc_monitor.command=0xa0000001U;fc_monitor_bench_poll();
 assert(!entered&&g_fc_monitor.status==4&&!owner.monitor_mode);
 fc_monitor_bench_sample(true,true,true);
 monitor_time+=2001;fc_monitor_bench_poll();
 assert(!entered&&!system_resets&&!estimator_initializations);
 g_fc_monitor.command=0;fc_monitor_bench_poll();
 assert(!entered&&!system_resets&&!estimator_initializations);
 assert(lows==before_lows&&ota_retires==before_ota&&gpio.ODR==outputs);
}
int main(int argc,char **argv){assert(argc==2);actual=Startup;
 if(!strcmp(argv[1],"accepted")){
  assert(fc_recovery_admit_new_flight(false));assert(owner.admitted_flight);
  deployment_pulse_start(false);assert(gpio.ODR==1);store_ready=false;
  refused();assert(fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"restored")){
  assert(fc_recovery_restore_admitted_flight());assert(owner.admitted_flight);
  actual=Ascent;deployment_pulse_start(true);assert(gpio.ODR==2);refused();
 }else if(!strcmp(argv[1],"current")){
  actual=Ascent;assert(!owner.admitted_flight);refused();
 }else if(!strcmp(argv[1],"requested")){
  actual=Armed;atomic_store(&g_conf,Launch_Requested);refused();
 }else if(!strcmp(argv[1],"lost_groundstation")){
  actual=Startup;atomic_store(&g_conf,Lost_GroundStation);refused();
 }else if(!strcmp(argv[1],"reset_active")){
  owner.phase=RESET_REQUESTED;refused();assert(owner.phase==RESET_REQUESTED);
 }else if(!strcmp(argv[1],"preflight")||!strcmp(argv[1],"preflight_armed")){
  if(!strcmp(argv[1],"preflight_armed"))actual=Armed;
  assert(fc_recovery_control_monitor_preflight());
  assert(fc_recovery_monitor_enter()&&!irq); /* New API preserves unmasked callers. */
  g_fc_monitor.command=0x80000001U;fc_monitor_bench_poll();
  assert(entered&&owner.monitor_mode&&!irq&&g_fc_monitor.status==3);
  assert(!fc_recovery_admit_new_flight(false)&&!fc_recovery_admit_new_flight(true));
  quiet=true;fc_recovery_control_service();assert(fc_recovery_monitor_ready());
  const uint32_t epoch=owner.epoch;fc_recovery_manual_stop();
  assert(owner.epoch==epoch+1&&owner.manual_stop&&seal&&fc_recovery_inhibits_pyro());
  assert(!system_resets);
 }else assert(!"unknown case");fprintf(stderr,"NATIVE_BOUNDARY %s PASS\n",argv[1]);return 0;}
'''
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);src=directory/'test.c';src.write_text(stub+actual+sensors+monitor+main);exe=directory/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-pie','-no-pie','-I',str(SOURCE/'Core/Inc'),str(src),'-o',str(exe)],check=True)
            cases=os.environ.get('FC_MONITOR_CAUSAL_CASES','accepted restored current requested lost_groundstation reset_active preflight preflight_armed').split()
            for case in cases:
                with self.subTest(case=case):subprocess.run([str(exe),case],check=True)

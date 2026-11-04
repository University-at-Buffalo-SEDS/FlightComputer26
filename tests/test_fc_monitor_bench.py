"""Exercise the production bench mailbox state machine without board hardware."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


class MonitorBenchTests(unittest.TestCase):
    def compile_run(self, source, flags=()):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'test.c';path.write_text(source)
            executable=Path(tmp)/'test'
            subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror',*flags,str(path),'-o',str(executable)],check=True)
            subprocess.run([str(executable)],check=True)

    def test_fresh_sensor_gates_start_stop_reset_lease_expiry_and_tick_wrap(self):
        source=(ROOT/'Core/Src/fc_monitor_bench.c').read_text()
        source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
        harness=r'''
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
enum { Startup=0, TX_SUSPENDED=3, MAX_DPS=2000, MAX_ACC=1000, MAX_ALT=100000, Acq=0, STATE_HISTORY_MASK=7 };
static uint32_t clock_ms, flight, init_calls, resets;
static bool ready;
static bool fc_recovery_monitor_enter(void) {return true;}
static bool fc_recovery_monitor_ready(void) {return ready;}
static void NVIC_SystemReset(void) {resets++;}
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(mask) ((void)(mask))
#define option(x) (x)
enum {Postinit_Requested=4, Launch_Requested=8};
typedef struct {float x,y,z;} xyz;
static struct {xyz gyro,accl;struct {float alt;} baro;} meas={.accl={0,0,10}};
typedef struct {float alt,vel;} vec;
static vec sv[8],imedsv={10,20};
static struct {unsigned idx;} sm;
static struct {float q0,rho1,rho2,rho3;} qv={1,0,0,0};
static uint32_t g_conf;
#define load(p,order) (*(p))
#define current() flight
#define svec(k) sv[(sm.idx-(k))&7]
#define __DMB() ((void)0)
static uint32_t HAL_GetTick(void) {return clock_ms;}
static bool kalman_monitor_initialize(void) {init_calls++;return true;}
static void ascent_predict(float dt,uint32_t conf) {(void)conf;assert(dt>0 && dt<=.1f);}
static void ascent_update(void) {svec(0)=imedsv;}
'''
        harness+=source+r'''
static void step(bool gyro,bool accel,bool baro) {fc_monitor_bench_poll();fc_monitor_bench_sample(gyro,accel,baro);}

int main(void) {
  clock_ms=100;
  flight=1;g_fc_monitor.command=0xa0000001U;
  step(true,true,true);assert(g_fc_monitor.status==3);
  assert(init_calls==0); /* Evaluator has not acknowledged its barrier. */
  ready=true;
  step(false,false,false);
  assert(g_fc_monitor.status==1 && init_calls==0);
  step(true,true,true);
  assert(init_calls==1 && g_fc_monitor.predictions==0);
  clock_ms=101;step(true,false,false);
  assert(g_fc_monitor.predictions==0);
  clock_ms=102;step(false,true,true);
  assert(g_fc_monitor.predictions==1 && g_fc_monitor.updates==1);
  assert(g_fc_monitor.status==2 && g_fc_monitor.altitude==10 && g_fc_monitor.velocity==20);
  assert(flight==1); /* Entry does not require Startup. */
  g_fc_monitor.command=0x80000002U;step(true,true,true);
  assert(g_fc_monitor.status==0 && resets==0);
  clock_ms=200;g_fc_monitor.command=0xe0000003U;
  step(true,true,true);
  assert(init_calls==2 && g_fc_monitor.predictions==0 && g_fc_monitor.updates==0);
  clock_ms=201;step(true,true,true);
  assert(g_fc_monitor.status==2 && g_fc_monitor.predictions==1);
  clock_ms=2201;step(true,true,true);
  assert(resets==1); /* Lost monitor lease resets the MCU. */
  clock_ms=0xfffffff0U;g_fc_monitor.command=0xa0000004U;
  step(true,true,true);
  clock_ms=0x10U;step(true,true,true);
  assert(g_fc_monitor.status==2); /* Tick wrap is not lease expiry. */
  g_fc_monitor.command=5;step(true,true,true);
  assert(resets==2); /* Explicit exit resets too. */
  g_fc_monitor.command=0x80000006U;step(false,false,false);
  clock_ms+=2001;step(false,false,false);
  assert(resets==3); /* A stopped estimator still has a monitor lease. */
  return 0;
}
'''
        self.compile_run(harness,('-ffast-math',))

    def test_runtime_mode_holds_sequence_and_actual_output_gate(self):
        owner=(ROOT/'Core/Src/fc_recovery_owner.c').read_text()
        start=owner.index('bool fc_recovery_holds_sequence(void)')
        end=owner.index('bool fc_recovery_requires_new_flight',start)
        gate=owner[start:end]
        harness='''#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
static struct {bool monitor_mode,monitor_ready,admitted_flight,commissioning_hold,manual_stop,reset_inhibit,verified_startup;} owner={.verified_startup=true};
static unsigned cancelled,irq;
static uint32_t __get_PRIMASK(void) {return irq;}
static void __disable_irq(void) {irq=1;}
static void __set_PRIMASK(uint32_t mask) {irq=mask;}
static bool reset_active(void) {return false;}
static bool fc_recovery_control_monitor_preflight(void) {return true;}
static bool persistent_store_controls_ready(void) {return true;}
static bool fc_watchdog_can_actuate(void) {return true;}
static void deployment_pulse_cancel_locked(void) {cancelled++;}
static void fc_recovery_ota_reset_retire_locked(void) {}
'''+gate+'''
int main(void) {
 assert(!fc_recovery_holds_sequence());
 owner.admitted_flight=true;assert(!fc_recovery_monitor_enter());
 assert(!fc_recovery_holds_sequence() && cancelled==0 && irq==0);
 owner.admitted_flight=false;assert(fc_recovery_monitor_enter());
 assert(fc_recovery_holds_sequence() && cancelled==1 && irq==0);
 assert(!fc_recovery_monitor_ready());
 assert(fc_recovery_monitor_enter() && cancelled==1);
 return 0;
}
'''
        self.compile_run(harness)
        # The GPIO-high path consults the same recovery hold under IRQ exclusion.
        pulse=(ROOT/'Core/Src/deployment_pulse.c').read_text()
        self.assertIn('!fc_recovery_inhibits_pyro() && fc_watchdog_can_actuate()',pulse)

"""Actual ThreadX owner hooks: no terminate until a lock-free safe-point."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
STUB=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdatomic.h>
#define TX_READY 0U
#define TX_SUSPENDED 1U
#define TX_COMPLETED 2U
#define TX_TERMINATED 3U
#define TX_QUEUE_SUSP 4U
#define TX_SLEEP 5U
#define TX_SUCCESS 0U
#define TX_OR 1U
#define IMU 0
#define Baro 1
#define Acq memory_order_acquire
#define Baro_Mask 1
#define Gyro_Mask 2
#define Accl_Mask 4
#define Invalid_Message UINT32_MAX
#define TELEMETRY_ENABLED 1
#define option(x) (x)
enum {Startup,Postinit,Armed,Launch};
enum {Eval_Abort_Flag=1U,Postinit_Requested=2U,Launch_Requested=4U,In_Aborted_State=8U,
Graceful_Reset=16U,Ascent_KF_Staged=32U,Defer_Baro_Fallback=64U,Using_Ascent_KF=128U,
Parachute_Deployed=256U,Parachute_Expanded=512U,CO2_Asserted=1024U,REEF_Asserted=2048U,
Manual_Biases=4096U,SD_Pipeline_Reset=8192U,G_Startup=0};
typedef unsigned UINT;
typedef struct {unsigned tx_thread_state;void *tx_thread_suspend_control_block;} TX_THREAD;
static TX_THREAD evaluation_task,recovery_task;
static int seds_syscall,eval_stage,other_queue;
static uint32_t g_recovery_receive_loop_ready,g_last_network_flight_command_msg;
static struct {atomic_uint lock;}meas_locks[2];
static struct {uint32_t flight,global_state;int confidence,kf_deviations;}sm;
static struct {int rezero;}baro_conf;
static uint32_t g_conf,clock_samples[3]={11,22,33};
static float calibration=123.5f;
static bool held=true,reset_ready=true;
static unsigned irq,events,resumes,terminates,resets,flushes,aborts;
#define load(p,o) atomic_load_explicit(p,o)
static uint32_t current(void){return sm.flight;}
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t n){irq=n;}
/* This extracted controller fixture omits its IRQ-owning caller. */
static unsigned ota_reset_retire_calls;
static void fc_recovery_ota_reset_retire_locked(void){++ota_reset_retire_calls;}
static void __DMB(void){}
static bool fc_recovery_holds_sequence(void){return held;}
static bool fc_watchdog_startup_reset_ready(void){return reset_ready;}
static UINT tx_thread_suspend(TX_THREAD*t){assert(t==&evaluation_task);t->tx_thread_state=TX_SUSPENDED;return 0;}
static UINT tx_thread_resume(TX_THREAD*t){assert(t==&evaluation_task);++resumes;t->tx_thread_state=TX_READY;return 0;}
static UINT tx_thread_terminate(TX_THREAD*t){assert(t==&evaluation_task);++terminates;t->tx_thread_state=TX_TERMINATED;return 0;}
static UINT tx_thread_reset(TX_THREAD*t){assert(t==&evaluation_task);++resets;t->tx_thread_state=TX_SUSPENDED;return 0;}
static UINT tx_event_flags_set(int*g,unsigned flags,unsigned mode){assert(g==&eval_stage&&flags==7&&mode==TX_OR);++events;return 0;}
static UINT tx_queue_flush(int*q){assert(q==&seds_syscall);++flushes;return 0;}
static UINT tx_thread_wait_abort(TX_THREAD*t){assert(irq&&t==&recovery_task);++aborts;return 0;}
'''
MAIN=r'''
int main(void){
 g_recovery_receive_loop_ready=1;evaluation_task.tx_thread_state=TX_READY;
 fc_recovery_evaluation_running();meas_locks[IMU].lock=1;
 assert(!fc_recovery_control_quiesce()&&events==1&&terminates==0);
 evaluation_task.tx_thread_state=TX_SUSPENDED;
 assert(!fc_recovery_control_quiesce()&&resumes==1&&terminates==0);
 fc_recovery_evaluation_quiescent();
 assert(!fc_recovery_control_quiesce()&&terminates==0); /* safe-point not enough while shared lock held */
 meas_locks[IMU].lock=0;
 assert(fc_recovery_control_quiesce()&&terminates==1&&resets==1&&flushes==1);
 assert(g_last_network_flight_command_msg==Invalid_Message);
 fc_recovery_control_mark_eval_stopped();assert(g_conf&Eval_Abort_Flag);
 g_conf|=Manual_Biases|SD_Pipeline_Reset|Parachute_Deployed|Parachute_Expanded|Launch_Requested;
 sm.flight=Launch;sm.global_state=3;baro_conf.rezero=1;
 reset_ready=false;assert(!fc_recovery_control_finalize_startup()&&sm.flight==Launch);
 reset_ready=true;assert(fc_recovery_control_finalize_startup()&&sm.flight==Startup);
 assert((g_conf&(Manual_Biases|SD_Pipeline_Reset|Parachute_Deployed|Parachute_Expanded))==(Manual_Biases|SD_Pipeline_Reset|Parachute_Deployed|Parachute_Expanded));
 assert(!(g_conf&(Eval_Abort_Flag|Launch_Requested))&&baro_conf.rezero==0);
 assert(ota_reset_retire_calls==1);
 assert(calibration==123.5f&&clock_samples[0]==11&&clock_samples[2]==33);
 sm.flight=Postinit;assert(!fc_recovery_control_new_flight_ready());
 sm.flight=Armed;g_conf|=Postinit_Requested;assert(!fc_recovery_control_new_flight_ready());
 g_conf&=~Postinit_Requested;assert(fc_recovery_control_new_flight_ready());
 fc_recovery_control_new_flight();assert(!(g_conf&(Parachute_Deployed|Parachute_Expanded)));
 recovery_task.tx_thread_state=TX_SLEEP;fc_recovery_control_wake();assert(aborts==0);
 recovery_task.tx_thread_state=TX_QUEUE_SUSP;recovery_task.tx_thread_suspend_control_block=&other_queue;
 fc_recovery_control_wake();assert(aborts==0);
 recovery_task.tx_thread_suspend_control_block=&seds_syscall;fc_recovery_control_wake();assert(aborts==1&&!irq);
 irq=1;fc_recovery_control_wake();assert(aborts==2&&irq==1);
 return 0;}
'''
class FCRecoveryControlTests(unittest.TestCase):
 def test_actual_control_owner_safe_point_and_preserved_context(self):
  source=(ROOT/'Core/Src/recovery.c').read_text();a=source.index('static volatile uint8_t evaluation_quiescent');b=source.index('/* Critical conditions */',a);actual=source[a:b]
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);f=p/'test.c';f.write_text(STUB+actual+MAIN);exe=p/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(f),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)

from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class DeploymentPulseTests(unittest.TestCase):
    def test_shutoff_without_hal_clock_or_command_task_progress(self):
        source = (ROOT / 'Core/Src/deployment_pulse.c').read_text()
        source = '\n'.join(x for x in source.splitlines() if not x.startswith('#include'))
        stub = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <assert.h>
#define TX_TIMER_TICKS_PER_SECOND 1000
#define CO2_ASSERT_INTERVAL 500
#define REEF_ASSERT_INTERVAL 500
#define Parachute_Deployed 1
#define Parachute_Expanded 2
#define CO2_Asserted 4
#define REEF_Asserted 8
#define option(x) (x)
atomic_uint_fast32_t g_conf;
static unsigned mask, co2, reef, hal_ticks, suspended, freeze_hal;
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __set_PRIMASK(uint32_t v) {mask=v;}
static void co2_high(void) {assert(mask); co2=1;}
static void reef_high(void) {assert(mask); reef=1;}
static void co2_low(void) {co2=0;}
static void reef_low(void) {reef=0;}
static void HAL_SuspendTick(void) {suspended++;}
static void HAL_IncTick(void) {if (!freeze_hal) hal_ticks++;}
'''
        main = r'''
int main(void) {
    assert(!co2 && !reef);
    fc_runtime_tick(); assert(suspended==1 && hal_ticks==1);
    deployment_pulse_start(false);
    assert(co2 && (g_conf & CO2_Asserted) && !mask);
    /* Neither command task nor TIM6 needs to run to finish the pulse. */
    freeze_hal=1;
    for (unsigned i=0;i<250;i++) fc_runtime_tick();
    deployment_pulse_start(false); /* retry cannot prolong pulse */
    deployment_pulse_start(true);
    for (unsigned i=0;i<249;i++) fc_runtime_tick();
    assert(co2 && reef);
    fc_runtime_tick(); assert(!co2 && reef && !(g_conf & CO2_Asserted));
    for (unsigned i=0;i<250;i++) fc_runtime_tick();
    assert(!reef && !(g_conf & REEF_Asserted));
    assert(g_deployment_pulse_completions==2 && hal_ticks==1 && suspended==1);
    assert((g_conf & (Parachute_Deployed|Parachute_Expanded))==3);
    freeze_hal=0; fc_runtime_tick(); assert(hal_ticks==2);
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d)/'pulse')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],input=stub+source+main,text=True,check=True)
            subprocess.run([exe],check=True)

    def test_pulse_armed_before_any_logging(self):
        s=(ROOT/'Core/Inc/fcapi.h').read_text()
        for name, call in [('release_parachute','deployment_pulse_start(false)'),('expand_parachute','deployment_pulse_start(true)')]:
            f=s[s.index('static inline bool '+name):]
            f=f[:f.index('\n}\n')]
            self.assertLess(f.index(call),f.index('approx altitude'))
        asm=(ROOT/'Core/Src/tx_initialize_low_level.S').read_text()
        self.assertEqual(asm.count('BL      fc_runtime_tick'),3)

    def test_can_wait_expires_when_hal_clock_stops(self):
        s=(ROOT/'Core/Src/can_bus.c').read_text()
        f=s[s.index('static HAL_StatusTypeDef can_bus_enqueue_tx_frame'):s.index('static inline void can_bus_notify_rx')]
        stub=r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef int HAL_StatusTypeDef;
typedef int FDCAN_TxHeaderTypeDef;
enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
#define HAL_FDCAN_ERROR_FIFO_FULL 1
#define CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U
static struct { unsigned State; } handle, *g_hfdcan=&handle;
static struct { uint32_t CYCCNT; } cycles;
#define DWT (&cycles)
static uint32_t SystemCoreClock=1000000U;
static unsigned g_can_tx_service_stage,g_fdcan_last_error,g_fdcan_last_state,g_fdcan_tx_fail_count,g_fdcan_tx_ok_count,slots,polls,accepted;
static uint32_t HAL_GetTick(void) {return 1;} /* deliberately frozen */
static int HAL_FDCAN_GetTxFifoFreeLevel(void *p) {(void)p;polls++;cycles.CYCCNT+=1000;assert(polls<10);return slots;}
static int can_bus_recover_if_bus_off(void) {return HAL_OK;}
static int HAL_FDCAN_AddMessageToTxFifoQ(void *p,const int *h,const uint8_t *d) {(void)p;(void)h;(void)d;accepted++;return HAL_OK;}
static unsigned HAL_FDCAN_GetError(void *p) {(void)p;return 0;}
'''
        main=r'''
int main(void) {
 int hdr=0;uint8_t d[64]={0};cycles.CYCCNT=UINT32_MAX-2000;
 assert(can_bus_enqueue_tx_frame(&hdr,d)==HAL_BUSY);
 assert(accepted==0 && polls==5 && g_fdcan_tx_fail_count==1);
 slots=1;assert(can_bus_enqueue_tx_frame(&hdr,d)==HAL_OK && accepted==1);
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe=str(Path(d)/'can')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-x','c','-','-o',exe],input=stub+f+main,text=True,check=True)
            subprocess.run([exe],check=True,timeout=5)

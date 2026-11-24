"""The FC tasks block when idle (so ThreadX reaches WFI) without losing work."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(test, source, *flags):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / 'test.c').write_text(source)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                        '-Werror', '-Wno-unused-function', *flags,
                        str(path / 'test.c'), '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)


def loop_body(source, entry):
    start = source.index(entry)
    loop = source.index('MrAnalog (WE_ARE_SO_BACK)', start)
    end = source.index('\nUINT create_', loop)
    return source[loop:end]


class IdleWaits(unittest.TestCase):
    def test_service_loops_block_instead_of_yielding(self):
        dma = loop_body((ROOT / 'Core/Src/dma.c').read_text(), 'void dma_entry')
        self.assertNotIn('tx_thread_relinquish', dma)
        self.assertEqual(dma.count('dma_wait_ready('), 2)
        telemetry = loop_body((ROOT / 'Core/Src/telemetry.c').read_text(),
                              'void telemetry_entry')
        self.assertNotIn('tx_thread_relinquish', telemetry)
        self.assertIn('telemetry_idle_wait();', telemetry)
        distribution = (ROOT / 'Core/Src/distribution.c').read_text()
        for function in ('static inline void for_ascent_update',
                         'static inline void descent_full_cycle',
                         'static inline void data_streaming_mode',
                         'static inline void post_initialization'):
            with self.subTest(function=function):
                start = distribution.index(function)
                body = distribution[start:distribution.index('\n}\n', start)]
                self.assertIn('dma_wait_for_samples(', body)
                self.assertNotIn('tx_thread_relinquish', body)
        predict = distribution[distribution.index('static inline void for_ascent_predict'):]
        predict = predict[:predict.index('\n}\n')]
        self.assertIn('dma_wait_for_samples(DMA_SENSOR_MASKS);', predict)

    def test_telemetry_wakes_on_can_rx_and_foreign_router_use(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        idle = telemetry[telemetry.index('static UNUSED_FUNCTION void telemetry_idle_wait'):]
        idle = idle[:idle.index('\n}\n')]
        # Pending CAN RX or router backlog never sleeps; otherwise one tick at most.
        self.assertLess(idle.index('can_bus_rx_pending()'), idle.index('tx_event_flags_get('))
        self.assertLess(idle.index('seds_router_has_queued_work('), idle.index('tx_event_flags_get('))
        self.assertIn('#define TELEMETRY_IDLE_TICKS 1U', telemetry)
        can = (ROOT / 'Core/Src/can_bus.c').read_text()
        for fifo in ('0', '1'):
            callback = can[can.index(f'void HAL_FDCAN_RxFifo{fifo}Callback'):]
            callback = callback[:callback.index('\n}\n')]
            self.assertLess(callback.index('can_bus_drain_rx_fifo('),
                            callback.index('can_bus_rx_ready_isr();'))
        hooks = (ROOT / 'Core/Src/fchooks.c').read_text()
        unlock = hooks[hooks.index('void telemetry_unlock(void)'):]
        unlock = unlock[:unlock.index('\n}\n')]
        self.assertLess(unlock.index('fchook_unlock('), unlock.index('telemetry_wake();'))

    def test_distribution_wakes_on_commands_gps_and_evaluator_stage(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        self.assertRegex(recovery, r'(?s)decode_flight_message\(\(fc_msg\)msg\.cmd\);\s*thread_comm_estimator_leave\(\);\s*/\*.*?\*/\s*'
                                   r'distribution_wake\(\);\s*telemetry_wake\(\);')
        distribution = (ROOT / 'Core/Src/distribution.c').read_text()
        gps = distribution[distribution.index('process_gps_packet(const uint8_t *data'):]
        gps = gps[:gps.index('\n}\n')]
        self.assertLess(gps.index('rfboard.updated = true;'), gps.index('distribution_wake();'))
        evaluation = (ROOT / 'Core/Src/evaluation.c').read_text()
        self.assertRegex(evaluation, r'~option\(Ascent_KF_Staged\), AcqRel\);\s*distribution_wake\(\);')

    def test_sample_waits_never_lose_a_wakeup(self):
        source = (ROOT / 'Core/Src/dma.c').read_text()
        waits = source[source.index('void HAL_GPIO_EXTI_Rising_Callback'):
                       source.index('/* DMA routine */')]
        propagate = source[source.index('static inline void propagate_rx'):
                           source.index('static inline void start_dma_transfer')]
        defines = ''.join(re.findall(r'#define (?:SENSOR_IDLE_POLL_TICKS|DIST_WAKE_OTHER) [^\n]*\n', source))
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef unsigned long ULONG; typedef unsigned UINT; typedef uint16_t fu16;
typedef struct { ULONG bits; } TX_EVENT_FLAGS_GROUP;
enum { TX_OR=0, TX_OR_CLEAR=1, TX_SUCCESS=0, TX_NO_EVENTS=7, Rel=0, Acq=0,
       Sensors=3, SENSOR_BUF_SIZE=8 };
enum { DMA_BARO_MASK=1, DMA_GYRO_MASK=6, DMA_ACCL_MASK=24 };
#define SENSOR_PROBE(x) ((void)0)
#define fc_lock(x) ((void)(x))
#define fc_unlock(x) ((void)(x))
static struct { fu16 drdy, relv; } flags;
static struct { int next, valid; } select;
static const struct { fu16 drdy[3]; int offset[3]; } gpio = {{1,6,24},{0,0,0}};
static uint8_t dmarx[SENSOR_BUF_SIZE], taskrx[3][SENSOR_BUF_SIZE - 2];
static int dma_locks[3];
static unsigned slept, last_timeout; static ULONG last_request;
static void (*during_wait)(void);
static fu16 fetch_or(fu16 *p, fu16 m, int o) { (void)o; fu16 v = *p; *p |= m; return v; }
static fu16 load(fu16 *p, int o) { (void)o; return *p; }
static UINT tx_event_flags_set(TX_EVENT_FLAGS_GROUP *g, ULONG bits, UINT op) {
  (void)op; g->bits |= bits; return TX_SUCCESS; }
static UINT tx_event_flags_get(TX_EVENT_FLAGS_GROUP *g, ULONG want, UINT op,
                               ULONG *got, ULONG timeout) {
  assert(op == TX_OR_CLEAR);
  last_request = want; last_timeout = (unsigned)timeout;
  if (during_wait) { void (*f)(void) = during_wait; during_wait = NULL; f(); }
  if (g->bits & want) { *got = g->bits & want; g->bits &= ~want; return TX_SUCCESS; }
  ++slept; return TX_NO_EVENTS;  /* blocked for the timeout */
}
static TX_EVENT_FLAGS_GROUP dma_ready, sample_ready;
''' + defines + waits + propagate + r'''
static void irq_baro(void) { HAL_GPIO_EXTI_Rising_Callback(DMA_BARO_MASK); }
static void deliver_baro(void) { select.next = 0; propagate_rx(); }
int main(void) {
  /* EXTI raises the flag and then the event, so a DMA task that read no
   * flag still wakes at once. */
  HAL_GPIO_EXTI_Rising_Callback(DMA_GYRO_MASK);
  assert(flags.drdy == DMA_GYRO_MASK && dma_ready.bits == DMA_GYRO_MASK);
  dma_ready.bits = 0; flags.drdy = 0;
  during_wait = irq_baro; dma_wait_ready(DMA_BARO_MASK);
  assert(slept == 0 && last_timeout == SENSOR_IDLE_POLL_TICKS);
  /* Idle: block for at most one tick. */
  dma_wait_ready(DMA_BARO_MASK); assert(slept == 1);

  /* A sample already pending in relv never sleeps, even with its event
   * consumed by an earlier wait. */
  slept = 0; flags.relv = DMA_BARO_MASK; sample_ready.bits = 0; last_request = 0;
  dma_wait_for_samples(DMA_BARO_MASK); assert(slept == 0 && last_request == 0);
  /* Pending samples the caller cannot consume do not keep it awake. */
  dma_wait_for_samples(DMA_GYRO_MASK); assert(slept == 1);
  assert(last_request == (DMA_GYRO_MASK | DIST_WAKE_OTHER));
  /* A sample published between the relv check and the wait is kept. */
  slept = 0; flags.relv = 0; during_wait = deliver_baro;
  dma_wait_for_samples(DMA_BARO_MASK); assert(slept == 0);
  assert(flags.relv == DMA_BARO_MASK);
  /* distribution_wake() ends any sample wait. */
  flags.relv = 0; distribution_wake();
  dma_wait_for_samples(DMA_SENSOR_MASKS_TEST); assert(slept == 0);
  dma_wait_for_samples(DMA_SENSOR_MASKS_TEST); assert(slept == 1);
  return 0;
}
'''
        harness = harness.replace('DMA_SENSOR_MASKS_TEST',
                                  '(DMA_BARO_MASK | DMA_GYRO_MASK | DMA_ACCL_MASK)')
        compile_and_run(self, harness, '-Wno-unused-parameter')


if __name__ == '__main__':
    unittest.main()

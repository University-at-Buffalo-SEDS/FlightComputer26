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
    loop = source.index('MrAnalog (WE_ARE_SO_BACK)', start) if 'MrAnalog (WE_ARE_SO_BACK)' in source[start:] else source.index('for (;;)', start)
    end = source.index('\nUINT create_', loop)
    return source[loop:end]


class IdleWaits(unittest.TestCase):
    def test_service_loops_block_instead_of_yielding(self):
        dma = (ROOT / 'Core/Src/dma.c').read_text()
        self.assertNotIn('tx_thread_relinquish', dma)
        self.assertNotIn('tx_thread_create', dma)
        telemetry = loop_body((ROOT / 'Core/Src/telemetry.c').read_text(),
                              'void telemetry_entry')
        self.assertNotIn('tx_thread_relinquish', telemetry)
        self.assertIn('telemetry_idle_wait();', telemetry)
        distribution = (ROOT / 'Core/Src/distribution.c').read_text()
        for function in ('static inline void for_ascent_update',
                         'static inline void descent_publish',
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
        exti = source[source.index('void HAL_GPIO_EXTI_Rising_Callback'):
                      source.index('/* DMA routine */')]
        propagate = source[source.index('static inline void propagate_rx'):
                           source.index('static inline void start_dma_transfer')]
        acquire = source[source.index('static void acquire_ready_samples(void)'):
                         source.index('/* Idle wait */')]
        waits = source[source.index('void dma_wait_for_samples(fu16 wanted)'):
                       source.index('/* Created before the distribution thread starts')]
        defines = ''.join(re.findall(r'#define (?:SENSOR_IDLE_POLL_TICKS|DIST_WAKE_OTHER) [^\n]*\n', source))
        harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef unsigned long ULONG; typedef unsigned UINT; typedef uint16_t fu16; typedef uint8_t fu8;
typedef struct { ULONG bits; } TX_EVENT_FLAGS_GROUP;
enum { TX_OR=0, TX_OR_CLEAR=1, TX_SUCCESS=0, TX_NO_EVENTS=7, Rel=0, Acq=0, Rlx=0,
       Sensors=3, SENSOR_BUF_SIZE=8, Sensor_Baro=0, Sensor_Gyro=1, Sensor_Accl=2,
       BARO_EXTI_GPIO_Port=0, BARO_EXTI_Pin=0, GPIO_PIN_SET=1, Using_Ascent_KF=1 };
enum { DMA_BARO_MASK=1, DMA_GYRO_MASK=6, DMA_ACCL_MASK=24 };
#define DMA_SENSOR_MASKS (DMA_BARO_MASK | DMA_GYRO_MASK | DMA_ACCL_MASK)
#define option(x) (x)
#define SENSOR_PROBE(x) ((void)0)
static struct { fu16 drdy, relv; } flags;
static struct { int next, valid; } select;
static const struct { fu16 drdy[3]; int offset[3]; } gpio = {{1,6,24},{0,0,0}};
static uint8_t dmarx[SENSOR_BUF_SIZE], taskrx[3][SENSOR_BUF_SIZE - 2];
static unsigned g_conf = Using_Ascent_KF, slept, last_timeout, reads[3], baro_pin;
static ULONG last_request;
static bool baro_level_seen;
static void (*during_wait)(void);
static fu16 fetch_or(fu16 *p, fu16 m, int o) { (void)o; fu16 v = *p; *p |= m; return v; }
static fu16 fetch_and(fu16 *p, fu16 m, int o) { (void)o; fu16 v = *p; *p &= m; return v; }
#define load(p, o) (*(p))
static int HAL_GPIO_ReadPin(int port, int pin) { (void)port; (void)pin; return (int)baro_pin; }
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
static TX_EVENT_FLAGS_GROUP sensor_events;
""" + defines + exti + propagate + r"""
/* One SPI transfer: consumes the ready flag and delivers the bytes. */
static void start_dma_transfer(void) {
  flags.drdy &= ~gpio.drdy[select.next]; ++reads[select.next]; propagate_rx(); }
""" + acquire + waits + r"""
static void irq_baro(void) { HAL_GPIO_EXTI_Rising_Callback(DMA_BARO_MASK); }
int main(void) {
  /* EXTI raises the flag and then the event, so a thread that read no flag
   * still wakes at once. */
  HAL_GPIO_EXTI_Rising_Callback(DMA_GYRO_MASK);
  assert(flags.drdy == DMA_GYRO_MASK && sensor_events.bits == DMA_GYRO_MASK);
  /* A ready sensor is read before deciding to wait: no sleep, and the
   * consumed data-ready event does not wake the next wait. */
  dma_wait_for_samples(DMA_GYRO_MASK);
  assert(slept == 0 && reads[Sensor_Gyro] == 1 && flags.relv == DMA_GYRO_MASK);
  /* A sample already pending in relv never sleeps. */
  dma_wait_for_samples(DMA_GYRO_MASK); assert(slept == 0 && reads[Sensor_Gyro] == 1);
  /* Pending samples the caller cannot consume do not keep it awake; idle
   * blocks for at most one tick on data-ready or a wake. */
  sensor_events.bits = 0;
  dma_wait_for_samples(DMA_BARO_MASK); assert(slept == 1 && last_timeout == SENSOR_IDLE_POLL_TICKS);
  assert(last_request == (DMA_SENSOR_MASKS | DIST_WAKE_OTHER));
  /* A data-ready interrupt during the wait is read before returning. */
  slept = 0; during_wait = irq_baro;
  dma_wait_for_samples(DMA_BARO_MASK);
  assert(slept == 0 && reads[Sensor_Baro] == 1 && (flags.relv & DMA_BARO_MASK));
  /* distribution_wake() ends any sample wait. */
  flags.relv = 0; distribution_wake();
  dma_wait_for_samples(DMA_SENSOR_MASKS); assert(slept == 0);
  dma_wait_for_samples(DMA_SENSOR_MASKS); assert(slept == 1);
  /* Every ready sensor is read in one pass, gyro first. */
  flags.drdy = DMA_SENSOR_MASKS; reads[0] = reads[1] = reads[2] = 0;
  dma_wait_for_samples(DMA_BARO_MASK);
  assert(reads[0] == 1 && reads[1] == 1 && reads[2] == 1 && flags.drdy == 0);
  /* Descent (no ascent filter): only the barometer is sampled. */
  g_conf = 0; flags.relv = 0; flags.drdy = DMA_SENSOR_MASKS; reads[0] = reads[1] = reads[2] = 0;
  dma_wait_for_samples(DMA_BARO_MASK);
  assert(reads[Sensor_Baro] == 1 && !reads[Sensor_Gyro] && !reads[Sensor_Accl]);
  assert(flags.drdy == (DMA_GYRO_MASK | DMA_ACCL_MASK));
  return 0;
}
"""
        compile_and_run(self, harness, '-Wno-unused-parameter')

if __name__ == '__main__':
    unittest.main()

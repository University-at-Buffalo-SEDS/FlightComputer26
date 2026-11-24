"""thread_comm.c on the host, against a recording ThreadX stub."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

TX_API = r'''
#ifndef TX_API_H
#define TX_API_H
typedef unsigned int UINT;
typedef unsigned int ULONG; /* 32-bit, as on the MCU */
typedef char CHAR;
typedef void VOID;
#define TX_SUCCESS 0U
#define TX_NOT_OWNED 0x1EU
#define TX_INHERIT 1U
#define TX_WAIT_FOREVER 0xFFFFFFFFU
#define TX_NO_WAIT 0U
#define TX_QUEUE_FULL 0x0BU
#define TX_QUEUE_EMPTY 0x0AU
#define TX_QUEUE_SUSP 5U
#define TX_READY 0U
typedef struct TX_THREAD { int id; UINT tx_thread_state; void *tx_thread_suspend_control_block; } TX_THREAD;
typedef struct TX_QUEUE { UINT words; ULONG *start; ULONG size; UINT head, count, capacity; int created; } TX_QUEUE;
UINT tx_queue_create(TX_QUEUE *q, CHAR *name, UINT words, VOID *start, ULONG size);
UINT tx_queue_send(TX_QUEUE *q, VOID *src, ULONG wait);
UINT tx_queue_receive(TX_QUEUE *q, VOID *dst, ULONG wait);
UINT tx_queue_flush(TX_QUEUE *q);
typedef struct TX_MUTEX { TX_THREAD *tx_mutex_owner; UINT tx_mutex_ownership_count; UINT inherit; int created; } TX_MUTEX;
UINT tx_mutex_create(TX_MUTEX *m, CHAR *name, UINT inherit);
UINT tx_mutex_get(TX_MUTEX *m, ULONG wait);
UINT tx_mutex_put(TX_MUTEX *m);
TX_THREAD *tx_thread_identify(void);
UINT tx_thread_sleep(ULONG ticks);
#endif
'''

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "thread_comm.h"
extern volatile uint32_t g_last_network_flight_command_msg;
static TX_THREAD evaluator = {1}, recovery = {2};
static TX_THREAD *running = &evaluator;
static TX_MUTEX *guard;
static unsigned creates, sleeps, held_during_sleep;
static ULONG slept;
UINT tx_mutex_create(TX_MUTEX *m, CHAR *n, UINT inherit)
{ (void)n; ++creates; m->created = 1; m->inherit = inherit; m->tx_mutex_owner = 0; m->tx_mutex_ownership_count = 0; guard = m; return TX_SUCCESS; }
UINT tx_mutex_get(TX_MUTEX *m, ULONG w)
{
  assert(m->created && w == TX_WAIT_FOREVER);
  /* The stub cannot block: another owner here would be a deadlock. */
  assert(m->tx_mutex_owner == 0 || m->tx_mutex_owner == running);
  m->tx_mutex_owner = running; ++m->tx_mutex_ownership_count; return TX_SUCCESS;
}
UINT tx_mutex_put(TX_MUTEX *m)
{
  if (m->tx_mutex_owner != running || m->tx_mutex_ownership_count == 0) return TX_NOT_OWNED;
  if (--m->tx_mutex_ownership_count == 0) m->tx_mutex_owner = 0;
  return TX_SUCCESS;
}
TX_THREAD *tx_thread_identify(void) { return running; }
/* A FIFO of `words`-ULONG messages, as ThreadX's. */
static TX_QUEUE *queue; static ULONG last_send_wait;
UINT tx_queue_create(TX_QUEUE *q, CHAR *n, UINT words, VOID *start, ULONG size)
{ (void)n; q->words = words; q->start = start; q->size = size; q->capacity = size / (words * sizeof(ULONG));
  q->head = q->count = 0; q->created = 1; queue = q; return TX_SUCCESS; }
UINT tx_queue_send(TX_QUEUE *q, VOID *src, ULONG w)
{
  assert(q->created); last_send_wait = w;
  if (q->count == q->capacity) return TX_QUEUE_FULL;
  memcpy(q->start + ((q->head + q->count) % q->capacity) * q->words, src, q->words * sizeof(ULONG));
  ++q->count; return TX_SUCCESS;
}
UINT tx_queue_receive(TX_QUEUE *q, VOID *dst, ULONG w)
{
  (void)w; if (!q->count) return TX_QUEUE_EMPTY;
  memcpy(dst, q->start + q->head * q->words, q->words * sizeof(ULONG));
  q->head = (q->head + 1) % q->capacity; --q->count; return TX_SUCCESS;
}
UINT tx_queue_flush(TX_QUEUE *q) { q->head = q->count = 0; return TX_SUCCESS; }
UINT tx_thread_sleep(ULONG t)
{
  ++sleeps; slept += t;
  if (guard->tx_mutex_owner) ++held_during_sleep;
  /* While the evaluator sleeps, recovery can run a command under the guard. */
  TX_THREAD *was = running; running = &recovery;
  thread_comm_estimator_enter(); assert(thread_comm_estimator_held()); thread_comm_estimator_leave();
  running = was;
  return TX_SUCCESS;
}
int main(void)
{
  assert(!thread_comm_estimator_held());
  assert(thread_comm_init() == TX_SUCCESS);
  assert(thread_comm_init() == TX_SUCCESS && creates == 1);
  assert(guard->inherit == TX_INHERIT);

  /* A pass: held by its owner only. */
  thread_comm_estimator_enter();
  assert(thread_comm_estimator_held());
  running = &recovery; assert(!thread_comm_estimator_held()); running = &evaluator;

  /* A sleep inside the pass gives the guard up (even nested) and takes it
   * back at the same depth. */
  thread_comm_estimator_enter();
  assert(guard->tx_mutex_ownership_count == 2);
  thread_comm_estimator_sleep(80);
  assert(sleeps == 1 && slept == 80 && held_during_sleep == 0);
  assert(guard->tx_mutex_owner == &evaluator && guard->tx_mutex_ownership_count == 2);
  thread_comm_estimator_leave();
  thread_comm_estimator_leave();
  assert(!thread_comm_estimator_held() && guard->tx_mutex_owner == 0);

  /* Outside a pass (the descent state machine on the distribution thread) the
   * sleep is a plain sleep and takes nothing. */
  thread_comm_estimator_sleep(5);
  assert(sleeps == 2 && slept == 85 && guard->tx_mutex_owner == 0);
  /* Command queue: FIFO of (command, origin); the network probe follows the
   * newest accepted network command until the receiver takes it. */
  thread_comm_msg_t m;
  assert(queue->words == 2 && queue->capacity == THREAD_COMM_QUEUE_DEPTH);
  assert(g_last_network_flight_command_msg == UINT32_MAX);
  assert(thread_comm_receive(&m, TX_NO_WAIT) == TX_QUEUE_EMPTY);
  assert(thread_comm_send(0x80020005u, TX_WAIT_FOREVER) == TX_SUCCESS && last_send_wait == TX_WAIT_FOREVER);
  assert(g_last_network_flight_command_msg == UINT32_MAX);
  assert(thread_comm_send_network(0x00020003u) == TX_SUCCESS && last_send_wait == TX_NO_WAIT);
  assert(g_last_network_flight_command_msg == 0x00020003u);
  assert(thread_comm_receive(&m, TX_NO_WAIT) == TX_SUCCESS);
  assert(m.cmd == 0x80020005u && m.origin == THREAD_COMM_LOCAL);
  assert(g_last_network_flight_command_msg == 0x00020003u);
  assert(thread_comm_receive(&m, TX_NO_WAIT) == TX_SUCCESS);
  assert(m.cmd == 0x00020003u && m.origin == THREAD_COMM_NETWORK);
  assert(g_last_network_flight_command_msg == UINT32_MAX);
  /* Full queue: a refused network command leaves no probe behind. */
  for (unsigned i = 0; i < THREAD_COMM_QUEUE_DEPTH; ++i) assert(thread_comm_send(i, TX_NO_WAIT) == TX_SUCCESS);
  assert(thread_comm_send(99u, TX_NO_WAIT) == TX_QUEUE_FULL);
  assert(thread_comm_send_network(7u) == TX_QUEUE_FULL && g_last_network_flight_command_msg == UINT32_MAX);
  /* A command equal to a queued local one is still identified by origin. */
  thread_comm_flush();
  assert(thread_comm_send(0x00020003u, TX_NO_WAIT) == TX_SUCCESS);
  assert(thread_comm_send_network(0x00020003u) == TX_SUCCESS);
  assert(thread_comm_receive(&m, TX_NO_WAIT) == TX_SUCCESS && m.origin == THREAD_COMM_LOCAL);
  assert(thread_comm_receive(&m, TX_NO_WAIT) == TX_SUCCESS && m.origin == THREAD_COMM_NETWORK);
  assert(thread_comm_send_network(5u) == TX_SUCCESS);
  thread_comm_flush();
  assert(g_last_network_flight_command_msg == UINT32_MAX && thread_comm_receive(&m, TX_NO_WAIT) == TX_QUEUE_EMPTY);
  /* Receiver detection: blocked on this queue only. */
  TX_THREAD control = {3, TX_QUEUE_SUSP, queue}, other = {4, TX_QUEUE_SUSP, &creates};
  assert(thread_comm_receiver_waiting(&control) && !thread_comm_receiver_waiting(&other));
  control.tx_thread_state = TX_READY; assert(!thread_comm_receiver_waiting(&control));
  puts("thread_comm ok");
  return 0;
}
'''


class ThreadCommTests(unittest.TestCase):
    def test_estimator_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            (work / 'tx_api.h').write_text(TX_API)
            (work / 'harness.c').write_text(HARNESS)
            exe = work / 'harness'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', '-fsanitize=address,undefined',
                            f'-I{work}', f'-I{ROOT / "Core/Inc"}', str(work / 'harness.c'),
                            str(ROOT / 'Core/Src/thread_comm.c'), '-o', str(exe)], check=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
            self.assertIn('thread_comm ok', out.stdout)

    def test_no_direct_queue_use_outside_thread_comm(self):
        for path in sorted((ROOT / 'Core/Src').glob('*.c')):
            if path.name in ('thread_comm.c', 'telemetry.c', 'can_tx_queue.c', 'can_bus.c'):
                continue
            text = path.read_text()
            self.assertNotIn('seds_syscall', text, path.name)
            self.assertNotRegex(text, r'\btx_queue_(send|receive|flush)\(', path.name)

    def test_created_before_any_thread(self):
        app = (ROOT / 'Core/Src/app_threadx.c').read_text()
        self.assertLess(app.index('thread_comm_init()'), app.index('create_telemetry_task('))
        self.assertLess(app.index('thread_comm_init()'), app.index('create_recovery_task('))
        self.assertIn('Core/Src/thread_comm.c', (ROOT / 'CMakeLists.txt').read_text())


if __name__ == '__main__':
    unittest.main()

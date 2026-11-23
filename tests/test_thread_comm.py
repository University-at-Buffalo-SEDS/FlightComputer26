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
typedef unsigned long ULONG;
typedef char CHAR;
typedef void VOID;
#define TX_SUCCESS 0U
#define TX_NOT_OWNED 0x1EU
#define TX_INHERIT 1U
#define TX_WAIT_FOREVER 0xFFFFFFFFUL
#define TX_NO_WAIT 0UL
typedef struct TX_THREAD { int id; } TX_THREAD;
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
#include "thread_comm.h"
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

    def test_created_before_any_thread(self):
        app = (ROOT / 'Core/Src/app_threadx.c').read_text()
        self.assertLess(app.index('thread_comm_init()'), app.index('create_telemetry_task('))
        self.assertLess(app.index('thread_comm_init()'), app.index('create_recovery_task('))
        self.assertIn('Core/Src/thread_comm.c', (ROOT / 'CMakeLists.txt').read_text())


if __name__ == '__main__':
    unittest.main()

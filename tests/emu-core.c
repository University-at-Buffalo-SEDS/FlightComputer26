/*
 * Core functions used for emulating
 * parts of the flight computer.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "platform.h"

/* Header for source being tested */
#include "deployment.h"

static task_t tasks[EMU_TASKS];

void proper_sleep(time_t sec, long nsec)
{
  if (nsec < 0) {
    nsec = 0;
  } else if (nsec >= 1e9L) {
    sec += nsec / NS_IN_SEC;
    nsec = nsec % NS_IN_SEC;
  }

  struct timespec delay = {sec, nsec};
  nanosleep(&delay, NULL);
}

/*
 * TODO populate tasks[] with emu filter function(s)
 */
void emu_yield(unsigned *thread)
{
  printf("[RTOS] Running task %u\n (%u cycles)",
         *thread, FAKE_YIELD_CYCLES);

  for (uint_fast8_t t = 0; t < FAKE_YIELD_CYCLES; ++t) {
    tasks[*thread](FAKE_THREAD_INPUT);
  }
}

void emu_sleep(unsigned ticks)
{
  float duration = TX_TO_SEC(ticks);
  printf("[RTOS] Thread requested to sleep for %u ticks"
         "(%f seconds)\n", ticks, duration);

  if (ticks >= MIN_TICKS_TO_YIELD) {
    /* Scheduling at its best */
    unsigned t = random() % EMU_TASKS;
    emu_yield(&t);
  } else {
    time_t sec = (time_t)duration;
    long nsec = (long)((duration - (double)sec) * 1e9);
    proper_sleep(sec, nsec);
  }
}

/*
 * Illegal compatibility adapter
 * between threadx and pthread.
 */
void *emu_adapter(void *p)
{
  callee_t *t = (callee_t *)p;
  t->fn(t->arg);
  free(p);
  return NULL;
}

unsigned emu_create_thread(TX_THREAD *thread, char *name,
                           task_t entry, ULONG input,
                           unsigned long *stack, unsigned stack_size,
                           unsigned priority, unsigned preemption,
                           unsigned time_slice, unsigned autostart)
{
  (void)stack; (void)stack_size;
  (void)priority; (void)preemption;
  (void)time_slice; (void)autostart;

  printf("[RTOS] Creating thread \"%s\"\n", name);
  tasks[*thread] = entry;

  pthread_t id;
  void *result;

  callee_t *t = malloc(sizeof(*t));
  if (!t) goto error;

  t->fn = entry;
  t->arg = input;

  if (pthread_create(&id, NULL, emu_adapter, t))
    goto error;

  if (pthread_join(id, &result))
    goto error;

  return TX_SUCCESS;

error:
  free(t);
  return TX_FAILURE;
}

void emu_print_buf(emu_data_e type, void *buf,
                   size_t size, size_t element_size)
{
  (void)type; (void)size; (void)element_size;
  printf("[MSG] %s", (char *)buf);
}

void emu_print_err(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  fprintf(stderr, "[ERR] %s", buf);
}

void emu_print_event(void *buf)
{
  printf("[EVENT] %s", (char *)buf);
}
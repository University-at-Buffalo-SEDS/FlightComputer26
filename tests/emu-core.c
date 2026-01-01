/*
 * Core functions used for emulating
 * parts of the flight computer.
 */

#include <bits/time.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "platform.h"

#include "deployment.h"

static task_t tasks[EMU_TASKS];

uint32_t emu_time_ms()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((ts.tv_sec * 1e3) + (ts.tv_nsec / 1e6));
}

int proper_sleep(time_t sec, long nsec)
{
  if (nsec < 0) {
    nsec = 0;
  } else if (nsec >= 1e9L) {
    sec += nsec / NS_IN_SEC;
    nsec = nsec % NS_IN_SEC;
  }

  struct timespec delay = {sec, nsec};
  return nanosleep(&delay, NULL);
}

int emu_sleep(UINT ticks)
{
  float duration = TX_TO_SEC(ticks);

  printf("[RTOS] Thread requested to sleep for %u ticks"
         " (%f seconds)\n", ticks, duration);

  time_t sec = (time_t)duration;
  long nsec = (long)((duration - (double)sec) * 1e9);
  return proper_sleep(sec, nsec);
}

void emu_yield(TX_THREAD *thread)
{
  /* Since emulation is multithreaded, yielding is redundant: 
   * requested thread is already running and this can just wait. */
  (void)thread;
  emu_sleep(YIELD_SLEEP);
}

/// Compatibility adapter between threadx and pthread.
void *emu_adapter(void *p)
{
  callee_t *t = (callee_t *)p;
  t->fn(t->arg);
  free(p);
  return NULL;
}

UINT emu_create_thread(TX_THREAD *thread, char *name,
                       task_t entry, ULONG input,
                       ULONG *stack, UINT stack_size,
                       UINT priority, UINT preemption,
                       UINT time_slice, UINT autostart)
{
  (void)stack; (void)stack_size;
  (void)priority; (void)preemption;
  (void)time_slice; (void)autostart;

  printf("[RTOS] Creating thread \"%s\"\n", name);
  tasks[*thread] = entry;

  pthread_t id;
  callee_t *t = malloc(sizeof(*t));
  if (!t)
    return TX_FAILURE;

  t->fn = entry;
  t->arg = input;

  if (pthread_create(&id, NULL, emu_adapter, t)) {
    free(t);
    return TX_FAILURE;
  }

  if (pthread_detach(id))
    return TX_FAILURE;

  return TX_SUCCESS;
}

void emu_print_buf(emu_data_e type, void *buf,
                   size_t size, size_t element_size)
{
  (void)type; (void)size; (void)element_size;
  printf("[MSG] %s\n", (char *)buf);
}

void emu_print_err(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  fprintf(stderr, "[ERR] %s\n", buf);
}

void emu_print_event(void *buf)
{
  printf("[EVENT] %s\n", (char *)buf);
}
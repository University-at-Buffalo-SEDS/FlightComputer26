/*
 * Core functions used for emulating
 * parts of the flight computer.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#include "platform.h"

typedef void *(*task_t)(void *);

#define EMU_TASKS           4u
#define MIN_TICKS_TO_YIELD  50u
#define FAKE_THREAD_INPUT   0u
#define FAKE_YIELD_CYCLES   4u

#define TX_TO_SEC(ticks) (((float)ticks) / 100.0f)

unsigned sensor_thread = 0;
unsigned kalman_thread = 1;
unsigned telemetry_thread = 2;
unsigned deployment_thread = 3;

static task_t tasks[EMU_TASKS];

void emu_yield(unsigned *thread)
{
  printf("[RTOS] Running task %u\n (%u cycles)",
         *thread, FAKE_YIELD_CYCLES);

  for (int t = 0; t < FAKE_YIELD_CYCLES; ++t) {
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
    sleep(duration);
  }
}

unsigned emu_create_thread(unsigned *thread, char *name,
                           task_t entry, unsigned input,
                           unsigned long *stack, unsigned stack_size,
                           unsigned priority, unsigned preemption,
                           unsigned time_slice, unsigned autostart)
{
  (void)input;
  (void)stack; (void)stack_size;
  (void)priority; (void)preemption;
  (void)time_slice; (void)autostart;

  printf("[RTOS] Creating thread \"%s\"\n", name);
  tasks[*thread] = entry;

  pthread_t id;
  void *result;

  if (!pthread_create(&id, NULL, entry, NULL))
    return FC_TX_FAILURE;

  if (pthread_join(id, &result))
    return FC_TX_FAILURE;

  return FC_TX_SUCCESS;
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
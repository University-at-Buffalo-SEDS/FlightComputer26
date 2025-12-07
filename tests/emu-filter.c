/*
 * Tools for emulation of any data producer
 * for the deployment thread.
 */

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "deployment.h"
#include "platform.h"

#define FILTER_RING_SIZE 32
#define FILTER_RING_MASK (FILTER_RING_SIZE - 1)

filter_t filter_ring[FILTER_RING_SIZE] = {0};
atomic_uint_fast16_t newdata = 0;

static inline float fgen(float low, float high)
{
  return low + (high - low)*((float)rand() / (float)(RAND_MAX + 1u));
}

static inline atomic_uint_fast16_t incr()
{
  return atomic_fetch_add_explicit(&newdata, 1, memory_order_acq_rel);
}

void filter_init()
{
  srand(time(0));
}
// Static Linked List (SLL) for SPSC scenario.
// Lock-free and conveys order of data to consumer.

#include <stdint.h>
#include <stdatomic.h>

#include "ukf.h"
#include "deployment.h"

#define POOL_SIZE 32
#define POOL_MASK (POOL_SIZE - 1)

uint_fast8_t next[POOL_SIZE] = {0};
filter_t pool[POOL_SIZE] = {0};

atomic_uint_fast8_t top = 0u;
atomic_uint_least32_t mask = 0u;

inline uint_fast8_t sll_select()
{
  uint_least32_t m = atomic_load_explicit(&mask, memory_order_acquire);

  for (uint_fast8_t k = POOL_MASK; k >= 0; --k)
  {
    uint_least32_t choice = (1u << k);

    if (!(m & choice)) {
      atomic_fetch_or_explicit(&mask, choice, memory_order_release);
      return k;
    }
  }

  return UINT_FAST8_MAX;
}

inline void sll_clean(uint_fast8_t curr, uint_fast8_t end)
{
  while (curr != end)
  {
    uint_fast8_t k = curr;
    curr = next[k];
    atomic_fetch_and_explicit(&mask, ~(1u << k), memory_order_release);
  }
}

/* Prototypes for producer and consumer */

void sll_produce() // one sample
{
  /* Before calling this function, must be certain
   * that there is data to write to pool. */
  uint_fast8_t k = sll_select();

  if (k != UINT_FAST8_MAX) {
    // Write stuff to pool[k]
    next[k] = atomic_exchange_explicit(&top, k, memory_order_release);
  }
}

void sll_consume() // several samples
{
  static uint_fast8_t top_cached = 0u;

  uint_fast8_t top_new = atomic_load_explicit(&top, memory_order_acquire);
  uint_fast8_t t = top_new;

  if (t != top_cached) {

    for (uint_fast8_t i = 0; t != top_cached && i < MAX_SAMPLE; ++i)
    {
      uint_fast8_t k = t;
      t = next[k];
      // Do stuff with pool[k]
      atomic_fetch_and_explicit(&mask, ~(1u << k), memory_order_release);
    }

    if (t != top_cached)
      sll_clean(t, top_cached);

    top_cached = top_new;
  }
}
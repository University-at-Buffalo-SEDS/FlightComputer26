/* Core/Src/fchooks.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fctasks.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "simulation.h"


static volatile fu32 lock_fails = 0;
static volatile fu32 unlock_fails = 0;
static volatile fu32 alloc_fails = 0;

static volatile conditional bool mem_hint = 0;
static volatile conditional bool mu_hint = 0;

static TX_BYTE_POOL reserve;

#ifdef EXPORT_SPINLOCK
static spinlock external_lock = {0};
#endif


/* Panic handlers */

static conditional noreturn void panic_memory(void)
{
  task_loop (DO_NOT_EXIT)
  {
    /* 2 fast blue, 1 slow green */

    blink(Blue, false, 2);
    blink(Green, true, 1);
  }
}

static conditional noreturn void panic_lock(void)
{
  task_loop (DO_NOT_EXIT)
  {
    /* 2 fast green, 1 slow blue */

    blink(Green, false, 2);
    blink(Blue, true, 1);
  }
}

static conditional noreturn void panic_alloc(void)
{
  task_loop (DO_NOT_EXIT)
  {
    /* 2 fast blue, 3 slow green */

    blink(Blue, false, 2);
    blink(Green, true, 3);
  }
}

static conditional noreturn void panic_unknown(void)
{
  task_loop (DO_NOT_EXIT)
  {
    /* 3 fast blue, 3 fast green */

    blink(Blue, false, 3);
    blink(Green, false, 3);
  }
}


/* Panic string parser */

static inline conditional bool
panics_for(const char *s, size_t n, const char *needle)
{
  if (!s || n <= 0 || !needle)
  {
    return false;
  }

  size_t needle_len = strlen(needle);

  if (needle_len == 0U || n < needle_len)
  {
    return false;
  }

  for (size_t i = 0; i + needle_len <= n; ++i)
  {
      size_t j = 0;

      for (; j < needle_len; ++j)
      {
          char a = s[i + j];
          char b = needle[j];

          if (a >= 'A' && a <= 'Z')
          {
            a = (char)(a - 'A' + 'a');
          }
          if (b >= 'A' && b <= 'Z')
          {
            b = (char)(b - 'A' + 'a');
          }

          if (a != b) break;
      }

      if (j == needle_len)
      {
        return true;
      }
  }

  return false;
}


/* TX mutex wrapper */

static inline conditional void fchook_lock(TX_MUTEX *mu)
{
#ifdef EXPORT_SPINLOCK

  if (tx_thread_identify() != TX_NULL)
  {
    fc_lock(&external_lock);
  }
  else ++lock_fails;

#else

  if (tx_mutex_get(mu, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    ++lock_fails;
  }

#endif /* EXPORT_SPINLOCK */
}

static inline conditional void fchook_unlock(TX_MUTEX *mu)
{
#ifdef EXPORT_SPINLOCK

  if (tx_thread_identify() != TX_NULL)
  {
    fc_unlock(&external_lock, true);
  }
  else ++unlock_fails;

#else

  if (tx_mutex_put(mu) != TX_SUCCESS)
  {
    ++unlock_fails;
  }

#endif /* EXPORT_SPINLOCK */
}


/* Fault-tolerant allocator wrapper */

static inline void *reserve_alloc(size_t size, size_t timeout)
{
  static bool restricted = false;

  void *ptr;
  UINT st = tx_byte_allocate(&reserve, &ptr, size, timeout);

  if (st == TX_SUCCESS)
  {
    return ptr;
  }
  else if (!restricted)
  {
    restricted = true;
    fc_msg cmd = fc_mask(Log_Restrict);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }

  st = tx_byte_allocate(tx_app_shared, &ptr, size, timeout);

  if (st != TX_SUCCESS)
  {
    fc_msg cmd = fc_mask(Log_Terminate);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);

    /* Assert unreachable */

    ++alloc_fails;
    return NULL;
  }

  return ptr;
}

static inline conditional void *
fchook_alloc(TX_BYTE_POOL *bp, size_t size, size_t timeout)
{
  static bool rate_limited = false;

  if (size == 0)
  {
    size = EXCESS_ALIGN;
  }
  else if (size & ALIGN_MASK)
  {
    size = (size + ALIGN_MASK) & ~ALIGN_MASK;
  }

  if (bp == NULL)
  {
    return reserve_alloc(size, timeout);
  }

  void *ptr;
  UINT st;
  
  for (fu8 k = 0; k < POOL_RETRIES; ++k)
  {
    st = tx_byte_allocate(bp, &ptr, size, timeout);

    if (st == TX_SUCCESS)
    {
      break;
    }
  }

  if (st == TX_SUCCESS)
  {
    return ptr;
  }
  else if (!rate_limited)
  {
    rate_limited = true;
    fc_msg cmd = fc_mask(Log_Rate_Limit);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }

  return reserve_alloc(size, timeout);
}

static inline conditional void fchook_free(void *ptr)
{
  tx_byte_release(ptr);  
}

void try_allocate_reserve_pool(void)
{
  extern uint8_t _end[];
  extern uint8_t _estack[];
  static uint8_t *curr_heap = NULL;

  if (curr_heap == NULL)
  {
    uintptr_t aligned_end = ((uintptr_t)_end + 3) & ~3;
    curr_heap = (uint8_t *)aligned_end;
  }

  uintptr_t stack_top = (uintptr_t)_estack;
  uintptr_t heap_limit = stack_top - MSP_STACK_MARGIN;

  heap_limit &= ~3;

  if ((uintptr_t)curr_heap >= heap_limit)
  {
    blink(Green, false, 2);
    blink(Blue, true, 4);
    return;
  }

  size_t psize = (size_t)(heap_limit - (uintptr_t)curr_heap);

  if (tx_byte_pool_create(&reserve, "RES",
                          curr_heap, psize) != TX_SUCCESS)
  {
    Error_Handler(); 
  }

  curr_heap = (uint8_t *)heap_limit;
}


/* Telemetry registration */

#ifdef TELEMETRY_ENABLED

void seds_error_msg(const char *str, size_t len)
{
  if (str != NULL && len > 0U)
  {
    mem_hint = panics_for(str, len, "alloc")  ||
               panics_for(str, len, "memory") ||
               panics_for(str, len, "oom");

    mu_hint = panics_for(str, len, "mutex") ||
              panics_for(str, len, "lock");

    printf("%.*s\r\n", (int)len, str);
  }
}

void telemetry_lock(void)
{
  fchook_lock(&telemetry_mu);
}

void telemetry_unlock(void)
{
  fchook_unlock(&telemetry_mu);
}

void *telemetryMalloc(size_t xSize)
{
  return fchook_alloc(&telemetry_pool, xSize, 5);
}

void telemetryFree(void *pv)
{
  fchook_free(pv);
}

void telemetry_panic_hook(const char *str, size_t len)
{
  if (panics_for(str, len, "alloc") || alloc_fails)
  {
    panic_alloc();
  }

  if (panics_for(str, len, "memory") || mem_hint)
  {
    panic_memory();
  }

  if (panics_for(str, len, "mutex") || panics_for(str, len, "lock")
      || mu_hint || lock_fails + unlock_fails != 0U)
  {
    panic_lock();
  }

  panic_unknown();
}

#endif /* TELEMETRY_ENABLED */
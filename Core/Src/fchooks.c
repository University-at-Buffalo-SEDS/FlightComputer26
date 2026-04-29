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

static spinlock alloc_lock = {0};

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
    fc_unlock(&alloc_lock, false);
    return ptr;
  }
  else if (!restricted)
  {
    restricted = true;
    fc_msg cmd = fc_mask(Log_Restrict);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }

  st = tx_byte_allocate(tx_app_shared, &ptr, size, timeout);

  while (st != TX_SUCCESS)
  {
    do {
      fc_msg cmd = fc_mask(Log_Terminate);
      st = tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }
    while (st != TX_SUCCESS);

    /* Assert unreachable */

    ++alloc_fails;
    return NULL;
  }

  fc_unlock(&alloc_lock, false);
  return ptr;
}

static inline conditional void *
fchook_alloc(TX_BYTE_POOL *bp, size_t size, size_t timeout)
{
  static bool rate_limited = false;

  fc_lock(&alloc_lock, true);

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
#ifdef TEST_ALLOC
    else
    {
      blink(Green, false, st);
      blink(Blue, false, 1);
    }
#endif
  }

  if (st == TX_SUCCESS)
  {
    fc_unlock(&alloc_lock, false);
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
  fc_lock(&alloc_lock, true);
  tx_byte_release(ptr);  
  fc_unlock(&alloc_lock, false);
}

void try_allocate_reserve_pool(void)
{
  extern uint8_t _end[];
  extern uint8_t _estack[];

  uintptr_t brk = ((uintptr_t)_end + ALIGN_MASK) & ~ALIGN_MASK;
  uintptr_t lim = ((uintptr_t)_estack - MSP_STACK_MARGIN);

  if (brk >= lim)
  {
    return;
  }

  size_t psize = (size_t)(lim - brk);

  void *checkout = _sbrk(psize);

  if (checkout == (void *)-1)
  {
    if (tx_byte_pool_create(&reserve, "RES", 
                            checkout, psize) != TX_SUCCESS)
    {
      Error_Handler();
    }
  }
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
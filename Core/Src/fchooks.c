/* Core/Src/fchooks.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fctasks.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "panic_match.h"


static volatile fu32 lock_fails = 0;
static volatile fu32 unlock_fails = 0;
static volatile fu32 alloc_leaks = 0;

volatile fu32 g_telemetry_alloc_count = 0;
volatile fu32 g_telemetry_free_count = 0;
volatile fu32 g_telemetry_alloc_fail = 0;
volatile fu32 g_telemetry_panic_count = 0;
volatile ULONG g_telemetry_pool_available = 0;
volatile ULONG g_telemetry_pool_low_water = ~0UL;
volatile ULONG g_telemetry_pool_fragments = 0;

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
  MrAnalog (WE_ARE_SO_BACK)
  {
    /* 2 fast blue, 1 slow green */

    blink(Blue, false, 2);
    blink(Green, true, 1);
  }
}

static conditional noreturn void panic_lock(void)
{
  MrAnalog (WE_ARE_SO_BACK)
  {
    /* 2 fast green, 1 slow blue */

    blink(Green, false, 2);
    blink(Blue, true, 1);
  }
}

static conditional noreturn void panic_alloc(void)
{
  MrAnalog (WE_ARE_SO_BACK)
  {
    /* 2 fast blue, 3 slow green */

    blink(Blue, false, 2);
    blink(Green, true, 3);
  }
}

static conditional noreturn void panic_unknown(void)
{
  MrAnalog (WE_ARE_SO_BACK)
  {
    /* 3 fast blue, 3 fast green */

    blink(Blue, false, 3);
    blink(Green, false, 3);
  }
}


/* Panic string parser */

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
  static fu8 log_state = Log_User_Bound;

  if (log_state < Log_Rate_Bound)
  {
    log_state = Log_Rate_Bound;
    fc_msg cmd = fc_mask(Log_Rate_Limit);
    tx_queue_send(&seds_syscall, &cmd, TX_NO_WAIT);
  }

  void *ptr;
  UINT st = tx_byte_allocate(&reserve, &ptr, size, timeout);

  if (st == TX_SUCCESS)
  {
    fc_unlock(&alloc_lock);
    return ptr;
  }
  else if (log_state < Log_Local_Bound)
  {
    log_state = Log_Local_Bound;
    fc_msg cmd = fc_mask(Log_Restrict);
    tx_queue_send(&seds_syscall, &cmd, TX_NO_WAIT);
  }

  st = tx_byte_allocate(tx_app_shared, &ptr, size, timeout);

  while (st != TX_SUCCESS)
  {
    ++alloc_leaks;

    do {
      fc_msg cmd = fc_mask(Log_Terminate);
      st = tx_queue_send(&seds_syscall, &cmd, TX_NO_WAIT);
    }
    while (st != TX_SUCCESS);

    /* Assert unreachable */

    fc_unlock(&alloc_lock);
    return NULL;
  }

  fc_unlock(&alloc_lock);
  return ptr;
}

static inline conditional void *
fchook_alloc(TX_BYTE_POOL *bp, size_t size, size_t timeout)
{
#ifdef TEST_ALLOC
  if (tx_thread_identify() == TX_NULL)
  {
    while (1) blink(Blue, false, 1);
  }
#endif

  fc_lock(&alloc_lock);

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
    fc_unlock(&alloc_lock);
    return ptr;
  }

  return reserve_alloc(size, timeout);
}

static inline conditional void fchook_free(void *ptr)
{
#ifdef TEST_ALLOC
  if (tx_thread_identify() == TX_NULL)
  {
    while (1) blink(Green, false, 1);
  }
#endif

  fc_lock(&alloc_lock);
  tx_byte_release(ptr);  
  fc_unlock(&alloc_lock);
}

void try_allocate_reserve_pool(void)
{
  extern uint8_t _end[];
  extern uint8_t _estack[];

  uintptr_t brk = ((uintptr_t)_end + ALIGN_MASK) & ~ALIGN_MASK;
  uintptr_t lim = ((uintptr_t)_estack - MSP_STACK_MARGIN);

  if (brk >= lim)
  {
    goto no_reserve_exit;
  }

  size_t psize = (size_t)(lim - brk);

  void *checkout = _sbrk(psize);

  if (checkout == (void *)-1)
  {
    Error_Handler();
  }
  else if (tx_byte_pool_create(&reserve, "RES", 
                               checkout, psize) == TX_SUCCESS)
  {
    return;
  }

no_reserve_exit:
  message("WARNING: insufficient memory for reserve pool", true);
}


/* Telemetry registration */

#ifdef TELEMETRY_ENABLED

static inline void telemetry_memory_profile_sample(void)
{
  ULONG available = 0;
  ULONG fragments = 0;
  if (tx_byte_pool_info_get(&telemetry_pool, TX_NULL, &available, &fragments,
                            TX_NULL, TX_NULL, TX_NULL) == TX_SUCCESS)
  {
    g_telemetry_pool_available = available;
    g_telemetry_pool_fragments = fragments;
    if (available < g_telemetry_pool_low_water)
    {
      g_telemetry_pool_low_water = available;
    }
  }
}

void seds_error_msg(const char *str, size_t len)
{
  if (str != NULL && len > 0U)
  {
    mem_hint = fc_panic_message_contains(str, len, "alloc")  ||
               fc_panic_message_contains(str, len, "memory") ||
               fc_panic_message_contains(str, len, "oom");

    mu_hint = fc_panic_message_contains(str, len, "mutex") ||
              fc_panic_message_contains(str, len, "lock");

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
  void *ptr = fchook_alloc(&telemetry_pool, xSize, 5);
  if (ptr == NULL)
  {
    ++g_telemetry_alloc_fail;
  }
  else
  {
    ++g_telemetry_alloc_count;
  }
  telemetry_memory_profile_sample();
  return ptr;
}

void telemetryFree(void *pv)
{
  fchook_free(pv);
  ++g_telemetry_free_count;
  telemetry_memory_profile_sample();
}

void telemetry_panic_hook(const char *str, size_t len)
{
  ++g_telemetry_panic_count;
  if (fc_panic_message_contains(str, len, "alloc") || alloc_leaks)
  {
    panic_alloc();
  }

  if (fc_panic_message_contains(str, len, "memory") || mem_hint)
  {
    panic_memory();
  }

  if (fc_panic_message_contains(str, len, "mutex") ||
      fc_panic_message_contains(str, len, "lock")
      || mu_hint || lock_fails + unlock_fails != 0U)
  {
    panic_lock();
  }

  panic_unknown();
}

#endif /* TELEMETRY_ENABLED */

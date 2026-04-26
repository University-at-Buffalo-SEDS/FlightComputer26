/* Core/Src/fchooks.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fcapi.h"
#include "fcconfig.h"


static volatile fu32 lock_fails = 0;
static volatile fu32 unlock_fails = 0;
static volatile fu32 alloc_fails = 0;
static volatile fu8 mem_hint = 0;
static volatile fu8 mu_hint = 0;

static greedy_hdr *reserve = NULL;


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
  if (tx_thread_identify() == TX_NULL)
  {
    return;
  }
  if (tx_mutex_get(mu, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    ++lock_fails;
  }
}

static inline conditional void fchook_unlock(TX_MUTEX *mu)
{
  if (tx_mutex_put(mu) != TX_SUCCESS)
  {
    ++unlock_fails;
  }
}


/* Small fault-tolerant allocator */

static inline void *seds_alloc(size_t size, size_t walk_depth)
{
  if (size > ALLOC_MAX - sizeof(greedy_hdr))
  {
    ++alloc_fails;
    return NULL;
  }

  greedy_hdr *node = reserve;
  greedy_hdr *prev = NULL;

  for (size_t k = 0; node && k < walk_depth; ++k)
  {
    if (node->size >= size)
    {
      if (prev != NULL)
      {
        prev->next = node->next;
      }
      else reserve = node->next;

      node->next = ALLOC_MAGIC;
      return (void *)(node + 1); 
    }

    prev = node;
    node = node->next;
  }

  void *ptr = _sbrk(sizeof(greedy_hdr) + size);

  if (ptr == (void *)-1)
  {
    ptr = NULL;
    if (tx_byte_allocate(tx_app_shared, &ptr, size,
                          walk_depth) != TX_SUCCESS)
    {
      ++alloc_fails;
      return NULL;
    }
    else return ptr;
  }

  ((greedy_hdr *)ptr)->size = size;
  ((greedy_hdr *)ptr)->next = ALLOC_MAGIC;

  return (void *)((greedy_hdr *)ptr + 1);
}

static inline void seds_free(void *ptr)
{
  greedy_hdr *head = ((greedy_hdr *)ptr) - 1;

  if (head->next != ALLOC_MAGIC)
  {
    return;
  }

  head->next = reserve;
  reserve = head;
}

static inline conditional void *
fchook_alloc(TX_BYTE_POOL *bp, size_t size, size_t timeout)
{
  if (size == 0)
  {
    size = ALLOC_ALIGN;
  }
  else if (size & (ALLOC_ALIGN - 1))
  {
    size = (size + (ALLOC_ALIGN - 1)) & (ALLOC_ALIGN - 1);
  }

  if (!bp)
  {
    return seds_alloc(size, timeout);
  }

  void *ptr;

  if (tx_byte_allocate(bp, &ptr, size, timeout) == TX_SUCCESS)
  {
    return ptr;
  }

  return seds_alloc(size, timeout);
}

static inline conditional void fchook_free(void *ptr)
{
  if (ptr == NULL)
  {
    return;
  }
  if (tx_byte_release(ptr) == TX_PTR_ERROR)
  {
    seds_free(ptr);
  }
}


/* Telemetry registration */

#ifdef TELEMETRY_ENABLED

void seds_error_msg(const char *str, size_t len)
{
  if (str != NULL && len > 0U)
  {
    mem_hint = (fu8)(
      panics_for(str, len, "alloc") ||
      panics_for(str, len, "memory") ||
      panics_for(str, len, "oom"));

    mu_hint = (fu8)(
      panics_for(str, len, "mutex") ||
      panics_for(str, len, "lock"));

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
  if (panics_for(str, len, "alloc") || alloc_fails != 0)
  {
    panic_alloc();
  }

  if (panics_for(str, len, "memory") || mem_hint != 0)
  {
    panic_memory();
  }

  if (panics_for(str, len, "mutex") || panics_for(str, len, "lock")
      || mu_hint != 0 || lock_fails + unlock_fails != 0U)
  {
    panic_lock();
  }

  panic_unknown();
}

#endif /* TELEMETRY_ENABLED */
/*
 * Flight Computer primitive type aliases.
 */

#ifndef FC_TYPES
#define FC_TYPES

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <assert.h> 


typedef uint_fast8_t  fu8;
typedef uint_fast16_t fu16;
typedef uint_fast32_t fu32;
typedef uint_fast64_t fu64;

typedef int_fast8_t  fi8;
typedef int_fast16_t fi16;
typedef int_fast32_t fi32;
typedef int_fast64_t fi64;


#define serial        __attribute__((packed, aligned(4)))
#define tx_align      __attribute__((aligned(sizeof(ULONG))))
#define IREC26_unused __attribute__((unused))
#define constexpr     __attribute__((const))
#define blind_inline  __attribute__((always_inline))


typedef enum seds_atomic_mo {
  Rlx    = memory_order_relaxed,
  Con    = memory_order_consume,
  Acq    = memory_order_acquire,
  Rel    = memory_order_release,
  AcqRel = memory_order_acq_rel,
  SeqCst = memory_order_seq_cst
} mo;

#define load        atomic_load_explicit
#define store       atomic_store_explicit
#define swap        atomic_exchange_explicit
#define fetch_add   atomic_fetch_add_explicit
#define fetch_sub   atomic_fetch_sub_explicit
#define fetch_and   atomic_fetch_and_explicit
#define fetch_or    atomic_fetch_or_explicit
#define fetch_xor   atomic_fetch_xor_explicit
#define cas_weak    atomic_compare_exchange_weak_explicit
#define cas_strong  atomic_compare_exchange_strong_explicit


#endif /* FC_TYPES */
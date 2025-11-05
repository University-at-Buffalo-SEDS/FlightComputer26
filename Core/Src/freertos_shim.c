// Core/Src/freertos_shim.c
#include "tx_api.h"
#include <stddef.h>

/*
 * Rust expects these FreeRTOS-style heap functions:
 *
 *   void *pvPortMalloc(size_t);
 *   void vPortFree(void *);
 *
 * We implement them on top of a ThreadX byte pool.
 */

#define RUST_HEAP_SIZE  (32 * 1024u)  // tune this to whatever you need

static TX_BYTE_POOL rust_byte_pool;
static UCHAR rust_heap[RUST_HEAP_SIZE];

void rust_heap_init(void)
{
    static UINT initialized = 0;
    if (initialized) {
        return;
    }

    UINT status = tx_byte_pool_create(&rust_byte_pool,
                                      "rust_heap",
                                      rust_heap,
                                      sizeof(rust_heap));
    if (status != TX_SUCCESS) {
        /* If this fails, you're in deep trouble – spin or assert */
        while (1) { }
    }

    initialized = 1;
}

void *pvPortMalloc(size_t xSize)
{
    void *ptr = NULL;

    /* Make sure pool is ready – safe to call multiple times */
    rust_heap_init();

    /* TX_NO_WAIT: allocator is fast and non-blocking */
    UINT status = tx_byte_allocate(&rust_byte_pool, &ptr, xSize, TX_NO_WAIT);
    if (status != TX_SUCCESS) {
        return NULL;
    }
    return ptr;
}

void vPortFree(void *pv)
{
    if (pv == NULL) {
        return;
    }

    /* If the pool wasn’t created yet, something is badly wrong,
       but tx_byte_release() will fail and we just ignore it. */
    (void)tx_byte_release(pv);
}

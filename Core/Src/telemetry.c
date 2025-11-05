#include "telemetry.h"
#include "sedsprintf.h"
#include "stm32h5xx_hal.h"
#include "app_threadx.h"   // should bring in tx_api.h; if not, include tx_api.h directly
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
    (void)user;
    static uint32_t last32 = 0;
    static uint64_t high   = 0;
    uint32_t cur32 = HAL_GetTick();
    if (cur32 < last32) {
        high += (1ULL << 32);   /* 32-bit wrap (~49.7 days) */
    }
    last32 = cur32;
    return high | (uint64_t)cur32;
}

uint64_t node_now_since_ms(void *user) {
    (void)user;
    const uint64_t now = stm_now_ms(NULL);
    const RouterState s = g_router;           /* snapshot */
    return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = { .r = NULL, .created = 0, .start_time = 0 };

/* ---------------- Internal locking (ThreadX) ----------------
   We use a single ThreadX mutex to serialize all interactions with
   the router and its queues/handlers.

   - ThreadX mutexes are recursive by default, so this still behaves
     like the old CMSIS osMutexRecursive.
   - We enable priority inheritance (TX_INHERIT) to mimic
     osMutexPrioInherit.
*/

static TX_MUTEX g_router_mtx;
static UINT     g_router_mtx_created = TX_FALSE;

static void lock_init_once(void) {
    if (g_router_mtx_created == TX_FALSE) {
        /* Called from ThreadX thread context (after tx_kernel_enter). */
        UINT status = tx_mutex_create(&g_router_mtx,
                                      "routerMutex",
                                      TX_INHERIT);  /* priority inheritance on */

        if (status == TX_SUCCESS) {
            g_router_mtx_created = TX_TRUE;
        } else {
            /* Optional: handle failure more aggressively */
            printf("routerMutex create failed: %u\r\n", (unsigned)status);
        }
    }
}

static inline void LOCK(void) {
    if (g_router_mtx_created == TX_TRUE) {
        /* Block forever until we get the mutex */
        (void)tx_mutex_get(&g_router_mtx, TX_WAIT_FOREVER);
    }
}

static inline void UNLOCK(void) {
    if (g_router_mtx_created == TX_TRUE) {
        (void)tx_mutex_put(&g_router_mtx);
    }
}

/* ---------------- TX path (CANSEND) ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
    (void)user;
    (void)bytes;
    (void)len;

    /*TODO: Implement the cansend function*/

    return SEDS_OK;
}

/* ---------------- RX helpers ---------------- */
void rx_synchronous(const uint8_t *bytes, size_t len) {
    if (!bytes || !len) return;
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return;
    }
    LOCK();
    seds_router_receive_serialized(g_router.r, bytes, len);
    UNLOCK();
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
    if (!bytes || !len) return;
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return;
    }
    LOCK();
    seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
    UNLOCK();
}

/* ---------------- Local endpoint handler (SD_CARD) ---------------- */
SedsResult on_sd_packet(const SedsPacketView *pkt, void *user) {
    (void)user;

    /* TODO: Implement the saving to SD logic*/
    char buf[seds_pkt_to_string_len(pkt)];
    SedsResult s = seds_pkt_to_string(pkt, buf, sizeof(buf));
    if (s == SEDS_OK) {
        printf("on_sd_packet: %s\r\n", buf);
    } else {
        printf("on_sd_packet: seds_pkt_to_string failed (%d)\r\n", s);
    }
    return s;
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
    lock_init_once();

    /* Fast check without lock to avoid needless acquire in the common case. */
    if (g_router.created && g_router.r) return SEDS_OK;

    LOCK();
    if (g_router.created && g_router.r) {
        UNLOCK();
        return SEDS_OK;
    }

    const SedsLocalEndpointDesc locals[] = {
        { .endpoint = SEDS_EP_SD_CARD,
          .packet_handler = on_sd_packet,
          .user = NULL },
    };

    SedsRouter *r = seds_router_new(
        tx_send,               /* tx callback */
        NULL,                  /* tx_user */
        node_now_since_ms, /* clock */
        locals,
        (uint32_t)(sizeof(locals) / sizeof(locals[0]))
    );

    if (!r) {
        printf("Error: failed to create router\r\n");
        g_router.r = NULL;
        g_router.created = 0;
        UNLOCK();
        return SEDS_ERR;
    }

    g_router.r          = r;
    g_router.created    = 1;
    g_router.start_time = stm_now_ms(NULL);

    UNLOCK();
    return SEDS_OK;
}

/* ---------------- Logging APIs ---------------- */
SedsResult log_telemetry_synchronous(SedsDataType data_type,
                                     const void *data,
                                     size_t element_count,
                                     size_t element_size) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    if (!data || element_count == 0 || element_size == 0) return SEDS_ERR;

    const size_t total_bytes = element_count * element_size;
    LOCK();
    SedsResult res = seds_router_log(g_router.r, data_type, data, total_bytes);
    UNLOCK();
    return res;
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type,
                                      const void *data,
                                      size_t element_count,
                                      size_t element_size) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    if (!data || element_count == 0 || element_size == 0) return SEDS_ERR;

    const size_t total_bytes = element_count * element_size;
    LOCK();
    SedsResult res = seds_router_log_queue(g_router.r, data_type, data, total_bytes);
    UNLOCK();
    return res;
}

/* ---------------- Queue processing ---------------- */
SedsResult dispatch_tx_queue(void) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    LOCK();
    SedsResult res = seds_router_process_send_queue(g_router.r);
    UNLOCK();
    return res;
}

SedsResult process_rx_queue(void) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    LOCK();
    SedsResult res = seds_router_process_received_queue(g_router.r);
    UNLOCK();
    return res;
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    LOCK();
    SedsResult res = seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
    UNLOCK();
    return res;
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    LOCK();
    SedsResult res = seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
    UNLOCK();
    return res;
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
    if (!g_router.r) {
        if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
    }
    LOCK();
    SedsResult res = seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
    UNLOCK();
    return res;
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(const int32_t error_code) {
    /* Use a small fixed buffer to avoid big stack frames. */
    char buf[seds_error_to_string_len(error_code)];
    SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
    if (res == SEDS_OK) {
        printf("Error: %s\r\n", buf);
    } else {
        printf("Error: seds_error_to_string failed: %d\r\n", res);
    }
    return res;
}

/* ---------------- Fatal helper ---------------- */
void die(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    while (1) {
        printf("FATAL: %s\r\n", buf);
        HAL_Delay(1000);
    }
}

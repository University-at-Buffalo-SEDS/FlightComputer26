#include "telemetry.h"

#include "app_threadx.h"
#include "sedsprintf.h"
#include "stm32h5xx_hal.h"
#include <stm32h5xx_hal_fdcan.h>

#include "sd_card.h"
#include "can_bus.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  (void)data;
  (void)len;
}
#endif

static uint8_t g_can_rx_subscribed = 0;
static int32_t g_can_side_id = -1;

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
  (void)user;
  static uint32_t last32 = 0;
  static uint64_t high = 0;

  uint32_t cur32 = HAL_GetTick();
  if (cur32 < last32) {
    high += (1ULL << 32);
  }
  last32 = cur32;
  return high | (uint64_t)cur32;
}

uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = stm_now_ms(NULL);
  const RouterState s = g_router;
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {
    .r = NULL,
    .created = 0,
    .start_time = 0,
};

/* ---------------- TX helpers ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;

  if (!bytes || len == 0) {
    return SEDS_BAD_ARG;
  }

  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK)
             ? SEDS_OK
             : SEDS_ERR;
}

/* ---------------- Local endpoint handler (SD_CARD) ---------------- */
SedsResult on_sd_packet(const SedsPacketView *pkt, void *user) {
  (void)user;

  char buf[seds_pkt_to_string_len(pkt)];
  SedsResult s = seds_pkt_to_string(pkt, buf, sizeof(buf));
  if (s != SEDS_OK) {
    return s;
  }

  UINT fx = sd_logger_enqueue_line(buf, strlen(buf));
  return (fx == FX_SUCCESS) ? SEDS_OK : SEDS_ERR;
}

/* ---------------- RX helpers ---------------- */
static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  rx_asynchronous(data, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0) {
    return;
  }

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return;
    }
  }

  if (g_can_side_id >= 0) {
    seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r,
        (uint32_t)g_can_side_id,
        bytes,
        len);
  } else {
    seds_router_rx_serialized_packet_to_queue(
        g_router.r,
        bytes,
        len);
  }
#endif
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1;
    } else {
      printf("Error: can_bus_subscribe_rx failed\r\n");
    }
  }

  const SedsLocalEndpointDesc locals[] = {
      {.endpoint = SEDS_EP_SD_CARD,
       .packet_handler = on_sd_packet,
       .serialized_handler = NULL,
       .user = NULL},
      {.endpoint = SEDS_EP_FLIGHT_CONTROLLER,
       .packet_handler = on_fc_packet,
       .serialized_handler = NULL,
       .user = NULL},
  };

  /* NOTE: correct constructor signature */
  SedsRouter *r = seds_router_new(
      Seds_RM_Sink,
      node_now_since_ms,
      NULL,
      locals,
      (uint32_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  /* Register CAN as a routing side */
  g_can_side_id = seds_router_add_side_serialized(
      r,
      "can",
      3,
      tx_send,
      NULL,
      false /* reliable */);

  if (g_can_side_id < 0) {
    printf("Error: failed to add CAN side (%ld)\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = stm_now_ms(NULL);

  return SEDS_OK;
#endif
}

/* ---------------- Logging APIs ---------------- */
SedsResult log_telemetry_synchronous(
    SedsDataType data_type,
    const void *data,
    size_t element_count,
    size_t element_size) {

#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_BAD_ARG;

  return seds_router_log(
      g_router.r,
      data_type,
      data,
      element_count * element_size);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(
    SedsDataType data_type,
    const void *data,
    size_t element_count,
    size_t element_size) {

#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_BAD_ARG;

  return seds_router_log_queue(
      g_router.r,
      data_type,
      data,
      element_count * element_size);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

/* ---------------- Queue processing ---------------- */
SedsResult dispatch_tx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

/* ---------------- Error logging ---------------- */
SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  va_start(args, fmt);

  char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  return log_telemetry_asynchronous(
      SEDS_DT_GENERIC_ERROR,
      buf,
      strlen(buf),
      1);
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  va_start(args, fmt);

  char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  return log_telemetry_synchronous(
      SEDS_DT_GENERIC_ERROR,
      buf,
      strlen(buf),
      1);
#endif
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  char buf[seds_error_to_string_len(error_code)];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  }
  return res;
#endif
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

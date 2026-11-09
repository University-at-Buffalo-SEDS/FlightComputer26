#include <stdint.h>
#include "board_commissioning.h"

#ifdef TELEMETRY_ENABLED
extern int telemetry_router_enter(void);
extern int telemetry_router_leave(void);
#endif
#include "can_tx_queue.h"
#include "board_health.h"
/* Core/Src/telemetry.c */

#include "platform.h"
#include "board_watchdog.h"
#include "av_bay_underglow.h"
#include "flight_buzzer.h"
#include "flight_state_cache.h"
#include "fc_recovery_owner.h"
#include "board_recovery_transport.h"
#include "telemetry_rate.h"
#include "ota_stream.h"
#include "sim_network_probe.h"
#include "board_can_priority.h"
#include "fctypes.h"
#include "fcapi.h"
#include "fctasks.h"
#include "can_bus.h"
#include "simulation.h"
#include "testing.h"

#define id "TE "

#ifdef TELEMETRY_ENABLED
#include "board_packet_store.h"
#ifdef TELEMETRY_USE_TLSF
#include "telemetry_tlsf.h"
#endif

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  (void)data;
  (void)len;
}
#endif

#if defined(__GNUC__) || defined(__clang__)
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif

#ifndef TELEMETRY_TIMESYNC_MASTER_PRIO
#define TELEMETRY_TIMESYNC_MASTER_PRIO (-1)
#endif

#ifndef TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS
#define TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS 5000U
#endif

#ifndef TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS
#define TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS 2000U
#endif

#ifndef TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS
#define TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS 2000U
#endif

#ifndef TELEMETRY_QUEUE_SERVICE_BUDGET_MS
#define TELEMETRY_QUEUE_SERVICE_BUDGET_MS 1U
#endif

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

#define TELEMETRY_TIMESYNC_ROLE_CONSUMER 0U
#define TELEMETRY_TIMESYNC_ROLE_SOURCE 1U

static uint8_t g_can_rx_subscribed = 0U;
static int32_t g_can_side_id = -1;
#define FC_CAN_MAX_FRAME_BYTES CAN_TX_ALIGNED_FRAME_BYTES
#define FC_SIDE_TRANSPORT_TEMPLATES 4U
static uint8_t g_local_unix_valid = 0U;
static uint64_t g_local_unix_ms = 0ULL;

static uint8_t g_discovery_schema_announced = 0U;
static uint64_t g_discovery_schema_retry_ms = 0ULL;

#define TELEMETRY_ROUTER_RETRY_MS 500U
#ifdef TELEMETRY_ENABLED
static uint32_t g_router_retry_after_ms;
#endif
RouterState g_router = {.r = NULL, .created = 0U, .start_time = 0ULL};

/* Exported simulator/HIL health signals. A linked-bay test requires both a
 * remote discovery topology change and a valid SEDSNet network clock. */
volatile int32_t g_telemetry_rx_last_result __attribute__((used, externally_visible)) = SEDS_OK;
volatile uint32_t g_telemetry_rx_ok __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_rx_errors __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_rx_error_length __attribute__((used, externally_visible)) = 0U;
volatile uint8_t g_telemetry_rx_error_prefix[16] __attribute__((used, externally_visible));
volatile uint32_t g_telemetry_discovery_seen = 0U;
volatile uint32_t g_telemetry_timesync_valid = 0U;
/* Network clock restarts after it disagreed with the station clock by >60 s. */
volatile uint32_t g_network_clock_resyncs = 0U;
/* A restart cannot fix a source that disagrees with the GS; repeating it on
 * every GS packet kept the clock torn down. Monotonic ticks pace it. */
#define NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS (30U * (ULONG)TX_TIMER_TICKS_PER_SECOND)
static ULONG g_network_clock_resync_tick;
volatile uint32_t g_telemetry_network_ready = 0U;
volatile uint32_t g_telemetry_thread_entered __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_peer_mask = 0U;
volatile uint32_t g_sim_heartbeat_attempts = 0U;
volatile uint32_t g_sim_heartbeat_ok = 0U;
volatile uint32_t g_sim_heartbeat_fail = 0U;
volatile uint32_t g_sim_heartbeat_wire_tx = 0U;
volatile uint32_t g_sim_imu_publish_attempts
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sim_imu_publish_ok
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sim_baro_publish_ok
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sim_baro_publish_attempts
    __attribute__((used, externally_visible)) = 0U;
volatile int32_t g_sim_baro_publish_result
    __attribute__((used, externally_visible)) = 0;
volatile uint32_t g_telemetry_queue_errors = 0U;
/* A full hardware TX FIFO is expected while this board is alone on CAN: the
 * controller retains unacknowledged frames and SEDSNet retries them. Keep that
 * recoverable link state separate from allocator/router failures so an
 * isolated, healthy controller is not reported as crashed. */
volatile uint32_t g_telemetry_link_backpressure
    __attribute__((used, externally_visible)) = 0U;
volatile int32_t g_telemetry_last_queue_result
    __attribute__((used, externally_visible)) = SEDS_OK;
volatile uint32_t g_telemetry_discovery_poll_errors = 0U;
volatile uint32_t g_telemetry_timesync_poll_errors = 0U;
/* Last completed telemetry service stage. This remains available in release
 * ELFs so simulator/HIL failures identify the exact boot or loop boundary. */
volatile uint32_t g_telemetry_service_stage
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_rx_stage
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_tx_stage
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_loop_completions
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_stack_size
    __attribute__((used, externally_visible)) = TLMT_STACK_BYTES;
volatile uint32_t g_telemetry_stack_free_min
    __attribute__((used, externally_visible)) = TLMT_STACK_BYTES;
extern volatile uint32_t g_fdcan_tx_fail_count;
extern volatile uint32_t g_fdcan_tx_busy_count;
extern volatile uint32_t g_fdcan_last_error;
static uint32_t g_queue_observed_tx_fail_count = 0U;
static uint32_t g_queue_observed_tx_busy_count = 0U;

/* The evaluation and distribution threads also publish through the SEDSNet
 * router on their own stacks (flight-state and sensor packets). Keep their
 * low-water marks next to telemetry's so the simulator and HIL can check the
 * headroom of every router caller. */
volatile uint32_t g_evaluation_stack_free_min
    __attribute__((used, externally_visible)) = EVAL_STACK_BYTES;
volatile uint32_t g_distribution_stack_free_min
    __attribute__((used, externally_visible)) = DIST_STACK_BYTES;

static void stack_note_free(volatile uint32_t *low_water, uint32_t free_bytes) {
  if (free_bytes < *low_water) {
    *low_water = free_bytes;
  }
}

static void stack_sample_fill(const TX_THREAD *thread,
                              volatile uint32_t *low_water) {
  const uint32_t *cursor = (const uint32_t *)thread->tx_thread_stack_start;
  const uint32_t *const end = (const uint32_t *)thread->tx_thread_stack_end;

  if (cursor == NULL || end == NULL || cursor >= end) {
    return;
  }
  while (cursor < end && *cursor == 0xEFEFEFEFUL) {
    ++cursor;
  }
  stack_note_free(low_water, (uint32_t)((uintptr_t)cursor -
                                        (uintptr_t)thread->tx_thread_stack_start));
}

static void telemetry_sample_stack_margin(void) {
  stack_sample_fill(&telemetry_task, &g_telemetry_stack_free_min);
  stack_sample_fill(&evaluation_task, &g_evaluation_stack_free_min);
  stack_sample_fill(&distribution_task, &g_distribution_stack_free_min);
}

/* Stack-fill scanning only sees bytes that a function actually writes.  Large
 * Rust frames reserve their full size by moving PSP, but may leave most of the
 * reserved area filled with ThreadX's marker.  Sample the live process stack
 * pointer from the deepest board callback as well so HIL and the simulator see
 * the real routing-path headroom of whichever thread is sending. */
static void telemetry_sample_active_stack_margin(void) {
  const TX_THREAD *const thread = tx_thread_identify();
  volatile uint32_t *low_water;

  if (thread == &telemetry_task) {
    low_water = &g_telemetry_stack_free_min;
  } else if (thread == &evaluation_task) {
    low_water = &g_evaluation_stack_free_min;
  } else if (thread == &distribution_task) {
    low_water = &g_distribution_stack_free_min;
  } else {
    return;
  }

  const uintptr_t stack_start = (uintptr_t)thread->tx_thread_stack_start;
  const uintptr_t stack_end = (uintptr_t)thread->tx_thread_stack_end;
  const uintptr_t psp = (uintptr_t)__get_PSP();

  if (psp < stack_start || psp > stack_end) {
    return;
  }

  stack_note_free(low_water, (uint32_t)(psp - stack_start));
}

static void telemetry_record_queue_result(SedsResult result) {
  const uint32_t tx_fail_count = g_fdcan_tx_fail_count;
  const uint32_t tx_busy_count = g_fdcan_tx_busy_count;
  const uint8_t new_fifo_full =
      (tx_fail_count != g_queue_observed_tx_fail_count &&
       (g_fdcan_last_error & HAL_FDCAN_ERROR_FIFO_FULL) != 0U) ? 1U : 0U;
  g_queue_observed_tx_fail_count = tx_fail_count;
  const uint8_t new_busy =
      tx_busy_count != g_queue_observed_tx_busy_count ? 1U : 0U;
  g_queue_observed_tx_busy_count = tx_busy_count;
  const uint8_t observed_backpressure = new_fifo_full || new_busy;
  /* Queue service can retain refused work and return SEDS_OK. Observe the
   * actual hardware refusal rather than requiring an aggregate SDK error. */
  if (observed_backpressure != 0U) {
    g_telemetry_link_backpressure++;
  }
  if (result == SEDS_OK) {
    return;
  }
  g_telemetry_last_queue_result = (int32_t)result;
  /* SEDSNet presents a failed side callback as SEDS_HANDLER_ERROR. Confirm
   * that it came from a newly observed FDCAN FIFO-full or busy event before
   * treating it as recoverable backpressure; genuine packet-handler failures remain
   * fatal queue errors. */
  if (result == SEDS_IO ||
      (result == SEDS_HANDLER_ERROR && observed_backpressure != 0U)) {
    if (observed_backpressure == 0U) {
      g_telemetry_link_backpressure++;
    }
    return;
  }

  /* Blue is owned by managed underglow and the recovery indicator. A raw
   * toggle here can latch it on after a transient queue error. Keep errors
   * observable through counters without changing the requested LED state. */
  g_telemetry_queue_errors++;
}

static SedsResult Board_Recovery_handler(const SedsPacketView *, void *);
static SedsResult Manual_Abort_handler(const SedsPacketView *, void *);
static const SedsLocalEndpointDesc locals[] = {
  { .endpoint = SEDS_EP_ABORT, .packet_handler = Manual_Abort_handler, .user = NULL },
      {
          .endpoint = SEDS_EP_BOARD_RECOVERY,
          .packet_handler = Board_Recovery_handler,
          .packed_handler = NULL,
          .user = NULL,
      },
  { .endpoint = SEDS_EP_FLIGHT_CONTROLLER, .packet_handler = on_fc_packet, .user = NULL },
};

static uint64_t tx_raw_now_ms_locked(void) {
  const uint32_t ticks32 = (uint32_t)tx_time_get();
  return ((uint64_t)ticks32 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static UNUSED_FUNCTION uint64_t tx_raw_now_ms(void *user) {
  (void)user;
  return tx_raw_now_ms_locked();
}

static uint8_t telemetry_timesync_is_source(void) {
  return (TELEMETRY_TIMESYNC_MASTER_PRIO >= 0) ? 1U : 0U;
}

static uint64_t telemetry_timesync_priority(void) {
  // Holdover consumers must yield when RF (1) or GroundStation (50) returns.
  return telemetry_timesync_is_source() ? (uint64_t)TELEMETRY_TIMESYNC_MASTER_PRIO : 100ULL;
}

static uint32_t telemetry_timesync_role(void) {
  return telemetry_timesync_is_source() ? TELEMETRY_TIMESYNC_ROLE_SOURCE
                                        : TELEMETRY_TIMESYNC_ROLE_CONSUMER;
}

static bool telemetry_unix_ms_to_utc(uint64_t unix_ms, int32_t *year, uint8_t *month,
                                     uint8_t *day, uint8_t *hour, uint8_t *minute,
                                     uint8_t *second, uint16_t *millisecond) {
  static const uint16_t days_before_month[12] = {0U,   31U,  59U,  90U,  120U, 151U,
                                                 181U, 212U, 243U, 273U, 304U, 334U};
  uint64_t whole_seconds = unix_ms / 1000ULL;
  const uint64_t days_since_epoch = whole_seconds / 86400ULL;
  uint32_t seconds_of_day = (uint32_t)(whole_seconds % 86400ULL);
  int32_t y = 1970;
  uint64_t days = days_since_epoch;

  if (!year || !month || !day || !hour || !minute || !second || !millisecond) {
    return false;
  }

  while (1) {
    const uint32_t y_u32 = (uint32_t)y;
    const uint8_t leap =
        ((y_u32 % 4U) == 0U && ((y_u32 % 100U) != 0U || (y_u32 % 400U) == 0U)) ? 1U : 0U;
    const uint32_t days_in_year = leap ? 366U : 365U;
    if (days < days_in_year) {
      uint8_t m = 1U;
      uint32_t day_of_year = (uint32_t)days;
      for (; m <= 12U; ++m) {
        uint32_t month_start = days_before_month[m - 1U];
        uint32_t month_end =
            (m < 12U) ? days_before_month[m] : (uint32_t)(leap ? 366U : 365U);
        if (leap && m > 2U) {
          month_start += 1U;
          month_end += 1U;
        }
        if (day_of_year < month_end) {
          *year = y;
          *month = m;
          *day = (uint8_t)(day_of_year - month_start + 1U);
          *hour = (uint8_t)(seconds_of_day / 3600U);
          *minute = (uint8_t)((seconds_of_day % 3600U) / 60U);
          *second = (uint8_t)(seconds_of_day % 60U);
          *millisecond = (uint16_t)(unix_ms % 1000ULL);
          return true;
        }
      }
      return false;
    }
    days -= days_in_year;
    ++y;
  }
}

static SedsResult telemetry_apply_local_unix_time_locked(SedsRouter *router) {
  int32_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;

  if (!router || !telemetry_timesync_is_source() || !g_local_unix_valid) {
    return SEDS_OK;
  }

  if (!telemetry_unix_ms_to_utc(g_local_unix_ms, &year, &month, &day, &hour, &minute,
                                &second, &millisecond)) {
    return SEDS_BAD_ARG;
  }

  return seds_router_set_local_network_datetime_millis(router, year, month, day, hour, minute,
                                                       second, millisecond);
}

static SedsResult telemetry_configure_timesync_locked(SedsRouter *router) {
  SedsResult result;

  if (!router) {
    return SEDS_BAD_ARG;
  }

  result = seds_router_configure_timesync(
      router, true, telemetry_timesync_role(), telemetry_timesync_priority(),
      (uint64_t)TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS,
      (uint64_t)TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS,
      (uint64_t)TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS);
  if (result != SEDS_OK) {
    return result;
  }

  return telemetry_apply_local_unix_time_locked(router);
}

uint64_t telemetry_now_ms(void) { return tx_raw_now_ms_locked(); }

static uint64_t telemetry_unix_ms_locked(void) {
#ifndef TELEMETRY_ENABLED
  return g_local_unix_valid ? g_local_unix_ms : 0ULL;
#else
  uint64_t unix_ms = 0ULL;

  if (g_router.r && seds_router_get_network_time_ms(g_router.r, &unix_ms) == SEDS_OK) {
    return unix_ms;
  }

  if (telemetry_timesync_is_source() && g_local_unix_valid) {
    return g_local_unix_ms;
  }

  return 0ULL;
#endif
}

uint64_t telemetry_unix_ms(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return 0;
#endif
  uint64_t result = telemetry_unix_ms_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave()) return 0;
#endif
  return result;
}


uint64_t telemetry_unix_s(void) { return telemetry_unix_ms() / 1000ULL; }

uint8_t telemetry_unix_is_valid(void) { return telemetry_unix_ms() != 0ULL ? 1U : 0U; }

static void telemetry_set_unix_time_ms_locked(uint64_t unix_ms) {
  g_local_unix_ms = unix_ms;
  g_local_unix_valid = (unix_ms != 0ULL) ? 1U : 0U;

#ifdef TELEMETRY_ENABLED
  if (g_router.r != NULL) {
    (void)telemetry_apply_local_unix_time_locked(g_router.r);
  }
#endif
}

void telemetry_set_unix_time_ms(uint64_t unix_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  telemetry_set_unix_time_ms_locked(unix_ms);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static uint64_t node_now_since_ms(void *user) {
  (void)user;
  const RouterState s = g_router;
  const uint64_t now = tx_raw_now_ms_locked();
  /* Construction and initial maintenance also need an advancing clock. */
  return now - s.start_time;
}

static SedsResult tx_send_with_priority(const uint8_t *bytes, size_t len,
                                        uint8_t priority, void *user) {
  (void)user;
  g_telemetry_tx_stage = 1U;

  telemetry_sample_active_stack_margin();

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }
  g_telemetry_tx_stage = 2U;
  sim_probe_observe_can_tx(bytes, len);
  g_telemetry_tx_stage = 3U;

  const uint32_t can_id =
      board_can_arbitration_id(bytes, len, priority,
          sim_probe_packed_data_type(bytes, len), (uint32_t)SEDS_DT_HEARTBEAT, 0x003U);
  if (can_bus_send_large(bytes, len, can_id) != HAL_OK) {
    g_telemetry_tx_stage = 5U;
    return SEDS_IO;
  }

  g_telemetry_tx_stage = 4U;
  /* This is the physical CAN TX indicator. Keeping it on the successful send
   * edge makes the LED distinguish an idle-but-healthy router from real bus
   * activity and prevents a solid startup LED from masquerading as traffic. */
  led_toggle(light[Green].port, light[Green].pin);
  return SEDS_OK;
}

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  return tx_send_with_priority(bytes, len, 0U, user);
}


static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  g_telemetry_rx_stage = 1U;
  sim_probe_observe_packed(data, len);
  g_telemetry_rx_stage = 2U;
  rx_asynchronous(data, len);
  g_telemetry_rx_stage = 20U;
}

static void rx_asynchronous_locked(const uint8_t *bytes, size_t len) {
  g_telemetry_rx_stage = 10U;
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }
  g_telemetry_rx_stage = 11U;

  SedsResult result;

  if (g_can_side_id >= 0) {
    g_telemetry_rx_stage = 12U;
    result = seds_router_receive_packed_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    result = seds_router_receive_packed(g_router.r, bytes, len);
  }

  g_telemetry_rx_stage = 13U;
  g_telemetry_rx_last_result = result;
  if (result == SEDS_OK) {
    g_telemetry_rx_ok++;
    g_telemetry_discovery_seen = 1U;
  } else {
    g_telemetry_rx_errors++;
    g_telemetry_rx_error_length = (uint32_t)len;
    for (size_t i = 0; i < sizeof(g_telemetry_rx_error_prefix); ++i)
      g_telemetry_rx_error_prefix[i] = i < len ? bytes[i] : 0U;
  }
  g_telemetry_rx_stage = 14U;
#endif
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  rx_asynchronous_locked(bytes, len);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static UNUSED_FUNCTION void rx_synchronous_locked(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_packed_from_side(g_router.r, (uint32_t)g_can_side_id, bytes,
                                                   len);
  } else {
    (void)seds_router_receive_packed(g_router.r, bytes, len);
  }

#endif
}

static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  rx_synchronous_locked(bytes, len);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static void telemetry_update_network_health(SedsRouter *router) {
  uint64_t network_time_ms = 0ULL;
  const ULONG now_ticks = tx_time_get();
  if (seds_router_get_network_time_ms(router, &network_time_ms) == SEDS_OK) {
    g_telemetry_timesync_valid = 1U;
    if ((g_network_clock_resyncs == 0U ||
         (ULONG)(now_ticks - g_network_clock_resync_tick) >= NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS) &&
        board_recovery_network_clock_resync_needed(network_time_ms) &&
        telemetry_configure_timesync_locked(router) == SEDS_OK) {
      g_network_clock_resyncs++;
      g_network_clock_resync_tick = now_ticks;
      network_time_ms = 0ULL;
    }
  }
  if (g_telemetry_discovery_seen != 0U &&
      g_telemetry_timesync_valid != 0U) {
    g_telemetry_network_ready = 1U;
  }
}

static SedsResult telemetry_poll_timesync_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result = seds_router_poll_timesync(g_router.r, NULL);
  telemetry_update_network_health(g_router.r);

  return result;
#endif
}

SedsResult telemetry_poll_timesync(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_poll_timesync_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult telemetry_announce_discovery_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result = seds_router_announce_discovery(g_router.r);

  return result;
#endif
}

SedsResult telemetry_announce_discovery(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_announce_discovery_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult telemetry_poll_discovery_locked(void) {
#ifdef TELEMETRY_ENABLED
  if (g_router.r) board_health_poll(g_router.r);
#endif

#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  g_telemetry_service_stage = 301U;
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  g_telemetry_service_stage = 302U;
  bool did_queue = false;

  /* Cadence polling advertises routes, not schema. Publish the complete local
   * catalog once after startup so existing relays learn newly added types.
   * Leave queue processing running between bounded retries on queue pressure. */
  const uint64_t now = tx_raw_now_ms_locked();
  if (now >= g_discovery_schema_retry_ms) {
    if (!g_discovery_schema_announced) {
      g_discovery_schema_retry_ms = now + 1000ULL;
      if (seds_router_announce_discovery(g_router.r) == SEDS_OK) {
        g_discovery_schema_announced = 1U;
      }
    }
  }
  (void)av_bay_underglow_poll(g_router.r);
  board_commissioning_service();
  fc_recovery_network_service();
  (void)flight_state_cache_poll(g_router.r);
  /* This also enforces the two-second startup-buzz deadline. It must run
   * before discovery succeeds so an unplugged CAN bus cannot hold the buzzer
   * on indefinitely. */
  (void)flight_buzzer_poll(g_router.r);
  ota_stream_poll();
  g_telemetry_service_stage = 303U;
  g_telemetry_service_stage = 611U;
  const SedsResult result = seds_router_poll_discovery(g_router.r, &did_queue);
  g_telemetry_service_stage = 612U;
  if (result == SEDS_OK && g_telemetry_discovery_seen != 0U) {
    /* Flight's real discovery and telemetry packets already identify it on
     * avionics CAN. Do not inject an extra simulation-only heartbeat into a
     * saturated TX queue; qualification traffic must not perturb scheduling. */
    g_telemetry_service_stage = 613U;
    g_telemetry_service_stage = 614U;
  }
  telemetry_update_network_health(g_router.r);

  return result;
#endif
}

#ifdef TELEMETRY_ENABLED
bool fc_recovery_ota_reset_boundary_ready(void)
{
  /* The SDK lock hooks use this same recursive ThreadX mutex. A nested
   * application/SDK caller must not reset even after one successful put. */
  return __get_IPSR() == 0U && tx_thread_identify() == &telemetry_task &&
         telemetry_mu.tx_mutex_owner == TX_NULL &&
         telemetry_mu.tx_mutex_ownership_count == 0U;
}
#endif
SedsResult telemetry_poll_discovery(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_poll_discovery_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave()) return result == SEDS_OK ? SEDS_ERR : result;
  /* Reset-only drain: flash/confirmation remain at their original poll site. */
  ota_stream_reset_poll();
#endif
  return result;
}


#ifdef TELEMETRY_USE_TLSF
static __attribute__((unused)) bool telemetry_memory_admission_probe(
    size_t additional, size_t largest, uint32_t work)
{
  if (work == SEDS_MEMORY_WORK_SMALL_PROTECTED)
    return telemetry_tlsf_admit_small_protected(additional, largest);
  return telemetry_tlsf_admit(additional, largest);
}
#endif

static SedsResult init_telemetry_router_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  SedsRouter *r = NULL;
  SedsResult result = SEDS_OK;

  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  const uint64_t init_now_ms = tx_raw_now_ms_locked();
  /* Discovery and time-sync deadlines are created during router/side setup.
   * Establish the epoch first so node_now_since_ms never moves backwards once
   * those deadlines exist, including after a simulated or hardware reboot. */
  g_router.start_time = init_now_ms;

  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1U;
    }
  }

  g_telemetry_service_stage = 21U;
#ifdef TELEMETRY_USE_TLSF
  seds_set_memory_admission_probe_v2(telemetry_memory_admission_probe);
#endif
  result = board_packet_store_init();
  if (result != SEDS_OK) return result;

  /* Never try zstd payload compression on transmit. ZSTD_compress keeps a
   * 3.8 KiB context on the calling stack and then mallocs a workspace of at
   * least 18.9 KiB from the newlib heap, which here has 14,464 B free, so
   * every attempt failed after spending ~4.5 KiB of the router thread's stack
   * and the payload went out uncompressed anyway. */
  SedsRuntimeTuningConfig tuning;
  result = seds_get_runtime_tuning_config(&tuning);
  if (result != SEDS_OK) return result;
  tuning.payload_compress_threshold = SIZE_MAX;
  result = seds_set_runtime_tuning_config(&tuning);
  if (result != SEDS_OK) return result;

  r = seds_router_new(node_now_since_ms, NULL, locals,
                                              sizeof(locals) / sizeof(locals[0]));
  g_telemetry_service_stage = 22U;
  if (!r) {
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  if (seds_router_set_preferred_discovery_master(r, "GS", 2U) != SEDS_OK) {
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    return SEDS_ERR;
  }

  g_telemetry_service_stage = 23U;
  /* The H523 has three FDCAN TX FIFO elements. The CAN driver streams larger
   * side frames through those slots with hardware retransmission and a
   * bounded disconnected-bus timeout. */
  g_can_side_id = seds_router_add_side_packed_profile_with_priority(
      r, "can", 3U, tx_send_with_priority, NULL, false,
      SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE, FC_CAN_MAX_FRAME_BYTES, 0U,
      FC_SIDE_TRANSPORT_TEMPLATES);
  g_telemetry_service_stage = 24U;
  if (g_can_side_id < 0) {
    g_can_side_id = -1;
  }

  g_telemetry_service_stage = 25U;
  g_telemetry_service_stage = 26U;
  result = telemetry_configure_timesync_locked(r);
  g_telemetry_service_stage = 27U;
  if (result != SEDS_OK) {
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    return result;
  }

  g_telemetry_service_stage = 28U;
  result = av_bay_underglow_init(r);
  g_telemetry_service_stage = 29U;
  if (result != SEDS_OK) {
    seds_router_free(r);
    return result;
  }

  g_telemetry_service_stage = 30U;
  result = flight_buzzer_init(r);
  g_telemetry_service_stage = 31U;
  if (result != SEDS_OK) {
    seds_router_free(r);
    return result;
  }


  /* Discovery begins from the normal poll loop after CAN startup. */

  g_telemetry_service_stage = 32U;
  result = ota_stream_init(r);
  if (result != SEDS_OK) {
    seds_router_free(r);
    return result;
  }
  g_telemetry_service_stage = 33U;
  g_discovery_schema_announced = 0U;
  g_discovery_schema_retry_ms = 0ULL;
  result = flight_state_cache_init(r);
  if (result != SEDS_OK) {
    seds_router_free(r);
    return result;
  }
  /* Publish only a complete router. RX and lazy producers share its mutex. */
  board_recovery_transport_set_router(r);
  g_router.r = r;
  g_router.created = 1U;
  return SEDS_OK;
#endif
}
SedsResult init_telemetry_router(void)
{
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result;
#ifdef TELEMETRY_ENABLED
  const uint32_t now_ms = (uint32_t)tx_raw_now_ms_locked();
  if (g_router.created && g_router.r) {
    result = SEDS_OK;
  } else if (g_router_retry_after_ms != 0U &&
             (int32_t)(now_ms - g_router_retry_after_ms) < 0) {
    result = SEDS_IO;
  } else {
    result = init_telemetry_router_locked();
    if (result == SEDS_OK) {
      g_router_retry_after_ms = 0U;
    } else {
      g_router_retry_after_ms = now_ms + TELEMETRY_ROUTER_RETRY_MS;
      if (g_router_retry_after_ms == 0U) g_router_retry_after_ms = 1U;
    }
  }
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#else
  result = init_telemetry_router_locked();
#endif
  return result;
}




static SedsResult Manual_Abort_handler(const SedsPacketView *packet, void *user)
{
  (void)user;
  static const char manual[] = "SEDS_MANUAL_ABORT_V1:";
  if (packet == NULL || packet->ty != SEDS_DT_ABORT) return SEDS_BAD_ARG;
  /* Legacy/unmarked and automatic ABORT never acquire a new FC effect. */
  if(packet->payload==NULL)return SEDS_OK;
  bool known=packet->payload_len==sizeof manual&&!memcmp(packet->payload,manual,sizeof manual-1U)&&packet->payload[sizeof manual-1U]=='K';
  if(!known&&(packet->payload_len!=sizeof manual-1U||memcmp(packet->payload,manual,sizeof manual-1U)))return SEDS_OK;
  /* An unidentified source is acknowledged only while the FC is already
   * stopped; otherwise it stays undelivered so the sender retries it. */
  if (!board_recovery_manual_stop_source_allowed(g_router.r, packet)) {
    if (!fc_recovery_abort_latched()) return SEDS_PERMISSION_DENIED;
    ++g_reliable_refusal_count; return SEDS_OK;
  }
  if(known)fc_recovery_manual_stop_original_utc(packet->timestamp);else fc_recovery_manual_stop();
  return SEDS_OK; /* Admission only: actual GPIO/evaluator readback is QUERY. */
}

static SedsResult Board_Recovery_handler(const SedsPacketView *packet, void *user)
{
  (void)user;
  return board_recovery_handle(g_router.r, packet, SEDS_EP_BOARD_RECOVERY,
      (uint32_t)(((uint64_t)tx_time_get() * 1000U) / TX_TIMER_TICKS_PER_SECOND));
}



/* STM32's native little-endian numeric representation is SEDSNet's wire
 * representation. The bytes API still applies the schema's fixed-length
 * padding and routing, without linking conversions for every numeric type. */
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "FC telemetry payloads require little-endian serialization"
#endif

#ifdef SEDS_FIRMWARE_SIM_TEST
/* Observe real producer calls; never manufacture qualification packets.
 * Keep the same instrumentation in both publication paths without duplicating
 * its code in the nearly full simulator image. Production builds omit it. */
static __attribute__((noinline)) void record_sensor_publication(
    SedsDataType type, SedsResult result) {
  volatile uint32_t *attempts;
  volatile uint32_t *ok;
  if (type == SEDS_DT_BAROMETER_DATA) {
    attempts = &g_sim_baro_publish_attempts;
    ok = &g_sim_baro_publish_ok;
    g_sim_baro_publish_result = result;
  } else if (type == SEDS_DT_IMU_DATA) {
    attempts = &g_sim_imu_publish_attempts;
    ok = &g_sim_imu_publish_ok;
  } else {
    return;
  }
  ++*attempts;
  if (result == SEDS_OK) ++*ok;
}
#endif

static SedsResult log_telemetry_synchronous_locked(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U ||
      (element_size != 1U && element_size != 2U && element_size != 4U &&
       element_size != 8U && element_size != 16U) ||
      element_count > SIZE_MAX / element_size) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result =
      seds_router_log_bytes(g_router.r, data_type, data, element_count * element_size);
#ifdef SEDS_FIRMWARE_SIM_TEST
  record_sensor_publication(data_type, result);
#endif

  return result;
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_synchronous_locked(data_type, data, element_count, element_size);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult log_telemetry_asynchronous_locked(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U ||
      (element_size != 1U && element_size != 2U && element_size != 4U &&
       element_size != 8U && element_size != 16U) ||
      element_count > SIZE_MAX / element_size) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  /* The CAN driver already provides bounded transport backpressure. Keeping a
   * second heap-backed SEDSNet queue increases latency and can retain stale
   * sensor packets indefinitely when the bus is congested. */

  /* The rate limiter is shared by every sensor producer. Keep its timestamp
   * table under the same lock as the router so simultaneous producer threads
   * cannot race and corrupt the per-type deadline state. */
  if (!fc_telemetry_rate_allow(data_type, HAL_GetTick())) {

    return SEDS_OK;
  }
  const SedsResult result =
      seds_router_log_bytes(g_router.r, data_type, data, element_count * element_size);
#ifdef SEDS_FIRMWARE_SIM_TEST
  record_sensor_publication(data_type, result);
#endif

  return result;
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_asynchronous_locked(data_type, data, element_count, element_size);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult log_telemetry_string_asynchronous_locked(SedsDataType data_type, const char *str) {
#ifdef TELEMETRY_ENABLED
  if (!str) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result =
      seds_router_log_bytes_ex(g_router.r, data_type, (const uint8_t *)str, strlen(str), NULL, 1);

  return result;
#else
  (void)data_type;
  (void)str;
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_string_asynchronous(SedsDataType data_type, const char *str) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_string_asynchronous_locked(data_type, str);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


SedsResult log_telemetry_queued_u8(SedsDataType data_type, const uint8_t *data,
                                   size_t element_count) {
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U) return SEDS_BAD_ARG;
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
  SedsResult result = SEDS_ERR;
  if (g_router.r || init_telemetry_router() == SEDS_OK) {
    result = seds_router_log_queue_typed(g_router.r, data_type, data, element_count,
                                         sizeof(uint8_t), SEDS_EK_UNSIGNED);
  }
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
  return result;
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count);
  return SEDS_OK;
#endif
}


static SedsResult dispatch_tx_queue_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result = seds_router_process_tx_queue(g_router.r);

  return result;
#endif
}

SedsResult dispatch_tx_queue(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = dispatch_tx_queue_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_rx_queue_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result = seds_router_process_rx_queue(g_router.r);

  return result;
#endif
}

SedsResult process_rx_queue(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_rx_queue_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult dispatch_tx_queue_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result =
      seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);

  return result;
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = dispatch_tx_queue_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_rx_queue_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result =
      seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);

  return result;
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_rx_queue_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_all_queues_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  const SedsResult result =
      seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);

  return result;
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_all_queues_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


#ifdef TELEMETRY_ENABLED
static SedsResult log_error_impl_locked(uint8_t queue, const char *fmt, va_list args) {
  char buf[513];
  if (!fmt) return SEDS_BAD_ARG;
  if (!g_router.r && init_telemetry_router() != SEDS_OK) return SEDS_ERR;

  const int written = vsnprintf(buf, sizeof(buf), fmt, args);
  /* snprintf reports the untruncated length. Never publish bytes beyond buf. */
  const size_t len = written < 0 ? 0U :
      ((size_t)written < sizeof(buf) ? (size_t)written : sizeof(buf) - 1U);

  const SedsResult result = seds_router_log_bytes_ex(
      g_router.r, SEDS_DT_TELEMETRY_ERROR, (const uint8_t *)buf, len, NULL, queue);

  return result;
}

static SedsResult log_error_impl(uint8_t queue, const char *fmt, va_list args) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_error_impl_locked(queue, fmt, args);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


#endif

SedsResult log_error_asynchronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_synchronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) {
    return (SedsResult)need;
  }

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
#ifdef USB_ENUMERATES
    printf("Error: %s\r\n", buf);
#endif
  } else {
    (void)log_error_asynchronous("Error: seds_error_to_string failed: %d\r\n", (int)res);
  }

  return res;
#endif
}

void die(const char *fmt, ...) {
  char buf[128];
  va_list args;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  while (1) {
#ifdef USB_ENUMERATES
    printf("FATAL: %s\r\n", buf);
#endif
    HAL_Delay(1000);
  }
}

#endif /* TELEMETRY_ENABLED */


/* Thread */

TX_THREAD telemetry_task;
TX_MUTEX telemetry_mu;
TX_BYTE_POOL telemetry_pool;
extern TX_THREAD fx_app_thread;

static cm_align CHAR static_pool[TELEMETRY_HEAP];
#ifdef TELEMETRY_USE_TLSF
static TX_BYTE_POOL telemetry_extra_pool;
static uncached CHAR telemetry_extra_storage[TELEMETRY_EXTRA_HEAP];
#endif

/* From Launch on, republish the current flight state on a fixed cadence.
 * Each transition report is a one-shot ordered managed value, and the SDK
 * fails an ordered stream's queued tail when one report exhausts its
 * end-to-end retries (any contracted recipient's receipt is late). In flight
 * that dropped Ascent, Coast, Descent and Landed at the ground station, which
 * then never saw the FC-owned Landed. The repeat only restates the FC's own
 * current state; boards ignore non-station reports and the GS acts on change. */
#ifndef FC_STATE_REPUBLISH_MS
#define FC_STATE_REPUBLISH_MS 2000U
#endif
volatile uint32_t g_flight_state_republished __attribute__((used, externally_visible)) = 0U;

static void telemetry_republish_flight_state(void)
{
#ifdef TELEMETRY_ENABLED
  static uint32_t last_ms;
  static bool started;
  const state local = current();
  if (local < Launch || local > Recovery) return;
  const uint32_t now_ms = HAL_GetTick();
  if (started && (uint32_t)(now_ms - last_ms) < FC_STATE_REPUBLISH_MS) return;
  started = true;
  last_ms = now_ms;
  const uint8_t value = (uint8_t)global_state_of(local);
  if (log_telemetry_queued_u8(SEDS_DT_FLIGHT_STATE, &value, 1U) == SEDS_OK)
    g_flight_state_republished++;
#endif
}

void telemetry_entry(ULONG _)
{
  g_telemetry_thread_entered++;
  g_telemetry_service_stage = 1U;
#ifdef FAKESTATION

  tx_thread_sleep(TLMT_TIME_SLICE * 10);
  return emulate_handler_caller();

#else

  can_bus_init(&hfdcan1);
  g_telemetry_service_stage = 2U;

  // Ensure router exists early (so we can send requests immediately)
  while (init_telemetry_router() != SEDS_OK)
  {
    /* Construction owns its 500 ms retry deadline. Keep hardware RX and the
     * watchdog live while unavailable; no flight/output command is issued. */
    can_bus_process_rx();
    board_watchdog_progress(BOARD_WATCHDOG_NETWORK);
    tx_thread_sleep(1U);
  }
  g_telemetry_service_stage = 3U;

  /* Prime the normal constrained-link discovery path after CAN and the router
   * are ready. The driver streams announcements through the three-entry H5
   * FIFO while preserving normal periodic discovery and schema-on-demand. */
  if (telemetry_poll_discovery() != SEDS_OK)
  {
    g_telemetry_queue_errors++;
  }
  g_telemetry_service_stage = 4U;
  telemetry_record_queue_result(
      process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS));
  g_telemetry_service_stage = 5U;

  /* Remove the temporary startup indication without overwriting the managed
   * underglow value restored from LaunchCore persistent storage. */
  av_bay_underglow_reapply();
  led_on(light[Green].port, light[Green].pin);

  MrAnalog (WE_ARE_SO_BACK)
    {
    g_telemetry_service_stage = 6U;
    can_bus_process_rx();
    g_telemetry_service_stage = 61U;
    if (telemetry_poll_discovery() != SEDS_OK)
      g_telemetry_discovery_poll_errors++;
    g_telemetry_service_stage = 62U;
    SedsResult k =
        process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS);
    g_telemetry_service_stage = 63U;

    telemetry_record_queue_result(k);
    telemetry_republish_flight_state();

    if (telemetry_poll_timesync() != SEDS_OK)
      g_telemetry_timesync_poll_errors++;

    g_telemetry_service_stage = 64U;
    telemetry_sample_stack_margin();
    g_telemetry_loop_completions++;
    board_watchdog_progress(BOARD_WATCHDOG_NETWORK);

    tx_thread_relinquish();
  }

#endif /* FAKESTATION */
}

UINT create_telemetry_task(TX_BYTE_POOL *shared_pool)
{
  UINT st;
  CHAR *ptr;

  st = tx_byte_pool_create(&telemetry_pool, id "bp",
                           static_pool, TELEMETRY_HEAP);

  if (st != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  st = tx_byte_allocate(shared_pool, (VOID**) &ptr,
                        TLMT_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* The telemetry thread enters SEDSNet immediately and its allocator hooks
   * take this mutex. Create it before making the thread runnable; creating an
   * auto-start thread first allowed a boot-time lock/unlock on an uncreated
   * ThreadX object. */
  st = tx_mutex_create(&telemetry_mu, id "mu", TX_INHERIT);
  if (st != TX_SUCCESS)
  {
    return st;
  }

#ifdef TELEMETRY_USE_TLSF
  telemetry_tlsf_register_pool(&telemetry_pool);
  st = tx_byte_pool_create(&telemetry_extra_pool, "TE reserve",
                           telemetry_extra_storage, sizeof(telemetry_extra_storage));
  if (st != TX_SUCCESS) return st;
  telemetry_tlsf_register_pool(&telemetry_extra_pool);
  try_allocate_reserve_pool();
#endif

  st = tx_thread_create(&telemetry_task,
                        "Telemetry Task",
                        telemetry_entry,
                        TLMT_INPUT,
                        ptr,
                        TLMT_STACK_BYTES,
                        /* No preemption threshold */
                        TLMT_PRIORITY,
                        TLMT_PRIORITY,
                        TLMT_TIME_SLICE,
                        TX_AUTO_START);

  if (st != TX_SUCCESS)
  {
    return st;
  }
                                
  return st;
}

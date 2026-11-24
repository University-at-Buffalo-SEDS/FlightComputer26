#include "board_health.h"
#include "av_bay_underglow.h"
#include "flight_buzzer.h"
#include "fc_estimator_status.h"
#include "ground_test.h"
#include "fc_sd_events.h"
#include "board_recovery_transport.h"
#include "board_temperature.h"
#include "board_uptime.h"
#include "board_watchdog.h"
#include "telemetry.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#define BOARD_HEALTH_FC_OUTPUTS_TAG (0x4F55U << 16U)
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Board diagnostic words require little-endian encoding"
#endif
#ifdef TELEMETRY_USE_TLSF
extern volatile uint32_t g_telemetry_tlsf_live_bytes, g_telemetry_tlsf_peak_bytes, g_telemetry_tlsf_region_bytes, g_telemetry_tlsf_failures;
#endif
extern uint32_t board_health_can_rx_drops(void);
extern int telemetry_router_enter(void);
extern int telemetry_router_leave(void);
#ifdef SD_AVAILABLE
extern volatile uint32_t g_sd_enqueued_records, g_sd_written_records, g_sd_flushed_records, g_sd_format_errors, g_sd_lock_contention_drops, g_sd_unavailable_drops, g_sd_shutdown_lost_records, g_sd_write_errors, g_sd_max_write_ms;
#endif

void board_health_poll(SedsRouter *router)
{
#ifdef TELEMETRY_ENABLED
  static uint32_t last_report_ms, last_sent_ms;
  static uint32_t failures;
  static uint8_t sent_phase, sent_frozen;
#if FC_GROUND_TEST
  static uint32_t sent_igt;
  const uint32_t igt = igt_health_bits();
#else
  enum { sent_igt = 0, igt = 0 };
#endif
  if (!router) return;
  const uint64_t uptime = board_uptime_ticks();
  const uint32_t now = (uint32_t)uptime;
  /* An estimator phase change (Launch starting the Kalman filter) is
   * reported at once, at most every 250 ms, and repeated 1 s later. */
  const uint8_t phase = g_fc_est_phase;
  /* So is the barometer ground reference freezing at Launch. */
  const uint32_t word15 = FC_EST_WORD15(g_fc_est_start, baro_ref_bits());
  const uint8_t frozen = FC_EST_BARO_FROZEN(word15);
  const bool changed = (phase != sent_phase || frozen != sent_frozen ||
                        igt != sent_igt) &&
                       (uint32_t)(now - last_sent_ms) >= 250U;
  if (!changed && (uint32_t)(now - last_report_ms) < 5000U) return;
  last_report_ms = changed ? now - 4000U : now;
  last_sent_ms = now;
  sent_phase = phase;
  sent_frozen = frozen;
#if FC_GROUND_TEST
  sent_igt = igt;
#endif
  board_temperature_poll();

  uint32_t words[24] = {1U, 2U, (uint32_t)uptime, (uint32_t)(uptime >> 32U), BOARD_UPTIME_FREQUENCY_HZ};
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  words[5] = g_watchdog_reset_flags; words[6] = g_watchdog_missing_mask;
#ifdef TELEMETRY_USE_TLSF
  /* Exact existing occupied-byte counters. Avoid interrupt-disabled heap
   * walks, and do not mislabel occupied headroom as contiguous free space. */
  words[1] |= 1U;
  words[7] = g_telemetry_tlsf_live_bytes; words[8] = g_telemetry_tlsf_peak_bytes;
  words[9] = g_telemetry_tlsf_region_bytes; words[10] = g_telemetry_tlsf_failures;
#endif
  words[12] = board_health_can_rx_drops();
  /* The cosmetic outputs this FC applied, so the station can confirm a write
   * whose end-to-end ACK was lost. Word 14 is the UART-drop counter on other
   * boards only with word 1 flag 8, which this FC never sets; the tag in bits
   * 16..31 marks this use. Bits: 0 underglow on, 1 buzzer on, 2 the underglow
   * value came from a network write this boot, 3 likewise for the buzzer. */
  words[14] = BOARD_HEALTH_FC_OUTPUTS_TAG |
              (g_av_bay_underglow_enabled != 0U ? 1U : 0U) |
              (g_flight_buzzer_enabled != 0U ? 2U : 0U) |
              (g_av_bay_underglow_network_seen != 0U ? 4U : 0U) |
              (g_flight_buzzer_network_seen != 0U ? 8U : 0U) |
              igt; /* bits 4..15: integrated ground test (ground_test.h) */
  words[13] = FC_EST_WORD13(phase, g_fc_est_cycles);
  words[15] = word15;
#ifdef SD_AVAILABLE
  words[1] |= 16U;
  words[16] = g_sd_enqueued_records; words[17] = g_sd_written_records;
  words[18] = g_sd_format_errors + g_sd_lock_contention_drops + g_sd_unavailable_drops + g_sd_shutdown_lost_records;
  words[19] = g_sd_flushed_records; words[20] = g_sd_write_errors;
  words[1] |= 128U; words[21] = g_sd_max_write_ms;
#endif
  __set_PRIMASK(saved);
  /* Why flight commands are refused (board_recovery_transport.c). */
  words[6] |= board_recovery_controller_gate() << 24U;
  words[6] |= (uint32_t)g_fc_sd_state << 30U; /* SD not ready, retry pending */
#ifdef FC_SD_SERVICE
  words[6] |= 1U << 29U; /* this image serves the SD read service (P2P 4511) */
#endif
  words[11] = (uint32_t)seds_memory_admission_rejected();
  words[23] = failures;
  words[1] |= board_temperature_health_flags();
  if (!telemetry_router_enter()) { ++failures; return; }
  /* Network time quality: SEDSnet health byte in word 22, tagged "SY" (word 6
   * bits 24..31 carry the recovery gate on FC). */
  {
    SedsTimeSyncStatus ts;
    if (seds_router_get_timesync_status(router, &ts) == SEDS_OK) {
      /* Bits 8..15: stratum (1 = the GS, 2 = RF re-serving it over CAN). */
      words[22] = (0x5359U << 16U) | ((uint32_t)ts.stratum << 8U) | ts.health_byte;
    }
  }
  /* Static UInt32 schema + exact little-endian bytes: use the existing
   * bounded byte path without pulling every generic numeric converter in. */
  const SedsResult result = seds_router_log_bytes_ex(router, SEDS_DT_BOARD_HEALTH,
      (const uint8_t *)words, sizeof(words), NULL, true);
  const int released = telemetry_router_leave();
  if (result != SEDS_OK || !released) ++failures;
#else
  (void)router;
#endif
}

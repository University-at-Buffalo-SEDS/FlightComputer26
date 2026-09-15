#ifndef FC_TELEMETRY_RATE_H
#define FC_TELEMETRY_RATE_H

#include <stdint.h>
#include <stdbool.h>

/* Board-local build setting: whole Hz, 1..1000. Override with -D at build
 * time. This is deliberately not a SEDSNet network variable. */
#ifndef FC_TELEMETRY_RATE_HZ
#define FC_TELEMETRY_RATE_HZ 5U
#endif
#if FC_TELEMETRY_RATE_HZ < 1 || FC_TELEMETRY_RATE_HZ > 1000
#error "FC_TELEMETRY_RATE_HZ must be a whole number from 1 to 1000"
#endif

/* Shared by producer scheduling and the per-type publication limiter. */
#define FC_TELEMETRY_PERIOD_MS \
  ((1000U + FC_TELEMETRY_RATE_HZ / 2U) / FC_TELEMETRY_RATE_HZ)

uint32_t fc_telemetry_period_ms(void);
bool fc_telemetry_rate_allow(uint32_t data_type, uint32_t now_ms);

#endif

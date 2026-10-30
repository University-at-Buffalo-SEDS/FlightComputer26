#ifndef GROUND_HEARTBEAT_OWNER_H
#define GROUND_HEARTBEAT_OWNER_H
#include <stddef.h>
/* Logical source identity, preserved by RRLS forwarding. This is an ownership
 * check, not cryptographic authentication. No physical-hop identity is trusted. */
#ifndef GROUND_HEARTBEAT_OWNER_NAME
#define GROUND_HEARTBEAT_OWNER_NAME "GS"
#endif
#ifndef GROUND_HEARTBEAT_OWNER_COMPACT
#define GROUND_HEARTBEAT_OWNER_COMPACT "@addr:3561204631"
#endif
static inline int ground_heartbeat_name_equal(const char *sender, size_t len,
                                               const char *expected, size_t count) {
  if (sender == NULL || len != count) return 0;
  for (size_t i = 0; i < count; ++i) if (sender[i] != expected[i]) return 0;
  return 1;
}
static inline int ground_heartbeat_owner_matches(const char *sender, size_t len) {
  return ground_heartbeat_name_equal(sender, len, GROUND_HEARTBEAT_OWNER_NAME,
      sizeof(GROUND_HEARTBEAT_OWNER_NAME) - 1U) ||
      ground_heartbeat_name_equal(sender, len, GROUND_HEARTBEAT_OWNER_COMPACT,
      sizeof(GROUND_HEARTBEAT_OWNER_COMPACT) - 1U);
}
#endif

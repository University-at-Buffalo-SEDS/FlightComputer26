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
/* The RRLS that takes over from a silent GS keeps the ground link alive. */
#ifndef GROUND_HEARTBEAT_TAKEOVER_NAME
#define GROUND_HEARTBEAT_TAKEOVER_NAME "RRLS"
#endif
#ifndef GROUND_HEARTBEAT_TAKEOVER_COMPACT
#define GROUND_HEARTBEAT_TAKEOVER_COMPACT "@addr:1895946653"
#endif
#define GROUND_HEARTBEAT_IS(sender, len, name) \
  ground_heartbeat_name_equal((sender), (len), (name), sizeof(name) - 1U)
static inline int ground_heartbeat_owner_matches(const char *sender, size_t len) {
  return GROUND_HEARTBEAT_IS(sender, len, GROUND_HEARTBEAT_OWNER_NAME) ||
         GROUND_HEARTBEAT_IS(sender, len, GROUND_HEARTBEAT_OWNER_COMPACT) ||
         GROUND_HEARTBEAT_IS(sender, len, GROUND_HEARTBEAT_TAKEOVER_NAME) ||
         GROUND_HEARTBEAT_IS(sender, len, GROUND_HEARTBEAT_TAKEOVER_COMPACT);
}
#endif

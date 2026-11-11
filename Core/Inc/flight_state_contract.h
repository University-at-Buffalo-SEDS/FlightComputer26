#ifndef FLIGHT_STATE_CONTRACT_H
#define FLIGHT_STATE_CONTRACT_H
#include <stdint.h>
/* Network FLIGHT_STATE set (a): 0 Startup, 1 Idle, 2 Fill, 6 Armed, 7 Launch,
 * 13 Landed, 15 Aborted, 16 InFlight (fcstructs.h gnd_state numbering).
 * A received retired value maps to the nearest one (3-5 Fill, 8-12 InFlight,
 * 14 Landed); -1 for a value outside the enumeration. */
static inline int network_flight_state(uint8_t value)
{
  if (value >= 3U && value <= 5U) return 2;
  if (value >= 8U && value <= 12U) return 16;
  if (value == 14U) return 13;
  return value <= 16U ? (int)value : -1;
}
#endif

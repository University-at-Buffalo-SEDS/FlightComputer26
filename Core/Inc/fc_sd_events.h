#ifndef FC_SD_EVENTS_H
#define FC_SD_EVENTS_H
#include <stdbool.h>
#include <stdint.h>
/* A GPIO audit, not proof of ignition or parachute deployment. Capture only
 * bounded POD records here; the SD task formats them and queries UTC later. */
typedef enum {
    FC_LOG_LAUNCH_DETECTED,
    FC_LOG_PYRO_HIGH,
    FC_LOG_PYRO_LOW,
    FC_LOG_PYRO_REJECTED,
    FC_LOG_PYRO_ACTIVE,
    FC_LOG_LAUNCH_ACCEPTED
} fc_sd_event_kind;
typedef struct { uint32_t tick; uint8_t kind, reef; } fc_sd_event;
extern volatile uint32_t g_sd_event_pending;
/* Authoritative state transition; monotonic milliseconds, no I/O. */
void sd_launch_landed(uint32_t tick);
void fc_sd_capture_event(fc_sd_event_kind kind, bool reef);
bool fc_sd_peek_event(fc_sd_event *event);
void fc_sd_consume_event(void);
#endif

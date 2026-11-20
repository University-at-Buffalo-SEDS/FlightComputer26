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
    FC_LOG_LAUNCH_ACCEPTED,
    FC_LOG_PYRO_DRY_RUN
} fc_sd_event_kind;
typedef struct { uint32_t tick; uint8_t kind, reef; } fc_sd_event;
extern volatile uint32_t g_sd_event_pending;
/* Authoritative state transition; monotonic milliseconds, no I/O. */
void sd_launch_landed(uint32_t tick);
void fc_sd_capture_event(fc_sd_event_kind kind, bool reef);
/* BOARD_HEALTH word 6 bit 30 / 31: the card is not mounted or the log file
 * cannot be opened; an operator SD retry is pending. */
#define FC_SD_NOT_READY     1U
#define FC_SD_RETRY_PENDING 2U
extern volatile uint8_t g_fc_sd_state;
bool fc_sd_peek_event(fc_sd_event *event);
void fc_sd_consume_event(void);
#endif

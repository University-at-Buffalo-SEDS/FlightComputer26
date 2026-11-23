#ifndef SD_FILE_STREAM_H
#define SD_FILE_STREAM_H
/* FC SD transfer service v2 (flight image only, FC_SD_SERVICE). The router
 * callback queues one LIST/OPEN/HASH request in a RAM slot, applies ACKs and
 * wakes the SD thread; the SD thread answers requests and builds DATA
 * messages (sd_xfer.c) from the card; the telemetry thread sends answers and
 * paces DATA out. */
#include "sd_file_wire.h"
#include "sd_xfer.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* SD thread side. Claim moves a pending request to busy; answer publishes a
 * frame only if the stream session that asked is still current. The borrowed
 * area (512-byte block, read handle, transfer state and workspace) exists
 * only while a session or an open handle does. */
bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket);
bool sd_file_stream_ticket_current(uint32_t ticket);
uint8_t *sd_file_stream_block(void);
void *sd_file_stream_handle(void);
void *sd_file_stream_scratch(void);          /* SHA-256 context while hashing */
void sd_file_stream_handle_open(bool open);
void sd_file_stream_answer(uint32_t ticket, const uint8_t *frame, size_t len);
/* OPEN: starts the transfer under the lock and answers it. */
void sd_file_stream_open(uint32_t ticket, const sd_xfer_open_t *open, uint32_t file_size, bool growing);
bool sd_file_stream_transfer_active(void);
/* Builds the next DATA message if one is due and the send slot is free;
 * true if it produced one. A read error ends the transfer. */
bool sd_file_stream_produce(sd_xfer_read_fn read, void *ctx, uint32_t now_ms);
bool sd_file_stream_session_active(void);
/* SD thread: the card was remounted fresh; ends the transfer, any queued
 * answer or busy request, and has the telemetry thread reset the session. */
void sd_file_stream_media_remounted(void);
/* Shared gate: local state and network FLIGHT_STATE (sd_file_fc_state_allowed). */
bool sd_file_stream_state_allowed(void);

/* Provided by sdpipeline.c: one bounded service step; wakes the SD thread. */
void sd_file_service_turn(void);
void sd_pipeline_wake(void);

#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
SedsResult sd_file_stream_init(SedsRouter *router);
/* Telemetry thread, router held: at most one frame per call. */
void sd_file_stream_poll(void);
#endif
#endif

#ifndef SD_FILE_STREAM_H
#define SD_FILE_STREAM_H
/* FC SD block-read service (flight image only, FC_SD_SERVICE). The router
 * callback copies one validated request into a single RAM slot and wakes the
 * SD thread; the SD thread performs one bounded read-only FileX operation for
 * it; the telemetry thread paces the answer out one frame at a time. */
#include "sd_file_wire.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t status;
    uint8_t flags;          /* LIST/READ: SD_FILE_ENTRY_OPEN */
    uint8_t name_len;       /* LIST: name bytes at the start of the block */
    uint16_t data_len;      /* READ: bytes in the block */
    uint16_t data_start;    /* READ: first valid byte in the block */
    uint32_t file_size;
    uint32_t offset;        /* READ: absolute offset of the first data byte */
    uint32_t crc;           /* READ: CRC-32 of the data bytes */
} sd_file_result_t;

/* SD thread side. Claim moves a pending request to busy; finish publishes the
 * answer only if the stream session that asked is still current. The block
 * (512 B) and read handle exist only while a session or an open handle does. */
bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket);
uint8_t *sd_file_stream_block(void);
void *sd_file_stream_handle(void);
void sd_file_stream_handle_open(bool open);
void sd_file_stream_finish(uint32_t ticket, const sd_file_result_t *result);
bool sd_file_stream_session_active(void);
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

/*
 * SD logger configuration and API.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stddef.h>
#include <stdint.h>

#include "app_threadx.h"
#include "tx_api.h"
#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TX_THREAD g_sd_log_thread;


/* ------ Local definitions ------ */

typedef VOID (*SdFxDriverEntry)(FX_MEDIA *media);

#define LOGGER_INPUT 0
#define LOGGER_PRIORITY 5
#define LOGGER_TIME_SLICE	5
#define LOGGER_STACK_BYTES 8192
#define LOGGER_STACK_ULONG (LOGGER_STACK_BYTES / sizeof(ULONG))

#define SD_LOG_QUEUE_DEPTH 64
#define SD_LOG_LINE_MAX 256


/* ------ Public API ------ */

/**
 * Initialize SD logger.
 *
 * @param filename       File to append to (e.g. "seds_log.txt").
 * @param driver_entry   FileX driver entry function (e.g. fx_stm32_sd_driver).
 * @param driver_info    Passed to fx_media_open driver_info (often 0 or &hsd1).
 *
 * Safe to call multiple times; only first call creates thread/queue.
 */
UINT sd_logger_init(const CHAR *filename, SdFxDriverEntry driver_entry, VOID *driver_info);

/**
 * Enqueue a line to be appended (non-blocking).
 *
 * - Copies up to internal max (256 incl CRLF).
 * - Returns FX_NO_MORE_SPACE / FX_BUFFER_ERROR style codes on saturation.
 */
UINT sd_logger_enqueue_line(const CHAR *data, size_t len);

/**
 * Optional: force a flush soon (thread will flush when it can).
 * If you don’t need it, you can omit using it.
 */
UINT sd_logger_request_flush(void);

#ifdef __cplusplus
}
#endif


#endif // LOGGER_H
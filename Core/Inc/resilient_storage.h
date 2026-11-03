#ifndef RESILIENT_STORAGE_H
#define RESILIENT_STORAGE_H

#include "main.h"
#include <stdbool.h>
#include "fx_api.h"

/* Sole storage owner only; no FileX call may be active at retry/quiescence. */
void flight_sd_note_io_failure(UINT status);
UINT flight_sd_prepare_io(void);
/* False means FileX success still has unresolved actual driver evidence. */
bool flight_sd_note_progress(void);
/* No readiness/status mutation; fence a following FileX operation. */
bool flight_sd_io_verified(void);
UINT flight_sd_dma_prepare(void);
void flight_sd_dma_release(void);
UINT flight_sd_dma_drain(void);

HAL_StatusTypeDef flight_sd_init(SD_HandleTypeDef *sd);
UINT flight_fx_media_open(FX_MEDIA *media_ptr, CHAR *media_name,
                          VOID (*media_driver)(FX_MEDIA *), VOID *driver_info_ptr,
                          VOID *memory_ptr, ULONG memory_size);

#endif

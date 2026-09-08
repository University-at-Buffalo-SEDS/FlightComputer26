#ifndef RESILIENT_STORAGE_H
#define RESILIENT_STORAGE_H

#include "main.h"
#include "fx_api.h"

HAL_StatusTypeDef flight_sd_init(SD_HandleTypeDef *sd);
UINT flight_fx_media_open(FX_MEDIA *media_ptr, CHAR *media_name,
                          VOID (*media_driver)(FX_MEDIA *), VOID *driver_info_ptr,
                          VOID *memory_ptr, ULONG memory_size);

#endif

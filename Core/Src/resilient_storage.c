#include "resilient_storage.h"

#include "tx_api.h"

extern SD_HandleTypeDef hsd1;

/* CubeMX treats a failed card initialization as a fatal board error. Storage
 * is optional at boot on the flight computer, so remember the result while
 * allowing networking and recovery tasks to start. The FileX thread retries
 * after yielding its priority level and supports a card inserted later. */
static volatile uint32_t g_sd_initialized = 0U;
volatile uint32_t g_sd_init_failures
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sd_mount_failures
    __attribute__((used, externally_visible)) = 0U;

HAL_StatusTypeDef flight_sd_init(SD_HandleTypeDef *sd) {
  HAL_StatusTypeDef status = HAL_SD_Init(sd);
  g_sd_initialized = (status == HAL_OK) ? 1U : 0U;
  if (status != HAL_OK) {
    g_sd_init_failures++;
  }

  /* Let CubeMX's generated startup continue. FileX owns retry policy. */
  return HAL_OK;
}

UINT flight_fx_media_open(FX_MEDIA *media_ptr, CHAR *media_name,
                          VOID (*media_driver)(FX_MEDIA *), VOID *driver_info_ptr,
                          VOID *memory_ptr, ULONG memory_size) {
  for (;;) {
    if (g_sd_initialized == 0U) {
      HAL_StatusTypeDef status = HAL_SD_Init(&hsd1);
      g_sd_initialized = (status == HAL_OK) ? 1U : 0U;
      if (status != HAL_OK) {
        g_sd_init_failures++;
      }
    }

    if (g_sd_initialized != 0U) {
      UINT result = fx_media_open(media_ptr, media_name, media_driver,
                                  driver_info_ptr, memory_ptr, memory_size);
      if (result == FX_SUCCESS) {
        return result;
      }
      g_sd_initialized = 0U;
      g_sd_mount_failures++;
    }

    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

#include "resilient_storage.h"

#ifdef TELEMETRY_ENABLED
#include "telemetry.h"
#else
#include <stdio.h>
#endif
#include "tx_api.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;
/* One mounted card owns all retained FileX cache, handles and pending offsets.
 * No allocation, remount or producer-side recovery is permitted. */
static uint32_t mounted_cid[4], mounted_blocks, retry_at;
static VOID (*source_driver)(FX_MEDIA *);
static uint8_t driver_io_failed;
static uint8_t mounted_identity, transport_lost, retry_started, dma_quiesced;
volatile uint32_t g_sd_transport_retries, g_sd_card_identity_mismatches;
volatile uint32_t g_sd_runtime_failures, g_sd_last_io_status;

static UINT quiesce_transport(void)
{
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  hsd1.Instance->MASK = 0U;
  hsd1.Instance->IDMACTRL = 0U;
  hsd1.Instance->DCTRL = 0U;
  __DSB();
  (void)HAL_SD_DeInit(&hsd1);
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __DSB();
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  HAL_NVIC_ClearPendingIRQ(SDMMC1_IRQn);
  __DSB();
  const UINT status = flight_sd_dma_drain();
  if (status == FX_SUCCESS) driver_io_failed = 0U;
  return status;
}


/* CubeMX treats a failed card initialization as a fatal board error. Storage
 * is optional at boot on the flight computer, so remember the result while
 * allowing networking and recovery tasks to start. The FileX thread retries
 * after yielding its priority level and supports a card inserted later. */
static volatile uint32_t g_sd_initialized = 0U;
volatile uint32_t g_sd_init_failures
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sd_mount_failures
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sd_warning_publish_count
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_sd_ready
    __attribute__((used, externally_visible)) = 0U;
/* FileX can map the original driver error to BOOT/FAT errors, and structural
 * failures also synthesize IO during UNINIT. Only actual data/boot requests
 * carry transport evidence. This sole-owner wrapper never wraps itself. */
static void flight_sd_media_driver(FX_MEDIA *media)
{
  const UINT request = media->fx_media_driver_request;
  source_driver(media);
  if ((request == FX_DRIVER_READ || request == FX_DRIVER_WRITE ||
       request == FX_DRIVER_BOOT_READ || request == FX_DRIVER_BOOT_WRITE) &&
      media->fx_media_driver_status == FX_IO_ERROR) {
    driver_io_failed = 1U;
    g_sd_ready = 0U;
  }
}
/* A failed synchronous FileX call may have left IDMA or an old completion
 * live. Retained static buffers remain owned until this bounded quiescence. */
void flight_sd_note_io_failure(UINT status)
{
  g_sd_ready = 0U;
  g_sd_last_io_status = status;
  ++g_sd_runtime_failures;
  if (status == FX_IO_ERROR || driver_io_failed) {
    transport_lost = 1U;
    dma_quiesced = quiesce_transport() == FX_SUCCESS;
  }
}
bool flight_sd_io_verified(void) { return !transport_lost && !driver_io_failed; }
bool flight_sd_note_progress(void)
{
  /* Original FileX success is distinct from verified transport progress. */
  g_sd_last_io_status = FX_SUCCESS;
  if (transport_lost || driver_io_failed) { g_sd_ready = 0U; return false; }
  g_sd_ready = 1U;
  return true;
}
UINT flight_sd_prepare_io(void)
{
  if (driver_io_failed && !transport_lost) {
    /* FileX may have retried internally and returned success. Do not erase
     * its original status/counters; settle actual DMA before any next I/O. */
    transport_lost = 1U;
    dma_quiesced = quiesce_transport() == FX_SUCCESS;
  }
  if (!transport_lost) return FX_SUCCESS;
  const uint32_t now = (uint32_t)tx_time_get();
  if (retry_started && (uint32_t)(now - retry_at) < TX_TIMER_TICKS_PER_SECOND)
    return FX_IO_ERROR;
  retry_started = 1U; retry_at = now; ++g_sd_transport_retries;
  if (!dma_quiesced && quiesce_transport() != FX_SUCCESS) return FX_IO_ERROR;
  dma_quiesced = 1U;
  /* HAL_SD_Init -> HAL_SD_InitCard -> SD_PowerON sends CMD0 GO_IDLE_STATE,
   * resetting interrupted card commands as well as the peripheral state. */
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
    ++g_sd_init_failures;
    dma_quiesced = quiesce_transport() == FX_SUCCESS;
    return FX_IO_ERROR;
  }
  dma_quiesced = 0U;
  if (mounted_identity &&
      (memcmp(mounted_cid, hsd1.CID, sizeof mounted_cid) != 0 ||
       mounted_blocks != hsd1.SdCard.LogBlockNbr)) {
    ++g_sd_card_identity_mismatches;
    dma_quiesced = quiesce_transport() == FX_SUCCESS;
    return FX_IO_ERROR;
  }
  transport_lost = 0U;
  /* Controller initialization is not successful record write/flush progress. */
  return FX_SUCCESS;
}

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
  /* One application media owner supplies the original driver on every mount.
   * Preserve NULL for FileX validation instead of replacing it with a wrapper. */
  if (media_driver != flight_sd_media_driver) source_driver = media_driver;
  uint32_t failed_retries = 0U;
  for (;;) {
    if (g_sd_initialized == 0U) {
      if (transport_lost && !dma_quiesced) dma_quiesced = quiesce_transport() == FX_SUCCESS;
      HAL_StatusTypeDef status = transport_lost && !dma_quiesced ? HAL_ERROR : HAL_SD_Init(&hsd1);
      g_sd_initialized = (status == HAL_OK) ? 1U : 0U;
      if (transport_lost) dma_quiesced = 0U;
      if (status != HAL_OK) {
        g_sd_init_failures++;
        if (transport_lost) dma_quiesced = quiesce_transport() == FX_SUCCESS;
      }
    }

    if (g_sd_initialized != 0U) {
      UINT result = fx_media_open(media_ptr, media_name, media_driver ? flight_sd_media_driver : NULL,
                                  driver_info_ptr, memory_ptr, memory_size);
      if (result == FX_SUCCESS) {
        memcpy(mounted_cid, hsd1.CID, sizeof mounted_cid);
        mounted_blocks = hsd1.SdCard.LogBlockNbr; mounted_identity = 1U;
        transport_lost = 0U; retry_started = 0U;
        g_sd_ready = driver_io_failed ? 0U : 1U;
        return result;
      }
      flight_sd_note_io_failure(result);
      g_sd_initialized = 0U;
      g_sd_mount_failures++;
    }

    if ((failed_retries == 0U) || ((failed_retries % 60U) == 0U)) {
#ifdef TELEMETRY_ENABLED
      if (log_telemetry_string_asynchronous(
              SEDS_DT_WARNING,
              "SD unavailable; flight/network continue") == SEDS_OK) {
        g_sd_warning_publish_count++;
      }
#else
      fprintf(stderr, "Flight Computer SD card unavailable; retrying\n");
#endif
    }
    failed_retries++;

    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

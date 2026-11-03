#include <stdlib.h>
#define BOARD_DELTA_SIZE 8192U
#define BOARD_FLASH_WRITE_ALIGNMENT 16U
#define OTA_STREAM_PORT 4510U
#define OTA_STREAM_MAX_CHUNK 112U
typedef int SedsResult; typedef int SedsRouter;
typedef int launchcore_storage_driver_t; typedef int launchcore_status_t;
typedef int launchcore_storage_status_t; typedef int ota_stream_status_t;
enum {SEDS_OK,SEDS_BAD_ARG,SEDS_ERR};
enum {LAUNCHCORE_OK,LAUNCHCORE_ERR_NO_SPACE,LAUNCHCORE_ERR_BAD_STATE,LAUNCHCORE_ERR_BAD_IMAGE,LAUNCHCORE_ERR_VERIFY,LAUNCHCORE_ERR_FLASH,LAUNCHCORE_ERR_METADATA};
enum {OTA_STREAM_OK,OTA_STREAM_BAD_MESSAGE,OTA_STREAM_BAD_STATE,OTA_STREAM_BAD_OFFSET,OTA_STREAM_NO_SPACE,OTA_STREAM_STORAGE_ERROR,OTA_STREAM_BAD_IMAGE,OTA_STREAM_INTERNAL_ERROR};
enum {SEDS_P2P_STREAM_ACCEPTED,SEDS_P2P_STREAM_DATA,SEDS_P2P_STREAM_CLOSED,SEDS_P2P_STREAM_RESET};
#define LAUNCHCORE_STORAGE_OK 0
// Existing repo OTA test C event facade: only used fields; real SDK provenance
// intentionally not fabricated. SDK transport itself is not exercised here.
typedef struct {int kind;uint32_t stream_id;const uint8_t *payload;size_t payload_len;} SedsP2pStreamEventView;
const launchcore_storage_driver_t launchcore_board_storage_driver=1;
static bool ota_store_locked,ota_fail_leave;
static void (*during_finish)(void);
static uint64_t ota_now;
static unsigned ota_nvics,ota_reset_store_held,ota_reset_irq_held,ota_flash_calls;
static bool persistent_store_enter(void){assert(!ota_store_locked);ota_store_locked=true;return true;}
static bool persistent_store_leave(void){assert(ota_store_locked);ota_store_locked=false;return !ota_fail_leave;}
static void launchcore_storage_set_driver(const launchcore_storage_driver_t *d){assert(d&&ota_store_locked);}
static int launchcore_storage_init(void){assert(ota_store_locked);return 0;}
static uint64_t telemetry_now_ms(void){assert(!irq);return ota_now;}
static void NVIC_SystemReset(void){assert(irq&&!ota_store_locked);++ota_nvics;ota_reset_store_held+=ota_store_locked;ota_reset_irq_held+=!!irq;}
static int launchcore_delta_update_begin(uint32_t n){assert(ota_store_locked&&n<=BOARD_DELTA_SIZE);++ota_flash_calls;return 0;}
static int launchcore_delta_update_write(const void *p,size_t n){assert(ota_store_locked&&p&&n);++ota_flash_calls;return 0;}
static int launchcore_delta_update_finish(void){assert(ota_store_locked);++ota_flash_calls;if(during_finish)during_finish();return 0;}
static int launchcore_delta_update_abort(void){assert(ota_store_locked);++ota_flash_calls;return 0;}
static int launchcore_confirm_boot(void){assert(ota_store_locked);++ota_flash_calls;return 0;}
static int seds_router_send_p2p_stream(SedsRouter *r,uint32_t id,const void *p,size_t n){assert(r&&id&&p&&n==13&&!ota_store_locked);return 0;}
static int seds_router_bind_p2p_stream_port(SedsRouter *r,unsigned port,SedsResult (*cb)(const SedsP2pStreamEventView*,void*),void *user){assert(r&&port==4510&&cb&&!user&&!ota_store_locked);return 0;}
uint32_t ota_stream_max_patch_size(void);




#include "board_config.h"
#include "launchcore/storage.h"
#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <string.h>
static const launchcore_storage_layout_t layout = {
    .slot_a_base = BOARD_SLOT_A_BASE, .slot_a_size = BOARD_SLOT_A_SIZE,
    .slot_b_base = BOARD_DELTA_BASE, .slot_b_size = BOARD_DELTA_SIZE,
    .slot_b_is_delta = true,
    .metadata0_base = BOARD_METADATA0_BASE, .metadata1_base = BOARD_METADATA1_BASE,
    .metadata_size = LAUNCHCORE_FLASH_ERASE_SIZE,
    .bootloader_base = LAUNCHCORE_INTERNAL_FLASH_BASE,
    .bootloader_size = LAUNCHCORE_BOOTLOADER_SIZE,
    .persistent_data_base = BOARD_PERSIST_BASE, .persistent_data_size = BOARD_PERSIST_SIZE,
    .persistent_data_erase_size = LAUNCHCORE_FLASH_ERASE_SIZE,
    .persistent_data_write_size = BOARD_FLASH_WRITE_ALIGNMENT,
    .slot_erase_size = LAUNCHCORE_FLASH_ERASE_SIZE, .supports_xip = true
};
static bool within(uint32_t a,uint32_t n,uint32_t b,uint32_t s){return n<=s&&a>=b&&a-b<=s-n;}
static bool writable(uint32_t a,uint32_t n){return within(a,n,layout.slot_a_base,layout.slot_a_size)||within(a,n,layout.slot_b_base,layout.slot_b_size)||within(a,n,layout.metadata0_base,layout.metadata_size)||within(a,n,layout.metadata1_base,layout.metadata_size)||within(a,n,layout.persistent_data_base,layout.persistent_data_size);}
static launchcore_storage_status_t init(void){return LAUNCHCORE_STORAGE_OK;}
static void sector(uint32_t a,uint32_t *bank,uint32_t *number){if(a>=FLASH_BASE+FLASH_BANK_SIZE){*bank=FLASH_BANK_2;*number=(a-FLASH_BASE-FLASH_BANK_SIZE)/FLASH_SECTOR_SIZE;}else{*bank=FLASH_BANK_1;*number=(a-FLASH_BASE)/FLASH_SECTOR_SIZE;}}
/* H5 caches reads from internal flash. Disable it before modifying flash;
 * disabling invalidates stale lines. Restore the caller's original state on
 * every exit, including unlock/program failures. */
static launchcore_storage_status_t erase(uint32_t a, uint32_t n)
{
    if (!n || a % FLASH_SECTOR_SIZE || n % FLASH_SECTOR_SIZE || !writable(a, n))
        return LAUNCHCORE_STORAGE_ERR_RANGE;
    const uint32_t cached = HAL_ICACHE_IsEnabled();
    if (cached && HAL_ICACHE_Disable() != HAL_OK) return LAUNCHCORE_STORAGE_ERR_ERASE;
    launchcore_storage_status_t status = LAUNCHCORE_STORAGE_ERR_ERASE;
    if (HAL_FLASH_Unlock() == HAL_OK) {
        status = LAUNCHCORE_STORAGE_OK;
        for (uint32_t p = a; p < a + n; p += FLASH_SECTOR_SIZE) {
            FLASH_EraseInitTypeDef e = {.TypeErase = FLASH_TYPEERASE_SECTORS, .NbSectors = 1};
            uint32_t error;
            sector(p, &e.Banks, &e.Sector);
            if (HAL_FLASHEx_Erase(&e, &error) != HAL_OK) {
                status = LAUNCHCORE_STORAGE_ERR_ERASE;
                break;
            }
        }
        (void)HAL_FLASH_Lock();
    }
    if (cached && HAL_ICACHE_Enable() != HAL_OK) status = LAUNCHCORE_STORAGE_ERR_ERASE;
    return status;
}

static launchcore_storage_status_t write_data(uint32_t a, const void *data, uint32_t n)
{
    if (!data || !n || (a & 15u) || !writable(a, n)) return LAUNCHCORE_STORAGE_ERR_RANGE;
    const uint32_t cached = HAL_ICACHE_IsEnabled();
    if (cached && HAL_ICACHE_Disable() != HAL_OK) return LAUNCHCORE_STORAGE_ERR_WRITE;
    launchcore_storage_status_t status = LAUNCHCORE_STORAGE_ERR_WRITE;
    if (HAL_FLASH_Unlock() == HAL_OK) {
        const uint8_t *src = data;
        status = LAUNCHCORE_STORAGE_OK;
        for (uint32_t o = 0; o < n; o += 16) {
            uint32_t words[4] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
            const uint32_t take = n - o > 16 ? 16 : n - o;
            memcpy(words, src + o, take);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, a + o, (uint32_t)(uintptr_t)words) != HAL_OK) {
                status = LAUNCHCORE_STORAGE_ERR_WRITE;
                break;
            }
        }
        (void)HAL_FLASH_Lock();
        /* Verify while cache is disabled, before restoring normal execution. */
        if (status == LAUNCHCORE_STORAGE_OK && memcmp((const void *)(uintptr_t)a, data, n))
            status = LAUNCHCORE_STORAGE_ERR_VERIFY;
    }
    if (cached && HAL_ICACHE_Enable() != HAL_OK) status = LAUNCHCORE_STORAGE_ERR_WRITE;
    return status;
}
static launchcore_storage_status_t read_data(uint32_t a,void *data,uint32_t n){if(!data||!within(a,n,LAUNCHCORE_INTERNAL_FLASH_BASE,LAUNCHCORE_INTERNAL_FLASH_SIZE))return LAUNCHCORE_STORAGE_ERR_RANGE;memcpy(data,(const void *)(uintptr_t)a,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t mapped(void){return LAUNCHCORE_STORAGE_OK;}static bool executable(uint32_t a){return a>=BOARD_VECTOR_TABLE&&a<BOARD_SLOT_A_BASE+BOARD_SLOT_A_SIZE;}static const launchcore_storage_layout_t *get_layout(void){return &layout;}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=init,.erase=erase,.write=write_data,.read=read_data,.enable_memory_mapped=mapped,.is_executable_addr=executable,.layout=get_layout};

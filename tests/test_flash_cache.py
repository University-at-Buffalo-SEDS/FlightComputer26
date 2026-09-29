"""Compile the production flash mutation functions with HAL fault injection."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FlashCacheTests(unittest.TestCase):
    def test_cache_disabled_for_mutation_and_restored_on_errors(self):
        source=(ROOT/'Bootloader/storage_internal_flash.c').read_text()
        source=source[source.index('static launchcore_storage_status_t erase('):source.index('static launchcore_storage_status_t read_data(')]
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int launchcore_storage_status_t;
enum { LAUNCHCORE_STORAGE_OK, LAUNCHCORE_STORAGE_ERR_RANGE, LAUNCHCORE_STORAGE_ERR_ERASE,
 LAUNCHCORE_STORAGE_ERR_WRITE, LAUNCHCORE_STORAGE_ERR_VERIFY };
#define HAL_OK 0
#define FLASH_SECTOR_SIZE 8192U
#define FLASH_TYPEERASE_SECTORS 1
#define FLASH_TYPEPROGRAM_QUADWORD 2
typedef struct { unsigned TypeErase, NbSectors, Banks, Sector; } FLASH_EraseInitTypeDef;
static unsigned cached=1, unlock_error, operation_error, compare_error, disable_error;
static unsigned enables, disables, erases, programs, compares;
static int writable(uint32_t a, uint32_t n) { return a>=0x08078000 && n<=8192; }
static void sector(uint32_t a,uint32_t *bank,uint32_t *number) { (void)a; *bank=2; *number=28; }
static unsigned HAL_ICACHE_IsEnabled(void) { return cached; }
static int HAL_ICACHE_Disable(void) { disables++; if(disable_error) return 1; cached=0; return 0; }
static int HAL_ICACHE_Enable(void) { cached=1; enables++; return 0; }
static int HAL_FLASH_Unlock(void) { assert(!cached); return unlock_error; }
static int HAL_FLASH_Lock(void) { assert(!cached); return 0; }
static int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error) {
 assert(!cached && e->NbSectors==1); *error=0; erases++; return operation_error;
}
static int HAL_FLASH_Program(unsigned type,uint32_t address,uint32_t data) {
 (void)data; assert(!cached && type==2 && address>=0x08078000); programs++; return operation_error;
}
static int verify(const void *a,const void *b,size_t n) {
 (void)a; (void)b; assert(n>0 && !cached); compares++; return compare_error;
}
#define memcmp verify
''' + source + r'''
int main(void) {
 unsigned char data[156]={0};
 assert(erase(0x08078000,8192)==LAUNCHCORE_STORAGE_OK);
 assert(cached && erases==1 && enables==1 && disables==1);
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_OK);
 assert(cached && programs==10 && compares==1);
 compare_error=1;
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_ERR_VERIFY && cached);
 compare_error=0; operation_error=1;
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_ERR_WRITE && cached);
 assert(erase(0x08078000,8192)==LAUNCHCORE_STORAGE_ERR_ERASE && cached);
 operation_error=0; unlock_error=1;
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_ERR_WRITE && cached);
 assert(erase(0x08078000,8192)==LAUNCHCORE_STORAGE_ERR_ERASE && cached);
 unlock_error=0; disable_error=1; unsigned before=programs;
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_ERR_WRITE);
 assert(programs==before); disable_error=0;
 cached=0; before=enables;
 assert(write_data(0x08078000,data,sizeof data)==LAUNCHCORE_STORAGE_OK);
 assert(erase(0x08078000,8192)==LAUNCHCORE_STORAGE_OK);
 assert(!cached && enables==before); /* Bootloader starts with cache off. */
 before=disables;
 assert(write_data(0x08078001,data,sizeof data)==LAUNCHCORE_STORAGE_ERR_RANGE);
 assert(erase(0x08078001,8192)==LAUNCHCORE_STORAGE_ERR_RANGE);
 assert(disables==before);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=Path(tmp)/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True)

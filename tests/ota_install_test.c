/* Production LaunchCore install/confirmation with H523-sized mock flash. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board_config.h"
#include "launchcore/storage.h"
#include "launchcore/delta.h"
#include "launchcore/update.h"

static unsigned char flash[LAUNCHCORE_INTERNAL_FLASH_SIZE];
static const launchcore_storage_layout_t layout = {
    .slot_a_base=BOARD_SLOT_A_BASE, .slot_a_size=BOARD_SLOT_A_SIZE,
    .slot_b_base=BOARD_DELTA_BASE, .slot_b_size=BOARD_DELTA_SIZE,
    .slot_b_is_delta=true, .slot_erase_size=LAUNCHCORE_FLASH_ERASE_SIZE,
    .metadata0_base=BOARD_METADATA0_BASE, .metadata1_base=BOARD_METADATA1_BASE,
    .metadata_size=LAUNCHCORE_FLASH_ERASE_SIZE,
    .persistent_data_base=BOARD_PERSIST_BASE, .persistent_data_size=BOARD_PERSIST_SIZE,
    .persistent_data_write_size=16, .supports_xip=true
};
static unsigned char *address(uint32_t addr, uint32_t n) {
    assert(addr>=LAUNCHCORE_INTERNAL_FLASH_BASE);
    assert(n<=sizeof(flash) && addr-LAUNCHCORE_INTERNAL_FLASH_BASE<=sizeof(flash)-n);
    return flash+addr-LAUNCHCORE_INTERNAL_FLASH_BASE;
}
static launchcore_storage_status_t read_data(uint32_t a,void *d,uint32_t n) {
    memcpy(d,address(a,n),n); return LAUNCHCORE_STORAGE_OK;
}
static launchcore_storage_status_t erase(uint32_t a,uint32_t n) {
    assert(a%LAUNCHCORE_FLASH_ERASE_SIZE==0 && n%LAUNCHCORE_FLASH_ERASE_SIZE==0);
    assert(a>=BOARD_SLOT_A_BASE && a+n<=BOARD_PERSIST_BASE);
    memset(address(a,n),255,n); return LAUNCHCORE_STORAGE_OK;
}
static launchcore_storage_status_t write_data(uint32_t a,const void *d,uint32_t n) {
    assert(a%16==0 && a>=BOARD_SLOT_A_BASE && a+n<=BOARD_PERSIST_BASE);
    unsigned char *p=address(a,n); const unsigned char *src=d;
    for (uint32_t i=0;i<n;++i) { assert((p[i]&src[i])==src[i]); p[i]=src[i]; }
    return LAUNCHCORE_STORAGE_OK;
}
static const launchcore_storage_layout_t *get_layout(void) { return &layout; }
static const launchcore_storage_driver_t driver = {
    .read=read_data, .write=write_data, .erase=erase, .layout=get_layout
};
static size_t load(const char *path, unsigned char *dst, size_t cap) {
    FILE *f=fopen(path,"rb"); assert(f);
    size_t n=fread(dst,1,cap,f); assert(feof(f)); fclose(f); return n;
}
int main(int argc,char **argv) {
    assert(argc==4);
    memset(flash,255,sizeof(flash));
    memset(address(BOARD_PERSIST_BASE,BOARD_PERSIST_SIZE),0x5a,BOARD_PERSIST_SIZE);
    (void)load(argv[1],address(BOARD_SLOT_A_BASE,BOARD_SLOT_A_SIZE),BOARD_SLOT_A_SIZE);
    launchcore_storage_set_driver(&driver);
    launchcore_metadata_t md;
    launchcore_metadata_init_default(&md);
    md.active_slot=md.confirmed_slot=LAUNCHCORE_SLOT_A;
    md.pending_slot=LAUNCHCORE_SLOT_NONE;
    launchcore_metadata_set_slot_state(&md,LAUNCHCORE_SLOT_A,LAUNCHCORE_SLOT_CONFIRMED);
    assert(launchcore_metadata_commit(&md)==LAUNCHCORE_METADATA_OK);
    unsigned char patch[BOARD_DELTA_SIZE], target[BOARD_SLOT_A_SIZE];
    size_t count=load(argv[2],patch,sizeof(patch));
    size_t target_size=load(argv[3],target,sizeof(target));
    assert(launchcore_delta_update_begin(BOARD_DELTA_SIZE+1)!=LAUNCHCORE_OK);
    assert(launchcore_delta_update_begin(count)==LAUNCHCORE_OK);
    assert(launchcore_delta_update_write(patch,112)==LAUNCHCORE_OK);
    assert(launchcore_delta_update_abort()==LAUNCHCORE_OK);
    /* Interrupted transport must leave installed firmware intact and permit retry. */
    assert(launchcore_delta_update_begin(count)==LAUNCHCORE_OK);
    for (size_t off=0;off<count;) {
        size_t len=count-off>112 ? 112 : count-off;
        assert(launchcore_delta_update_write(patch+off,len)==LAUNCHCORE_OK); off+=len;
    }
    assert(launchcore_delta_update_finish()==LAUNCHCORE_OK);
    assert(launchcore_metadata_read_latest(&md)==LAUNCHCORE_METADATA_OK);
    assert(launchcore_process_delta_update(&md)==LAUNCHCORE_DELTA_READY);
    assert(memcmp(address(BOARD_SLOT_A_BASE,target_size),target,target_size)==0);
    md.active_slot=LAUNCHCORE_SLOT_A;
    assert(launchcore_metadata_commit(&md)==LAUNCHCORE_METADATA_OK);
    assert(launchcore_confirm_boot()==LAUNCHCORE_OK);
    assert(launchcore_metadata_read_latest(&md)==LAUNCHCORE_METADATA_OK);
    assert(md.confirmed_slot==LAUNCHCORE_SLOT_A && md.pending_slot==LAUNCHCORE_SLOT_NONE);
    for (unsigned i=0;i<BOARD_PERSIST_SIZE;++i)
        assert(address(BOARD_PERSIST_BASE,BOARD_PERSIST_SIZE)[i]==0x5a);
    puts("delta installed, confirmed, and persistent settings preserved");
}

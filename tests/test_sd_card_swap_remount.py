"""SD card swap under the actual FileX: a card that left its slot is mounted fresh.

The real FileX sources (standalone) run over a RAM card behind the actual
resilient_storage.c driver wrapper and recovery. A mount keeps the old card's
directory, FAT and sector cache: a card rewritten elsewhere (another IGT
dataset) and put back reads the old CSV bytes unless the volume is aborted and
reopened. Abort writes nothing, so the old card's dirty cache never reaches the
new one.
"""
from pathlib import Path
import os, subprocess, tempfile, unittest

ROOT = Path(__file__).resolve().parents[1]
FX = ROOT / 'Middlewares/ST/filex'


def stripped(path):
    return '\n'.join(line for line in path.read_text().splitlines() if not line.startswith('#include'))


PRE = r'''
#include "fx_api.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define HAL_OK 0
#define HAL_ERROR 1
#define SDMMC1_IRQn 49
#define __DSB() ((void)0)
#define __HAL_RCC_SDMMC1_FORCE_RESET() ((void)0)
#define __HAL_RCC_SDMMC1_RELEASE_RESET() ((void)0)
typedef int HAL_StatusTypeDef;
typedef struct {uint32_t MASK,IDMACTRL,DCTRL;} Registers;
typedef struct {Registers *Instance;uint32_t CID[4];struct {uint32_t LogBlockNbr;} SdCard;} SD_HandleTypeDef;
static Registers regs; SD_HandleTypeDef hsd1={.Instance=&regs};
#define SECTORS 4096U
typedef struct {uint8_t img[SECTORS*512U];uint32_t cid[4];} Card;
static Card card_a={.cid={1,2,3,4}},card_b={.cid={1,2,3,4}},blank={.cid={1,2,3,4}};
static Card *slot; /* NULL: the slot is empty */
static unsigned long ticks;
static unsigned hooks;
void HAL_NVIC_DisableIRQ(int irq){(void)irq;}
void HAL_NVIC_ClearPendingIRQ(int irq){(void)irq;}
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *h){(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *h){
 if(!slot)return HAL_ERROR;
 memcpy(h->CID,slot->cid,sizeof h->CID);h->SdCard.LogBlockNbr=SECTORS;return HAL_OK;
}
UINT flight_sd_dma_drain(void){return FX_SUCCESS;}
ULONG tx_time_get(void){return ticks;}
void tx_thread_sleep(unsigned n){ticks+=n;}
void sd_pipeline_media_remounted(void){++hooks;}
/* The card itself: sector reads/writes into the card in the slot, IO error with none. */
static void card_driver(FX_MEDIA *m){
 m->fx_media_driver_status=FX_SUCCESS;
 const ULONG first=m->fx_media_driver_request==FX_DRIVER_BOOT_READ||m->fx_media_driver_request==FX_DRIVER_BOOT_WRITE?
   0U:m->fx_media_driver_logical_sector+m->fx_media_hidden_sectors;
 const ULONG count=m->fx_media_driver_request==FX_DRIVER_BOOT_READ||m->fx_media_driver_request==FX_DRIVER_BOOT_WRITE?
   1U:m->fx_media_driver_sectors;
 switch(m->fx_media_driver_request){
 case FX_DRIVER_READ: case FX_DRIVER_BOOT_READ:
  if(!slot){m->fx_media_driver_status=FX_IO_ERROR;return;}
  assert((first+count)<=SECTORS);memcpy(m->fx_media_driver_buffer,slot->img+first*512U,count*512U);return;
 case FX_DRIVER_WRITE: case FX_DRIVER_BOOT_WRITE:
  if(!slot){m->fx_media_driver_status=FX_IO_ERROR;return;}
  assert((first+count)<=SECTORS);memcpy(slot->img+first*512U,m->fx_media_driver_buffer,count*512U);return;
 default: return;
 }
}
'''

MAIN = r'''
static FX_MEDIA disk; static ULONG cache[512/sizeof(ULONG)]; /* the FC's one-sector media cache */
static FX_FILE file,log_file;
static const char old_csv[]="t_ms,ax,ay,az,gx,gy,gz\n0,0,0,9.8,0,0,0\n";
static const char new_csv[]="t_ms,ax,ay,az,gx,gy,gz\n0,0.1,0,9.81,0,0,0\n10,0.2,0,9.82,0,0,0\n";

/* Formatted elsewhere (a laptop): the new card's CSV lives in other clusters. */
static void prepare(Card *card,const char *csv,unsigned pad){
 Card *was=slot;slot=card;FX_MEDIA m;static ULONG c[128];
 assert(fx_media_format(&m,card_driver,0,(UCHAR*)c,sizeof c,"SEDS",1,32,0,SECTORS,512,1,1,1)==FX_SUCCESS);
 assert(fx_media_open(&m,"host",card_driver,0,c,sizeof c)==FX_SUCCESS);
 assert(fx_directory_create(&m,"IGT")==FX_SUCCESS);
 if(pad){static char z[3*512];assert(fx_file_create(&m,"PAD.BIN")==FX_SUCCESS);
  assert(fx_file_open(&m,&file,"PAD.BIN",FX_OPEN_FOR_WRITE)==FX_SUCCESS);
  assert(fx_file_write(&file,z,pad)==FX_SUCCESS);assert(fx_file_close(&file)==FX_SUCCESS);}
 assert(fx_file_create(&m,"/IGT/IMU0.CSV")==FX_SUCCESS);
 assert(fx_file_open(&m,&file,"/IGT/IMU0.CSV",FX_OPEN_FOR_WRITE)==FX_SUCCESS);
 assert(fx_file_write(&file,(void*)csv,strlen(csv))==FX_SUCCESS);
 assert(fx_file_close(&file)==FX_SUCCESS);assert(fx_media_close(&m)==FX_SUCCESS);slot=was;
}
/* igt_port_open/read/close: one open-read-close per chunk. */
static UINT read_csv(char *out,ULONG cap){
 memset(out,0,cap);ULONG n=0;
 UINT st=fx_file_open(&disk,&file,"/IGT/IMU0.CSV",FX_OPEN_FOR_READ);
 if(st!=FX_SUCCESS)return st;
 st=fx_file_read(&file,out,cap-1U,&n);(void)fx_file_close(&file);return st;
}
static void mount(void){
 slot=&card_a;
 assert(flight_fx_media_open(&disk,"SD",card_driver,0,cache,sizeof cache)==FX_SUCCESS);
 /* The logger's open part with unflushed FAT/directory state in the cache. */
 assert(fx_file_create(&disk,"FC-1.CSV")==FX_SUCCESS);
 assert(fx_file_open(&disk,&log_file,"FC-1.CSV",FX_OPEN_FOR_WRITE)==FX_SUCCESS);
 static char rows[700];memset(rows,'r',sizeof rows);
 assert(fx_file_write(&log_file,rows,sizeof rows)==FX_SUCCESS);
 /* IGT validation of the first dataset leaves its CSV in the cache. */
 char buf[256];assert(read_csv(buf,sizeof buf)==FX_SUCCESS&&!strcmp(buf,old_csv));
}
/* The card leaves: the next write fails, as the pipeline reports it. */
static void pull(void){
 slot=NULL;
 char row[600];memset(row,'x',sizeof row);
 UINT st=fx_file_write(&log_file,row,sizeof row);
 if(st==FX_SUCCESS)st=fx_media_flush(&disk);
 assert(st!=FX_SUCCESS);flight_sd_note_io_failure(st);
 assert(!flight_sd_io_verified());
}
static Card snapshot;
int main(int argc,char**argv){
 assert(argc==2);const unsigned scenario=(unsigned)strtoul(argv[1],0,10);
 fx_system_initialize();
 prepare(&card_a,old_csv,0);prepare(&card_b,new_csv,1500);
 char buf[256];
 if(scenario==0){
  /* Control: without a remount FileX answers from the old card's cache. */
  mount();slot=&card_b;assert(read_csv(buf,sizeof buf)==FX_SUCCESS);
  assert(!strcmp(buf,old_csv));return 0;
 }
 mount();
 if(scenario==1){
  /* Same card rewritten elsewhere (same CID): out long enough that
   * re-initialisation found the slot empty. */
  pull();assert(flight_sd_prepare_io()!=FX_SUCCESS); /* empty slot */
  snapshot=card_b;slot=&card_b;
  assert(flight_sd_prepare_io()!=FX_SUCCESS);       /* retry is paced */
  assert(hooks==0);ticks+=1000;
 }else if(scenario==2){
  /* Another card (other CID) in the slot by the first re-initialisation. */
  card_b.cid[0]=99;pull();snapshot=card_b;slot=&card_b;
 }else if(scenario==3){
  /* An unformatted card first: no mount, retried; then the dataset card. */
  pull();assert(flight_sd_prepare_io()!=FX_SUCCESS);
  slot=&blank;ticks+=1000;assert(flight_sd_prepare_io()!=FX_SUCCESS);
  assert(hooks==1&&g_sd_remounts==0&&g_sd_mount_failures==1);
  assert(read_csv(buf,sizeof buf)!=FX_SUCCESS); /* nothing mounted: no I/O */
  snapshot=card_b;slot=&card_b;ticks+=1000;
 }else if(scenario==4){
  /* A seated card's transient fault resumes on its retained mount. */
  slot=NULL;assert(fx_media_flush(&disk)!=FX_SUCCESS);flight_sd_note_io_failure(FX_IO_ERROR);
  slot=&card_a;ticks+=1000;assert(flight_sd_prepare_io()==FX_SUCCESS);
  assert(hooks==0&&g_sd_remounts==0);
  assert(fx_media_flush(&disk)==FX_SUCCESS&&fx_file_close(&log_file)==FX_SUCCESS);
  ULONG size=0;assert(fx_file_open(&disk,&file,"FC-1.CSV",FX_OPEN_FOR_READ)==FX_SUCCESS);
  size=(ULONG)file.fx_file_current_file_size;(void)fx_file_close(&file);assert(size==700U);
  return 0;
 }
 assert(flight_sd_prepare_io()==FX_SUCCESS);
 assert(g_sd_remounts==1&&hooks==(scenario==3?2U:1U));
 assert(g_sd_card_identity_mismatches==(scenario==2));
 /* The dataset now on the card, not the cached one. */
 assert(read_csv(buf,sizeof buf)==FX_SUCCESS);assert(!strcmp(buf,new_csv));
 /* The old card's dirty cache and log handle were dropped, unwritten. */
 assert(!memcmp(snapshot.img,card_b.img,sizeof card_b.img));
 assert(fx_file_write(&log_file,buf,1)!=FX_SUCCESS);
 /* The new mount takes a new log part as usual. */
 assert(fx_file_create(&disk,"FC-2.CSV")==FX_SUCCESS);
 assert(fx_file_open(&disk,&log_file,"FC-2.CSV",FX_OPEN_FOR_WRITE)==FX_SUCCESS);
 assert(fx_file_write(&log_file,"row\n",4)==FX_SUCCESS&&fx_file_close(&log_file)==FX_SUCCESS);
 assert(fx_media_flush(&disk)==FX_SUCCESS);
 assert(read_csv(buf,sizeof buf)==FX_SUCCESS&&!strcmp(buf,new_csv));
 return 0;
}
'''


class SdCardSwapRemount(unittest.TestCase):
    def test_swapped_card_is_read_fresh_and_never_written_with_old_cache(self):
        program = PRE + stripped(ROOT / 'Core/Src/resilient_storage.c') + '\n' + MAIN
        sources = sorted(str(p) for p in (FX / 'common/src').glob('*.c'))
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'swap.c').write_text(program)
            exe = d / 'swap'
            # FileX itself builds with its own warnings; only the harness is -Werror.
            subprocess.run(['cc', '-std=gnu11', '-O1', '-g', '-w', '-DFX_STANDALONE_ENABLE',
                            '-DFX_INCLUDE_USER_DEFINE_FILE', '-I', str(ROOT / 'FileX/App'),
                            '-I', str(FX / 'common/inc'), '-I', str(FX / 'ports/generic/inc'),
                            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                            '-c', *sources], cwd=d, check=True)
            subprocess.run(['cc', '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-parameter', '-Wno-unused-function', '-DFX_STANDALONE_ENABLE',
                            '-DFX_INCLUDE_USER_DEFINE_FILE', '-I', str(ROOT / 'FileX/App'),
                            '-I', str(FX / 'common/inc'), '-I', str(FX / 'ports/generic/inc'),
                            '-I', str(ROOT / 'Core/Inc'), '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', str(d / 'swap.c'), *sorted(str(p) for p in d.glob('*.o')),
                            '-o', str(exe)], check=True)
            env = os.environ.copy()
            env['ASAN_OPTIONS'] = 'detect_leaks=1:abort_on_error=1'
            env['UBSAN_OPTIONS'] = 'halt_on_error=1'
            for scenario in range(5):
                with self.subTest(scenario=scenario):
                    subprocess.run([str(exe), str(scenario)], check=True, env=env)


if __name__ == '__main__':
    unittest.main()

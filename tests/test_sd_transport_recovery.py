"""Actual FC writer + resilience + DMA glue, with deterministic FileX/HAL faults."""
from pathlib import Path
import subprocess, tempfile, unittest, os
ROOT=Path(__file__).resolve().parents[1]
def stripped(path):
    return '\n'.join(line for line in path.read_text().splitlines() if not line.startswith('#include'))
PRE=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sd_file_limit.h"
typedef unsigned UINT; typedef unsigned long ULONG; typedef int INT;
typedef void VOID; typedef char CHAR; typedef uint16_t fu16;
#define FX_SUCCESS 0U
#define FX_IO_ERROR 0x90U
#define FX_DRIVER_READ 1U
#define FX_DRIVER_WRITE 2U
#define FX_DRIVER_BOOT_READ 3U
#define FX_DRIVER_BOOT_WRITE 4U
#define FX_NO_MORE_SPACE 0x0aU
#define TX_SUCCESS 0U
#define TX_NO_INSTANCE 13U
#define TX_NO_WAIT 0U
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_SD_CARD_TRANSFER 4U
#define FX_STM32_SD_INIT 0
#define UNUSED(x) ((void)(x))
#define SDMMC1_IRQn 49
#define id "SD "
#define WE_ARE_SO_BACK 0
#define sweetbench_start(x) ((void)0)
#define sweetbench_catch(x) ((void)0)
#define log_err(...) ((void)0)
typedef int HAL_StatusTypeDef;
typedef struct {uint32_t MASK,IDMACTRL,DCTRL;} Registers;
typedef struct {Registers *Instance;uint32_t CID[4];struct {uint32_t LogBlockNbr;} SdCard;} SD_HandleTypeDef;
typedef struct {unsigned count,created;} TX_SEMAPHORE;
typedef struct {unsigned mounted;UINT fx_media_driver_request,fx_media_driver_status;} FX_MEDIA;
Registers regs; SD_HandleTypeDef hsd1={.Instance=&regs};
unsigned ticks,irq_enabled=1,pending_irq,barriers,resets,initializations,sema_creates;
unsigned card_present=1,mount_failures,protocol_multiblock;
uint32_t physical_cid[4]={1,2,3,4},expected_cid[4]={1,2,3,4},physical_blocks=10000;
extern volatile uint32_t g_sd_ready;
void HAL_NVIC_DisableIRQ(int irq){assert(irq==SDMMC1_IRQn);irq_enabled=0;}
void HAL_NVIC_ClearPendingIRQ(int irq){assert(irq==SDMMC1_IRQn&&!irq_enabled);pending_irq=0;}
void barrier(void){++barriers;}
#define __DSB() barrier()
void force_reset(void){assert(!irq_enabled&&!regs.IDMACTRL&&!regs.DCTRL&&!regs.MASK);++resets;pending_irq=1;}
#define __HAL_RCC_SDMMC1_FORCE_RESET() force_reset()
#define __HAL_RCC_SDMMC1_RELEASE_RESET() ((void)0)
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *h){assert(h==&hsd1&&!irq_enabled&&!regs.IDMACTRL);return HAL_OK;}
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *h){
 assert(h==&hsd1);++initializations;assert(!regs.IDMACTRL&&!pending_irq);
 protocol_multiblock=0; /* actual H5 InitCard/PowerON sends CMD0; source check below */
 irq_enabled=1;if(!card_present)return HAL_ERROR;
 memcpy(h->CID,physical_cid,sizeof physical_cid);h->SdCard.LogBlockNbr=physical_blocks;return HAL_OK;
}
unsigned HAL_SD_GetCardState(SD_HandleTypeDef *h){(void)h;return card_present?HAL_SD_CARD_TRANSFER:0;}
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef*h,uint8_t*b,UINT s,UINT n){(void)h;(void)b;(void)s;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef*h,uint8_t*b,UINT s,UINT n){(void)h;(void)b;(void)s;(void)n;return HAL_OK;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,unsigned count){(void)n;assert(!s->created);s->created=1;s->count=count;++sema_creates;return TX_SUCCESS;}
UINT tx_semaphore_delete(TX_SEMAPHORE*s){assert(s->created);s->created=0;s->count=0;return TX_SUCCESS;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,unsigned timeout){assert(timeout==TX_NO_WAIT);if(!s->created)return 1;if(!s->count)return TX_NO_INSTANCE;--s->count;return TX_SUCCESS;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){assert(s->created);++s->count;return TX_SUCCESS;}
ULONG tx_time_get(void){return ticks;}
void tx_thread_sleep(unsigned count){ticks+=count;}
UINT flight_sd_dma_prepare(void);void flight_sd_dma_release(void);UINT flight_sd_dma_drain(void);
UINT fx_media_open(FX_MEDIA*m,CHAR*n,VOID(*d)(FX_MEDIA*),VOID*i,VOID*b,ULONG s){
 (void)n;(void)d;(void)i;(void)b;(void)s;assert(flight_sd_dma_prepare()==FX_SUCCESS);
 if(mount_failures){--mount_failures;return FX_IO_ERROR;}m->mounted=1;return FX_SUCCESS;
}
'''
WRITER=r'''
struct {ULONG fx_file_current_file_size;} file;
struct {int lock;bool free,cur;fu16 off[2];} line;
char sdbuf[2][256];uint32_t sd_buffer_records[2],g_sd_written_records,g_sd_flushed_records,g_sd_max_write_ms;
uint32_t g_sd_write_errors,g_sd_shutdown_lost_records,g_sd_shutdown_lost_events,g_sd_event_pending;
bool sd_writer_slot,sd_retry_pending,sd_accepting,sd_launch_pending,sd_launch_active;
uint8_t sd_file_open=1;
bool sd_launch_completed,sd_cutoff_set,sd_cutoff_slot,sd_landed_seen;
uint32_t sd_landed_tick,sd_cutoff_records,g_sd_launch_closed;
uint16_t sd_cutoff_offset,sd_writer_skip;ULONG sd_retry_offset;
FX_MEDIA sdio_disk;char disk[1024];unsigned position,durable_size,fail_status,fail_bytes,seek_failure,flush_failure,write_calls,flush_calls,close_calls;
void fc_lock(int*p){assert(!*p);*p=1;}void fc_unlock(int*p){assert(*p);*p=0;}
void sd_service_events(bool draining){(void)draining;}void sd_handoff_tail(void){}
uint32_t now_ms(void){return ticks;}
static void verify_same_card(void){assert(card_present&&!transport_lost&&!memcmp(physical_cid,expected_cid,sizeof expected_cid)&&physical_blocks==10000);}
UINT fx_file_seek(void*f,ULONG at){(void)f;assert(!line.lock);if(seek_failure){seek_failure=0;return FX_IO_ERROR;}verify_same_card();position=at;return FX_SUCCESS;}
UINT fx_file_write(void*f,void*b,ULONG n){
 (void)f;assert(!line.lock);verify_same_card();++write_calls;ticks+=7;
 if(fail_status){unsigned status=fail_status;fail_status=0;unsigned count=fail_bytes<n?fail_bytes:(unsigned)n;
  memcpy(disk+position,b,count);position+=count;if(position>file.fx_file_current_file_size)file.fx_file_current_file_size=position;
  if(status==FX_IO_ERROR){regs.MASK=1;regs.IDMACTRL=1;regs.DCTRL=1;pending_irq=1;protocol_multiblock=1;
   HAL_SD_TxCpltCallback(&hsd1);HAL_SD_RxCpltCallback(&hsd1);}
  return status;
 }
 memcpy(disk+position,b,n);position+=n;if(position>file.fx_file_current_file_size)file.fx_file_current_file_size=position;return FX_SUCCESS;
}
UINT fx_media_flush(void*m){(void)m;assert(!line.lock);++flush_calls;if(flush_failure){flush_failure=0;return FX_IO_ERROR;}verify_same_card();durable_size=file.fx_file_current_file_size;return FX_SUCCESS;}
UINT fx_file_close(void*f){(void)f;++close_calls;return FX_SUCCESS;}UINT fx_media_close(void*m){(void)m;return FX_SUCCESS;}
static inline int sd_pipeline_open(const char*s){(void)s;assert(!"unexpected rotation/open in retained-write control");return 1;}
'''
MAIN=r'''
static const char row[]="1700000000123,9876,raw,1.25\n";
static void begin(void){
 FX_MEDIA media={0};assert(flight_fx_media_open(&media,"test",0,0,0,0)==FX_SUCCESS);
 assert(sema_creates==2&&g_sd_ready==1);memcpy(disk,"header\n",7);file.fx_file_current_file_size=position=7;
 memcpy(sdbuf[0],row,sizeof row-1);line.off[0]=sizeof row-1;sd_buffer_records[0]=1;
}
static void exact_row(void){assert(file.fx_file_current_file_size==7+sizeof row-1);assert(!memcmp(disk,"header\n",7));assert(!memcmp(disk+7,row,sizeof row-1));assert(g_sd_written_records==1);}
int main(int argc,char**argv){assert(argc==2);unsigned scenario=(unsigned)strtoul(argv[1],0,10);
 if(scenario==6){mount_failures=1;begin();assert(sema_creates==2&&g_sd_mount_failures==1);return 0;}
 if(scenario==7){begin();sd_retry_pending=true;sd_retry_offset=7;seek_failure=1;
 assert(!sd_try_write_line());assert(!g_sd_ready&&!g_sd_written_records&&write_calls==0);
 assert(sd_try_write_line());exact_row();return 0;}
 if(scenario==8){begin();sd_tx_semaphore.count=20;flight_sd_note_io_failure(FX_IO_ERROR);
 assert(!dma_quiesced&&sd_tx_semaphore.count==12);unsigned before=initializations;
 assert(flight_sd_prepare_io()!=FX_SUCCESS&&initializations==before&&sd_tx_semaphore.count==4);
 ticks+=1000;assert(flight_sd_prepare_io()==FX_SUCCESS&&sd_tx_semaphore.count==0);
 assert(!g_sd_ready);assert(sd_try_write_line());exact_row();return 0;}
 if(scenario==9){begin();for(unsigned n=0;n<5;n++){fail_status=FX_IO_ERROR;fail_bytes=n%3;
 assert(!sd_try_write_line());assert(!g_sd_ready&&!g_sd_written_records&&sd_retry_pending);ticks+=1000;}
 assert(sd_try_write_line());exact_row();assert(g_sd_write_errors==5);return 0;}
 if(scenario==10){begin();fail_status=FX_IO_ERROR;fail_bytes=2;assert(!sd_try_write_line());
 card_present=0;sd_pipeline_shutdown("lost");assert(write_calls==1&&!flush_calls&&!close_calls&&!g_sd_ready);
 assert(g_sd_shutdown_lost_records==1&&!g_sd_flushed_records&&sd_file_open);return 0;}
 if(scenario==11){begin();assert(sd_try_write_line());flush_failure=1;sd_pipeline_shutdown("flush");
 assert(flush_calls==1&&!close_calls&&!g_sd_ready&&!g_sd_flushed_records&&sd_file_open);return 0;}
 begin();unsigned resets_before=resets;
 fail_status=scenario==2?FX_NO_MORE_SPACE:FX_IO_ERROR;fail_bytes=scenario==1?sizeof row-1:2;
 assert(!sd_try_write_line());assert(line.off[0]==sizeof row-1&&sd_retry_pending&&!line.free&&!sd_writer_slot);
 assert(!g_sd_written_records&&!g_sd_flushed_records&&g_sd_write_errors==1&&!g_sd_ready);
 if(scenario==2){assert(resets==resets_before);assert(sd_try_write_line());exact_row();assert(g_sd_ready);return 0;}
 assert(!irq_enabled&&!regs.IDMACTRL&&!regs.DCTRL&&!pending_irq&&!sd_tx_semaphore.count&&!sd_rx_semaphore.count);
 /* A latched old IRQ cannot now generate a completion for a new write. */
 if(irq_enabled&&pending_irq)HAL_SD_TxCpltCallback(&hsd1);assert(!sd_tx_semaphore.count);
 if(scenario==3){card_present=0;assert(!sd_try_write_line());assert(!g_sd_ready&&!g_sd_written_records);card_present=1;ticks+=1000;}
 if(scenario==4||scenario==5){if(scenario==4)physical_cid[0]=9;else physical_blocks++;
  unsigned before=write_calls;for(unsigned n=0;n<3;n++){ticks+=1000;assert(!sd_try_write_line());}
  assert(write_calls==before&&g_sd_card_identity_mismatches==3&&!g_sd_ready&&sd_retry_pending);return 0;}
 assert(sd_try_write_line());exact_row();assert(g_sd_ready&&!protocol_multiblock&&g_sd_transport_retries>=1);
 assert(g_sd_max_write_ms==7&&!g_sd_flushed_records);
 flush_failure=1;assert(sd_io_failed(fx_media_flush(&sdio_disk)));assert(!g_sd_ready&&!g_sd_flushed_records);
 ticks+=1000;assert(flight_sd_prepare_io()==FX_SUCCESS);assert(!g_sd_ready); /* HAL init is not write proof */
 assert(fx_media_flush(&sdio_disk)==FX_SUCCESS);g_sd_flushed_records=g_sd_written_records;flight_sd_note_progress();
 assert(g_sd_ready&&g_sd_flushed_records==1&&durable_size==7+sizeof row-1);
 assert(sizeof mounted_cid+sizeof mounted_blocks+sizeof retry_at+sizeof mounted_identity+sizeof transport_lost+sizeof retry_started+sizeof dma_quiesced+
 sizeof g_sd_transport_retries+sizeof g_sd_card_identity_mismatches+sizeof g_sd_runtime_failures+sizeof g_sd_last_io_status+sizeof g_sd_max_write_ms+sizeof dma_semaphores<=64);
 return 0;
}
'''
class TransportRecovery(unittest.TestCase):
 def test_actual_writer_transport_and_dma_faults(self):
  source=(ROOT/'Core/Src/sdpipeline.c').read_text();writer=source[source.index('static inline bool sd_io_failed(UINT status)'):source.index('static inline int sd_pipeline_init(')]
  program=PRE+stripped(ROOT/'FileX/Target/fx_stm32_sd_driver_glue.c')+'\n'+stripped(ROOT/'Core/Src/resilient_storage.c')+'\n'+WRITER+writer+MAIN
  with tempfile.TemporaryDirectory()as d:
   d=Path(d);c=d/'faults.c';c.write_text(program)
   for label,flags in [('native',[]),('sanitized',['-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
    exe=d/label;subprocess.run(['cc','-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-misleading-indentation','-I',str(ROOT/'Core/Inc'),*flags,str(c),'-o',str(exe)],check=True)
    env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=1:abort_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1'
    for scenario in range(12):subprocess.run([str(exe),str(scenario)],check=True,env=env)
 def test_actual_h5_initialization_sends_cmd0_after_reset(self):
  hal=(ROOT/'Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_sd.c').read_text()
  init=hal[hal.index('HAL_StatusTypeDef HAL_SD_Init('):hal.index('HAL_StatusTypeDef HAL_SD_InitCard(')]
  card=hal[hal.index('HAL_StatusTypeDef HAL_SD_InitCard('):hal.index('HAL_StatusTypeDef HAL_SD_DeInit(')]
  power=hal[hal.index('static uint32_t SD_PowerON(SD_HandleTypeDef *hsd)\n{'):]
  self.assertIn('HAL_SD_InitCard(hsd)',init);self.assertIn('SD_PowerON(hsd)',card);self.assertIn('SDMMC_CmdGoIdleState',power)
if __name__=='__main__':unittest.main()

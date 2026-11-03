"""Actual pipeline/driver-origin gate, including FileX's structural error alias."""
from pathlib import Path
import importlib.util, subprocess, tempfile, unittest, os, re
ROOT=Path(__file__).resolve().parents[1]
def extract(t,sig):
 start=t.index(sig);end=t.index('{',start)+1;depth=1
 while depth:depth+=(t[end]=='{')-(t[end]=='}');end+=1
 return t[start:end]
def program(root=ROOT):
 spec=importlib.util.spec_from_file_location('transport_fixture',root/'tests/test_sd_transport_recovery.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 vendor=(root/'Middlewares/ST/filex/common/src/fx_media_open.c').read_text()
 boot=extract(vendor[vendor.index('/* Determine if the boot sector was read correctly. */'):],'if (media_ptr -> fx_media_driver_status != FX_SUCCESS)')
 fat=extract(vendor[vendor.index('Read the FAT sectors directly from the driver.'):],'if (media_ptr -> fx_media_driver_status != FX_SUCCESS)')
 structural=extract(vendor,'if ((cluster_number == FAT_entry) || (i > media_ptr -> fx_media_total_clusters))')
 invalid=extract(vendor,'if (_fx_media_boot_info_extract(media_ptr) != FX_SUCCESS)')
 pre=m.PRE.replace('typedef struct {unsigned mounted;UINT fx_media_driver_request,fx_media_driver_status;} FX_MEDIA;', 'typedef struct FX_MEDIA_STRUCT {unsigned mounted;UINT fx_media_driver_request,fx_media_driver_status,fx_media_total_clusters;void(*fx_media_driver_entry)(struct FX_MEDIA_STRUCT*);} FX_MEDIA;')
 api=(root/'Middlewares/ST/filex/common/inc/fx_api.h').read_text()
 defs='\n'+''.join('#define '+n+' '+re.search(r'#define\s+'+n+r'\s+(\S+)',api).group(1)+'\n'for n in ['FX_BOOT_ERROR','FX_FAT_READ_ERROR','FX_MEDIA_INVALID'])+'\n#define FX_DRIVER_UNINIT 7U\n#define FX_TRACE_IN_LINE_INSERT(...) ((void)0);\n'
 pre=defs+pre
 old=extract(pre,'UINT fx_media_open(')
 pre=pre.replace(old,'''void HAL_SD_TxCpltCallback(SD_HandleTypeDef*);
static unsigned scenario, mount_attempt, actual_driver_calls, hidden_write, hidden_flush;
static void actual_driver(FX_MEDIA*media){
 ++actual_driver_calls;
 if(media->fx_media_driver_request==FX_DRIVER_UNINIT){media->fx_media_driver_status=FX_IO_ERROR;return;}
 if((scenario<2&&!mount_attempt)||hidden_write||hidden_flush){
  regs.MASK=regs.IDMACTRL=regs.DCTRL=1;pending_irq=1;
  HAL_SD_TxCpltCallback(&hsd1);media->fx_media_driver_status=FX_IO_ERROR;
 }else media->fx_media_driver_status=FX_SUCCESS;
}
static UINT _fx_media_boot_info_extract(FX_MEDIA*m){(void)m;return FX_MEDIA_INVALID;}
UINT fx_media_open(FX_MEDIA*media_ptr,CHAR*n,VOID(*d)(FX_MEDIA*),VOID*info,VOID*b,ULONG z){
 (void)n;(void)info;(void)b;(void)z;
 assert(flight_sd_dma_prepare()==FX_SUCCESS);
 media_ptr->fx_media_driver_entry=d;
 if(!mount_attempt){
  media_ptr->fx_media_driver_request=scenario==0?FX_DRIVER_BOOT_READ:FX_DRIVER_READ;
  d(media_ptr);
  if(scenario==0){++mount_attempt;'''+boot+'''}
  if(scenario==1){++mount_attempt;'''+fat+'''}
  if(scenario==2){UINT cluster_number=2,FAT_entry=2,i=1;media_ptr->fx_media_total_clusters=10;
   ++mount_attempt;'''+structural+'''}
  if(scenario==3){++mount_attempt;'''+invalid+'''}
 }
 ++mount_attempt;media_ptr->mounted=1;return FX_SUCCESS;
}
''')
 writer=m.WRITER.replace('return FX_SUCCESS;\n}', '''if(hidden_write){sdio_disk.fx_media_driver_request=FX_DRIVER_WRITE;sdio_disk.fx_media_driver_entry(&sdio_disk);hidden_write=0;}
 return FX_SUCCESS;
}''',1)
 # Replacement above targets fx_file_write's final return, not seek's one-line definition.
 writer=writer.replace('verify_same_card();durable_size=file.fx_file_current_file_size;', 'verify_same_card();if(hidden_flush){sdio_disk.fx_media_driver_request=FX_DRIVER_WRITE;sdio_disk.fx_media_driver_entry(&sdio_disk);hidden_flush=0;}durable_size=file.fx_file_current_file_size;')
 main=r'''
int main(int argc,char**argv){assert(argc==2);scenario=(unsigned)strtoul(argv[1],0,10);
 assert(flight_fx_media_open(&sdio_disk,"test",actual_driver,0,0,0)==FX_SUCCESS);
 if(scenario<2){assert(resets>0&&!regs.IDMACTRL&&!pending_irq);assert(g_sd_last_io_status==(scenario==0?FX_BOOT_ERROR:FX_FAT_READ_ERROR));}
 if(scenario==2||scenario==3){assert(!resets);return 0;}
 if(scenario<2)return 0;
 if(scenario==6){
  VOID(*wrapped)(FX_MEDIA*)=sdio_disk.fx_media_driver_entry;
  assert(flight_fx_media_open(&sdio_disk,"again",wrapped,0,0,0)==FX_SUCCESS);
  sdio_disk.fx_media_driver_request=FX_DRIVER_UNINIT;wrapped(&sdio_disk);
  assert(flight_sd_io_verified()&&!resets);return 0;
 }
 memcpy(disk,"header\n",7);file.fx_file_current_file_size=position=7;
 memcpy(sdbuf[0],"originalUTC,exactROW\n",21);line.off[0]=21;sd_buffer_records[0]=1;
 if(scenario==4){
  hidden_write=1;assert(!sd_try_write_line());
  assert(!g_sd_ready&&!g_sd_written_records&&!g_sd_flushed_records&&sd_retry_pending&&line.off[0]==21);
  assert(sd_try_write_line());assert(g_sd_written_records==1&&!g_sd_flushed_records&&g_sd_ready);
  assert(!memcmp(disk+7,"originalUTC,exactROW\n",21));assert(file.fx_file_current_file_size==28);return 0;
 }
 assert(scenario==5);assert(sd_try_write_line());hidden_flush=1;
 assert(!sd_flush_close_current());assert(!g_sd_ready&&!g_sd_flushed_records&&!close_calls);
 assert(flight_sd_prepare_io()==FX_SUCCESS);assert(!g_sd_ready);
 assert(sd_flush_close_current());assert(g_sd_flushed_records==1&&g_sd_ready);return 0;
}
'''
 src=(root/'Core/Src/sdpipeline.c').read_text();body=src[src.index('static inline bool sd_io_failed(UINT status)'):src.index('static inline int sd_pipeline_init(')]
 return pre+m.stripped(root/'FileX/Target/fx_stm32_sd_driver_glue.c')+'\n'+m.stripped(root/'Core/Src/resilient_storage.c')+writer+body+main
class DriverWitness(unittest.TestCase):
 def test_actual_origin_and_pipeline_progress(self):
  with tempfile.TemporaryDirectory()as d:
   d=Path(d);src=d/'witness.c';src.write_text(program());exe=d/'witness'
   subprocess.run(['cc','-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-misleading-indentation','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(ROOT/'Core/Inc'),str(src),'-o',str(exe)],check=True)
   for scenario in range(7):
    with self.subTest(scenario=scenario):subprocess.run([str(exe),str(scenario)],check=True,env=dict(os.environ,ASAN_OPTIONS='detect_leaks=1:abort_on_error=1',UBSAN_OPTIONS='halt_on_error=1'))

"""Execute the real launch-file boundary, including partial I/O and tick wrap."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class LandedWindow(unittest.TestCase):
 def test_exact_deadline_wrap_repeated_transition_and_retry_split(self):
  source=(ROOT/'Core/Src/sdpipeline.c').read_text()
  clocks=source[source.index('void sd_launch_landed('):source.index('static bool sd_writer_slot;')]
  writer=source[source.index('static inline bool sd_try_write_line(void)'):source.index('static inline void sd_pipeline_shutdown(',source.index('static inline bool sd_try_write_line(void)'))]
  code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sd_file_limit.h"
#define WE_ARE_SO_BACK 0
#define FX_SUCCESS 0
#define sweetbench_start(x) ((void)0)
#define sweetbench_catch(x) ((void)0)
typedef unsigned UINT;
typedef unsigned long ULONG;
typedef uint16_t fu16;
struct {ULONG fx_file_current_file_size;} file={7};
struct {unsigned lock;fu16 off[2];bool free,cur;} line;
char sdbuf[2][64],normal[128],launch[128]="header\n";
char *disk=launch;
unsigned pos=7,flushed,closed,opened,mask,tick;
unsigned sd_buffer_records[2],g_sd_written_records,g_sd_flushed_records,g_sd_write_errors;
bool sd_launch_pending,sd_launch_active=true,sd_writer_slot,sd_retry_pending;
bool sd_landed_seen,sd_launch_completed,sd_cutoff_set,sd_cutoff_slot;
uint32_t sd_landed_tick,sd_cutoff_records,g_sd_launch_closed;
fu16 sd_cutoff_offset,sd_writer_skip;
unsigned sd_file_open=1,sdio_disk;
ULONG sd_retry_offset;
bool fail_write,fail_flush,fail_close,fail_open;
unsigned now_ms(void){return tick;}
unsigned __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(unsigned n){mask=n;}
void fc_lock(unsigned *lock){assert(!*lock);*lock=1;}
void fc_unlock(unsigned *lock){assert(*lock);*lock=0;}
unsigned fx_file_seek(void *f,unsigned at){(void)f;pos=at;return 0;}
unsigned fx_file_write(void *f,void *data,unsigned len){(void)f;assert(!line.lock);
 if(fail_write){fail_write=false;memcpy(disk+pos,data,2);pos+=2;file.fx_file_current_file_size=pos;return 1;}
 memcpy(disk+pos,data,len);pos+=len;file.fx_file_current_file_size=pos;return 0;}
unsigned fx_media_flush(void *f){(void)f;++flushed;return fail_flush;}
unsigned fx_file_close(void *f){(void)f;++closed;return fail_close;}
static int sd_pipeline_open(const char *why){(void)why;++opened;assert(!sd_launch_active);
 if(fail_open)return 1;sd_file_open=1;file.fx_file_current_file_size=7;
 disk=normal;memcpy(disk,"header\n",7);return 0;}
'''+clocks+writer+r'''
int main(void){
 strcpy(sdbuf[0],"grace\n");line.off[0]=6;sd_buffer_records[0]=1;
 tick=UINT32_MAX-10000;sd_launch_landed(tick);assert(!mask);
 uint32_t first=sd_landed_tick;
 tick=first+19999;sd_launch_landed(tick);assert(sd_landed_tick==first);
 sd_mark_launch_cutoff();assert(!sd_cutoff_set);
 tick=first+20000;sd_mark_launch_cutoff();assert(sd_cutoff_set&&sd_cutoff_offset==6&&sd_cutoff_records==1);
 strcat(sdbuf[0],"ordinary\n");line.off[0]=15;sd_buffer_records[0]=2;
 /* Producer has handed off this buffer; its successor is retained separately. */
 line.cur=1;line.free=false;
 fail_write=true;assert(!sd_try_write_line());assert(!sd_writer_skip&&sd_retry_pending);
 fail_flush=true;assert(!sd_try_write_line());
 assert(sd_writer_skip==6&&g_sd_written_records==1&&sd_launch_active);
 assert(!memcmp(launch,"header\ngrace\n",13));
 fail_flush=false;fail_close=true;assert(!sd_try_write_line());
 assert(sd_writer_skip==6&&g_sd_written_records==1&&sd_cutoff_set);
 fail_close=false;fail_open=true;assert(!sd_try_write_line());
 assert(!sd_launch_active&&sd_launch_completed&&!sd_cutoff_set&&g_sd_launch_closed==1);
 assert(sd_writer_skip==6&&line.off[0]==15&&sd_buffer_records[0]==1);
 fail_open=false;fail_write=true;assert(!sd_try_write_line());
 assert(sd_retry_pending&&sd_writer_skip==6);
 assert(sd_try_write_line());assert(!memcmp(normal,"header\nordinary\n",16));
 assert(!memcmp(launch,"header\ngrace\n",13)&&g_sd_written_records==2);
 assert(line.free&&!line.off[0]&&!sd_writer_skip&&!sd_retry_pending);
 tick+=20000;sd_mark_launch_cutoff();assert(!sd_cutoff_set&&g_sd_launch_closed==1);
 /* Empty launch at its deadline still closes and does not need a future row. */
 sd_launch_completed=false;sd_launch_active=true;sd_file_open=1;line.cur=sd_writer_slot;
 sd_mark_launch_cutoff();assert(sd_cutoff_set&&sd_cutoff_offset==0);
 assert(sd_try_write_line());assert(g_sd_launch_closed==2&&!sd_launch_active);
}
'''
  with tempfile.TemporaryDirectory() as tmp:
   exe=Path(tmp)/'landed-window'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=code,text=True,check=True)
   subprocess.run([str(exe)],check=True)
   for old,new in [('>= 20000U','>= 19999U'),('>= 20000U','>= 20001U'),
                   ('if (!sd_landed_seen) {','if (true) {')]:
    mutant=code.replace(old,new);self.assertNotEqual(mutant,code)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=mutant,text=True,check=True)
    failed=subprocess.run([str(exe)],capture_output=True,text=True)
    self.assertNotEqual(failed.returncode,0,'deadline negative control survived')

import pathlib,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
class SdWriterRetryTests(unittest.TestCase):
 def test_partial_failure_retains_buffer_and_shutdown_drains_tail(self):
  source=(ROOT/'Core/Src/sdpipeline.c').read_text()
  funcs=source[source.index('static inline bool sd_try_write_line(void)'):source.index('static inline int sd_pipeline_init(')]
  harness=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sd_file_limit.h"
#define WE_ARE_SO_BACK 0
typedef unsigned UINT;
typedef unsigned long ULONG;
typedef uint16_t fu16;
#define FX_SUCCESS 0
#define id "SD "
#define sweetbench_start(x) ((void)0)
#define sweetbench_catch(x) ((void)0)
#define log_err(...) ((void)0)
struct {ULONG fx_file_current_file_size;} file;
struct {int lock; bool free; fu16 off[2];} line;
char sdbuf[2][128];
uint32_t sd_buffer_records[2],g_sd_written_records,g_sd_flushed_records;
uint32_t g_sd_write_errors,g_sd_shutdown_lost_records;
bool sd_writer_slot,sd_retry_pending,sd_accepting,sd_file_open=true;
ULONG sd_retry_offset;
int sdio_disk,fail_writes,fail_flush;
unsigned sleeps,closes,opens;
char disk[256]; size_t position;
void fc_lock(int *p){assert(!*p);*p=1;}
void fc_unlock(int *p){assert(*p);*p=0;}
UINT fx_file_seek(void *f,ULONG at){(void)f;position=at;return 0;}
UINT fx_file_write(void *f,void *buf,ULONG n){
 (void)f;assert(!line.lock);
 if(fail_writes){--fail_writes;memcpy(disk+position,buf,2);position+=2;
  if(position>file.fx_file_current_file_size)file.fx_file_current_file_size=position;
  return 1;}
 memcpy(disk+position,buf,n);position+=n;
 if(position>file.fx_file_current_file_size)file.fx_file_current_file_size=position;
 return 0;
}
UINT fx_media_flush(void *m){(void)m;return fail_flush;}
UINT fx_file_close(void *f){(void)f;++closes;return 0;}
UINT fx_media_close(void *m){(void)m;++closes;return 0;}
void tx_thread_sleep(unsigned n){sleeps+=n;}
static inline int sd_pipeline_open(const char *why){(void)why;
 ++opens;sd_file_open=true;file.fx_file_current_file_size=7;
 memcpy(disk,"header\n",7);return 0;}

''' + funcs + r'''
int main(void){
 strcpy(disk,"header\n");file.fx_file_current_file_size=7;
 strcpy(sdbuf[0],"first\n");strcpy(sdbuf[1],"tail\n");
 line.off[0]=6;line.off[1]=5;sd_buffer_records[0]=2;sd_buffer_records[1]=1;
 fail_writes=1;
 assert(!sd_try_write_line());
 assert(line.off[0]==6&&!line.free&&!sd_writer_slot&&sd_retry_pending);
 assert(g_sd_written_records==0&&g_sd_write_errors==1);
 assert(sd_try_write_line());
 assert(!memcmp(disk,"header\nfirst\n",13));
 assert(file.fx_file_current_file_size==13&&line.off[0]==0&&line.free&&sd_writer_slot);
 assert(g_sd_written_records==2);
 /* Active short buffer has never been handed off by a producer. */
 sd_pipeline_shutdown("test");
 assert(!memcmp(disk,"header\nfirst\ntail\n",18));
 assert(file.fx_file_current_file_size==18&&g_sd_written_records==3);
 assert(g_sd_flushed_records==3&&!g_sd_shutdown_lost_records&&closes==2);
 /* Permanent I/O failure must terminate bounded shutdown and account loss. */
 strcpy(sdbuf[sd_writer_slot],"lost\n");line.off[sd_writer_slot]=5;
 sd_buffer_records[sd_writer_slot]=1;fail_writes=99;fail_flush=1;
 sd_pipeline_shutdown("failed");
 assert(g_sd_shutdown_lost_records==1&&sleeps==2);
 assert(g_sd_flushed_records==3&&g_sd_write_errors==5);
 /* Rotation retains a complete queued row and its record count. */
 fail_writes=0;fail_flush=0;sd_retry_pending=false;sd_file_open=true;
 file.fx_file_current_file_size=SD_LOG_PART_LIMIT_BYTES-2;
 unsigned previous_opens=opens;
 assert(sd_try_write_line());
 assert(opens==previous_opens+1&&file.fx_file_current_file_size==12);
 assert(!memcmp(disk,"header\nlost\n",12)&&g_sd_written_records==4);

}
'''
  with tempfile.TemporaryDirectory() as d:
   binary=pathlib.Path(d)/'writer'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
   subprocess.run([str(binary)],check=True)
 def test_task_retries_rotation_failure_without_waiting_for_a_new_buffer(self):
  source=(ROOT/'Core/Src/sdpipeline.c').read_text()
  task=source[source.index('void sd_pipeline_task()'):]
  harness=r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
typedef unsigned UINT;
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER 0xffffffffU
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define FX_SUCCESS 0U
#define SD_FLUSH_INTERVAL 30000U
#define SDFlush 0
#define WE_ARE_SO_BACK 0
#define id "SD "
#define MrAnalog(condition) while(!(condition))
#define load(p,o) stopped
#define option(x) 1U
#define log_err(...) ((void)0)
struct {unsigned full;} line;
bool stopped,sd_retry_pending;
unsigned sema_calls,write_calls,sleep_calls,shutdown_calls;
unsigned g_sd_flushed_records,g_sd_written_records,g_sd_write_errors,sdio_disk;
unsigned tx_semaphore_get(void *s,unsigned timeout){(void)s;(void)timeout;
 if(++sema_calls>1)exit(42);return TX_SUCCESS;}
int sd_pipeline_init(const char *s){(void)s;return 0;}
bool sd_try_write_line(void){
 if(++write_calls==1)return false; /* rotation/open failed before retry offset exists */
 stopped=true;return true;}
void tx_thread_sleep(unsigned ticks){assert(ticks>0);++sleep_calls;}
void sd_pipeline_shutdown(const char *s){(void)s;++shutdown_calls;}
bool timer_probe(unsigned timer,unsigned interval){(void)timer;(void)interval;return false;}
unsigned fx_media_flush(void *m){(void)m;return 0;}
''' +task+r'''
int main(void){sd_pipeline_task();assert(sema_calls==1&&write_calls==2&&sleep_calls==1&&shutdown_calls==1);}
'''
  with tempfile.TemporaryDirectory() as d:
   binary=pathlib.Path(d)/'pending'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
   self.assertEqual(subprocess.run([str(binary)]).returncode,0)
 def test_startup_retries_create_and_partial_header_without_recreating_semaphore(self):
  source=(ROOT/'Core/Src/sdpipeline.c').read_text()
  funcs=source[source.index('static inline int sd_pipeline_init('):source.index('\nvoid sd_pipeline_task()')]
  harness=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sd_record_format.h"
typedef unsigned UINT;
#define TX_SUCCESS 0U
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define FX_SUCCESS 0U
#define FX_ALREADY_CREATED 2U
#define FX_OPEN_FOR_WRITE 3U
#define WE_ARE_SO_BACK 0
#define IT_IS_NOW_OVER 1
#define id "SD "
#define fetch_and(...) (stopped=false)
#define load(...) stopped
#define option(...) 1U
#define log_err(...) ((void)0)
struct {bool free;unsigned full;} line;
struct {const char *fx_file_name;} file;
uint32_t sd_buffer_records[2],g_sd_write_errors;
bool sd_writer_slot,sd_retry_pending,sd_accepting,stopped,stop_on_sleep;
uint8_t sd_file_open;
int sdio_disk,create_fail,header_fail;
unsigned sema_creates,sema_deletes,creates,opens,writes,sleeps,shutdowns;
char disk[512];unsigned position;
static inline int sd_pipeline_open(const char *why);
const char *unique_sd_filename(uint32_t suffix){(void)suffix;return "test.csv";}
UINT tx_semaphore_create(unsigned *s,const char *name,unsigned value){assert(!*s);*s=1;++sema_creates;return 0;}
UINT tx_semaphore_delete(unsigned *s){assert(*s);*s=0;++sema_deletes;return 0;}
void tx_thread_sleep(unsigned ticks){assert(ticks==1000);++sleeps;if(stop_on_sleep)stopped=true;}
void sd_pipeline_shutdown(const char *why){++shutdowns;sd_file_open=0;}
UINT fx_file_create(void *media,const char *name){++creates;if(create_fail){--create_fail;return 1;}return 0;}
UINT fx_file_open(void *media,void *f,const char *name,unsigned mode){++opens;position=0;return 0;}
UINT fx_file_seek(void *f,unsigned at){position=at;return 0;}
UINT fx_file_write(void *f,void *data,unsigned length){++writes;
 if(header_fail){--header_fail;memcpy(disk+position,data,4);position+=4;return 1;}
 memcpy(disk+position,data,length);position+=length;return 0;}
''' +funcs+r'''
int main(void){
 create_fail=2;header_fail=1;
 assert(sd_pipeline_init("test")==WE_ARE_SO_BACK);
 assert(sd_accepting&&sd_file_open==1&&sema_creates==1&&sema_deletes==0);
 assert(creates==3&&opens==1&&writes==2&&sleeps==3&&g_sd_write_errors==3);
 assert(position==strlen(SD_CSV_HEADER)&&!memcmp(disk,SD_CSV_HEADER,position));
 tx_semaphore_delete(&line.full);sd_file_open=0;sd_accepting=false;
 create_fail=99;stop_on_sleep=true;
 unsigned oldcreates=creates,oldsleeps=sleeps;
 assert(sd_pipeline_init("stop")==IT_IS_NOW_OVER);
 assert(!sd_accepting&&shutdowns==1&&sema_creates==2&&sema_deletes==1&&line.full==1);
 assert(creates==oldcreates+1&&sleeps==oldsleeps+1);
}
'''
  with tempfile.TemporaryDirectory() as d:
   binary=pathlib.Path(d)/'startup'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-function','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
   subprocess.run([str(binary)],check=True)

if __name__=='__main__':unittest.main()

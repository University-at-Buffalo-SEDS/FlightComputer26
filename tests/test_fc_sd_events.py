"""Run the production GPIO audit FIFO and SD tail/launch ownership paths."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

def run_c(code):
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'check'
        subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
                        '-Wno-unused-function','-I',str(ROOT/'Core/Inc'),
                        '-x','c','-','-o',str(exe)],input=code,text=True,check=True)
        subprocess.run([str(exe)],check=True)

class FlightLogTests(unittest.TestCase):
    def test_irq_fifo_retains_original_timestamps_and_does_not_extend_pulses(self):
        src=(ROOT/'Core/Src/deployment_pulse.c').read_text()
        src='\n'.join(x for x in src.splitlines() if not x.startswith('#include'))
        code=r'''
#include <assert.h>
#include <stdatomic.h>
#include "fc_sd_events.h"
#define TX_TIMER_TICKS_PER_SECOND 1000
#define CO2_ASSERT_INTERVAL 5
#define REEF_ASSERT_INTERVAL 3
#define option(x) (x)
#define Parachute_Expanded 1
#define REEF_Asserted 2
#define Parachute_Deployed 4
#define CO2_Asserted 8
atomic_uint_fast32_t g_conf;
unsigned clock_ms, mask,co2,reef;
bool permitted=true;
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t x){mask=x;}
uint32_t HAL_GetTick(void){return clock_ms;}
void HAL_IncTick(void){++clock_ms;}
void HAL_SuspendTick(void){}
bool fc_watchdog_can_actuate(void){return permitted;}
bool fc_watchdog_record_deployment(bool r){(void)r;return permitted;}
void reef_high(void){assert(mask);reef=1;}
void co2_high(void){assert(mask);co2=1;}
void reef_low(void){reef=0;}
void co2_low(void){co2=0;}
'''+src+r'''
int main(void){
 fc_sd_event event,again;
 clock_ms=0xfffffffeU;
 deployment_pulse_start(false);
 assert(fc_sd_peek_event(&event)&&event.kind==FC_LOG_PYRO_HIGH&&event.tick==0xfffffffeU);
 /* A failed/blocked consumer must leave the head owned and timestamp intact. */
 clock_ms=0xffffffffU;assert(fc_sd_peek_event(&again));
 assert(event.tick==again.tick&&event.kind==again.kind);
 fc_sd_consume_event();
 deployment_pulse_start(false);assert(g_sd_event_pending==1);
 assert(fc_sd_peek_event(&event)&&event.kind==FC_LOG_PYRO_ACTIVE);fc_sd_consume_event();
 for(unsigned i=0;i<5;i++)fc_runtime_tick();
 assert(!co2&&g_deployment_pulse_completions==1);
 assert(fc_sd_peek_event(&event)&&event.kind==FC_LOG_PYRO_LOW&&event.tick==4);fc_sd_consume_event();
 permitted=false;deployment_pulse_start(true);
 assert(!reef&&fc_sd_peek_event(&event)&&event.kind==FC_LOG_PYRO_REJECTED&&event.reef);fc_sd_consume_event();
 for(unsigned i=0;i<32;i++){clock_ms=i;fc_sd_capture_event(FC_LOG_PYRO_HIGH,true);}
 assert(g_sd_event_pending==32&&g_sd_event_queue_peak==32);
 fc_sd_capture_event(FC_LOG_PYRO_LOW,true);
 assert(g_sd_event_queue_drops==1&&g_sd_event_pending==32);
 for(unsigned i=0;i<32;i++){assert(fc_sd_peek_event(&event)&&event.tick==i);fc_sd_consume_event();}
 assert(!fc_sd_peek_event(&event)&&!mask);
}
'''
        run_c(code)

    def test_tail_handoff_waits_for_writer_and_event_head_retries(self):
        src=(ROOT/'Core/Src/sdpipeline.c').read_text()
        funcs=src[src.index('static void sd_service_events('):src.index('/* Task */')]
        code=r'''
#include <assert.h>
#include <string.h>
#include "fc_sd_events.h"
#include "board_uptime.h"
struct {unsigned lock,cur,off[2];bool free;} line;
bool sd_launch_pending,sd_launch_active,allow_append;
bool sd_launch_completed,sd_cutoff_set,sd_landed_seen;
uint32_t sd_landed_tick;
enum {Landed=10};
unsigned current_state;
unsigned current(void){return current_state;}
unsigned consumed,attempts;
uint64_t seen_uptime;
const char *seen_label;
fc_sd_event pending={0xfffffffeU,FC_LOG_LAUNCH_DETECTED,0};
bool fc_sd_peek_event(fc_sd_event *event){if(consumed)return false;*event=pending;return true;}
void fc_sd_consume_event(void){++consumed;}
uint64_t board_uptime_ticks(void){return UINT64_C(4294967299);}
bool sd_append_ready(const char *label,const char *value,const char *raw,uint64_t uptime,bool draining){
 (void)raw;(void)draining;assert(!strcmp(value,"Launch") || !strcmp(value,"drogue"));
 ++attempts;seen_label=label;seen_uptime=uptime;return allow_append;}
void fc_lock(unsigned *lock){assert(!*lock);*lock=1;}
void fc_unlock(unsigned *lock){assert(*lock);*lock=0;}
'''+funcs+r'''
int main(void){
 sd_service_events(false);assert(!consumed&&!sd_launch_pending&&attempts==1);
 allow_append=true;sd_service_events(false);
 assert(consumed==1&&sd_launch_pending&&!strcmp(seen_label,"launch_detected"));
 assert(seen_uptime==UINT64_C(4294967294));
 sd_launch_pending=false;sd_launch_completed=true;sd_landed_seen=true;
 sd_landed_tick=pending.tick-1;current_state=Landed;consumed=0;
 sd_service_events(false);assert(consumed&&!sd_launch_pending&&sd_landed_seen);
 current_state=0;consumed=0;sd_service_events(false);
 assert(consumed&&sd_launch_pending&&!sd_launch_completed&&!sd_landed_seen);
 sd_launch_pending=false;sd_launch_completed=true;sd_launch_active=false;
 current_state=Landed;pending.kind=FC_LOG_PYRO_HIGH;consumed=0;
 sd_service_events(false);
 assert(consumed&&!sd_launch_pending&&!sd_launch_active&&!strcmp(seen_label,"pyro_gpio_high"));
 line.free=true;line.cur=0;line.off[0]=17;sd_handoff_tail();
 assert(!line.free&&line.cur==1&&line.off[0]==17);
 line.off[1]=12;sd_handoff_tail();assert(line.cur==1&&line.off[0]==17);
 line.off[0]=0;line.free=true;sd_handoff_tail();
 assert(!line.free&&line.cur==0&&line.off[1]==12&&!line.lock);
}
'''
        run_c(code)

    def test_launch_rotation_retains_old_retry_and_failed_transition(self):
        src=(ROOT/'Core/Src/sdpipeline.c').read_text()
        funcs=src[src.index('static inline bool sd_try_write_line(void)'):src.index('static inline void sd_pipeline_shutdown(',src.index('static inline bool sd_try_write_line(void)'))]
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
typedef unsigned short fu16;
struct {ULONG fx_file_current_file_size;} file={7};
struct {unsigned lock;fu16 off[2];bool free;} line;
char sdbuf[2][32],normal[80]="header\n",launch[80];
char *disk=normal;
unsigned pos=7,flushed,closed,opened;
unsigned sd_buffer_records[2],g_sd_written_records,g_sd_flushed_records,g_sd_write_errors;
bool sd_launch_pending,sd_launch_active,sd_writer_slot,sd_retry_pending;
bool sd_launch_completed,sd_cutoff_set,sd_cutoff_slot;
uint32_t sd_cutoff_records,g_sd_launch_closed;
uint16_t sd_cutoff_offset,sd_writer_skip;
unsigned sd_file_open=1,sdio_disk;
ULONG sd_retry_offset;
bool fail_write,fail_flush,fail_close,fail_open;
void fc_lock(unsigned *lock){assert(!*lock);*lock=1;}
void fc_unlock(unsigned *lock){assert(*lock);*lock=0;}
unsigned fx_file_seek(void *f,unsigned at){(void)f;pos=at;return 0;}
unsigned fx_file_write(void *f,void *data,unsigned len){(void)f;
 if(fail_write){fail_write=false;memcpy(disk+pos,data,2);pos+=2;file.fx_file_current_file_size=pos;return 1;}
 memcpy(disk+pos,data,len);pos+=len;file.fx_file_current_file_size=pos;return 0;}
unsigned fx_media_flush(void *f){(void)f;++flushed;return fail_flush;}
unsigned fx_file_close(void *f){(void)f;++closed;return fail_close;}
static int sd_pipeline_open(const char *why){(void)why;++opened;assert(sd_launch_active);
 if(fail_open)return 1;sd_file_open=1;file.fx_file_current_file_size=7;
 disk=launch;memcpy(disk,"header\n",7);return 0;}
'''+funcs+r'''
int main(void){
 strcpy(sdbuf[0],"before\n");line.off[0]=7;sd_buffer_records[0]=1;
 strcpy(sdbuf[1],"launch\n");line.off[1]=7;sd_buffer_records[1]=1;
 fail_write=true;assert(!sd_try_write_line());
 sd_launch_pending=true;
 assert(sd_try_write_line());assert(!sd_launch_active&&!opened&&!flushed);
 assert(!memcmp(normal,"header\nbefore\n",14));
 fail_flush=true;assert(!sd_try_write_line());
 assert(sd_launch_pending&&!sd_launch_active&&line.off[1]==7&&!closed);
 fail_flush=false;fail_close=true;assert(!sd_try_write_line());
 assert(sd_launch_pending&&!sd_launch_active&&line.off[1]==7);
 fail_close=false;fail_open=true;assert(!sd_try_write_line());
 assert(!sd_file_open&&sd_launch_active&&!sd_retry_pending&&line.off[1]==7);
 fail_open=false;assert(sd_try_write_line());
 assert(!memcmp(launch,"header\nlaunch\n",14)&&g_sd_written_records==2);
 assert(g_sd_flushed_records==1&&g_sd_write_errors==4);
}
'''
        run_c(code)

    def test_launch_filename_and_unsynchronized_collision_suffix(self):
        src=(ROOT/'Core/Src/sdpipeline.c').read_text()
        func=src[src.index('static inline char *unique_sd_filename'):src.index('/* API */')]
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define SEDS_LOG_FILENAME "FLIGHT"
bool sd_launch_active;
uint64_t utc;
unsigned monotonic;
uint64_t telemetry_unix_s(void){return utc;}
unsigned now_ms(void){return monotonic;}
'''+func+r'''
int main(void){
 monotonic=1;
 assert(!strcmp(unique_sd_filename(0),"FLIGHT-1-000.csv"));
 assert(!strcmp(unique_sd_filename(999),"FLIGHT-1-999.csv"));
 sd_launch_active=true;
 assert(!strcmp(unique_sd_filename(2),"FLIGHT_LAUNCH-1-002.csv"));
 utc=1793232000;
 assert(!strcmp(unique_sd_filename(0),"FLIGHT_LAUNCH-1793232000-000.csv"));
}
'''
        run_c(code)

    def test_each_kalman_output_is_saved_while_radio_timer_is_not_due(self):
        src=(ROOT/'Core/Src/evaluation.c').read_text()
        func=src[src.index('static inline void propel_kalman_state('):src.index('void evaluate_rocket_state(')]
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define SD_AVAILABLE
#define conditional
#define option(x) 1U
#define MAX_STATE 6
#define EKF_STATE 6
#define DKF_STATE 4
#define STATE_HISTORY_MASK 7
#define SEDS_DT_ASCENT_LOCAL 10
#define SEDS_DT_ASCENT_STATE 11
#define SEDS_DT_DESCENT_LOCAL 12
#define SEDS_DT_DESCENT_STATE 13
#define KFRemote 1
#define KFLocal 2
typedef unsigned fu16;
typedef unsigned fu32;
typedef struct {float a,b,c,d;} quat;
quat qv={1,2,3,4};
struct {struct {float lon,lat;} gps;float alt,vel;} state={ {5,6},7,8};
struct {unsigned idx;} sm;
struct {unsigned sd,gnd;} rates={50,50};
#define svec(x) state
unsigned written,seen_type,seen_count;
float seen[6];
bool timer_probe_ground(unsigned timer,unsigned interval){(void)timer;(void)interval;return false;}
void sd_append_f32(unsigned type,const float *data,unsigned count){++written;seen_type=type;seen_count=count;memcpy(seen,data,count*sizeof(float));}
void log_periodic_f32(unsigned type,unsigned count,const float *data){(void)type;(void)count;(void)data;assert(false);}
'''+func+r'''
int main(void){
 propel_kalman_state(1);
 assert(written==1&&seen_type==SEDS_DT_ASCENT_LOCAL&&seen_count==6);
 assert(seen[0]==1&&seen[3]==4&&seen[4]==7&&seen[5]==8);
 propel_kalman_state(0);
 assert(written==2&&seen_type==SEDS_DT_DESCENT_LOCAL&&seen_count==4);
 assert(seen[0]==5&&seen[1]==6&&seen[2]==7&&seen[3]==8&&sm.idx==2);
}
'''
        run_c(code)

    def test_record_clock_snapshot_preserves_uptime_and_never_fabricates_utc(self):
        src=(ROOT/'Core/Src/sdpipeline.c').read_text()
        clocks=src[src.index('static uint64_t sd_clock_utc_ms,'):src.index('/* All producers share one bounded append path;')]
        run_c(r'''
#include <assert.h>
#include <stdint.h>
#define TELEMETRY_ENABLED
static uint64_t uptime, network;
static unsigned mask;
uint64_t board_uptime_ticks(void){return uptime;}
uint64_t telemetry_unix_ms(void){assert(!mask);return network;}
unsigned __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(unsigned value){mask=value;}
'''+clocks+r'''
int main(void){
 uptime=1000;network=1793232000000ULL;sd_refresh_clock();
 uptime=1050;assert(sd_record_utc(1025)==network+25);
 mask=1;assert(sd_record_utc(1040)==network+40&&mask==1);mask=0;
 assert(sd_record_utc(999)==0); /* No extrapolation across unknown clock corrections. */
 uptime=3001;assert(sd_record_utc(1050)==0); /* Expired lease. */
 uptime=4000;network=0;sd_refresh_clock();assert(sd_record_utc(4000)==0);
 uptime=(UINT64_C(1)<<32)+20;network=1793232001000ULL;sd_refresh_clock();
 uptime+=50;assert(sd_record_utc((UINT64_C(1)<<32)+45)==network+25);
 network-=100000;sd_refresh_clock();uptime+=1;assert(sd_record_utc(uptime)==network+1);
 uptime=9000;network=UINT64_MAX-2;sd_refresh_clock();
 uptime=9005;assert(sd_record_utc(9005)==0); /* Reject UTC overflow. */
 uptime=8999;assert(sd_record_utc(9000)==0);assert(!mask);
 return 0;
}
''')

if __name__=='__main__':unittest.main()

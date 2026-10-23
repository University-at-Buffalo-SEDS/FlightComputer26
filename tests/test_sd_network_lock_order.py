import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class SdNetworkLockTests(unittest.TestCase):
    def test_busy_sd_writer_retains_ownership_of_its_buffer(self):
        source = (ROOT / "Core/Src/sdpipeline.c").read_text()
        function = source[source.index("static inline void sd_release_notify("):source.index("static inline constexpr char *seds_msg(")]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint16_t fu16;
#define SD_POST_MARGIN 100
struct { int lock, full; bool cur, free; fu16 off[2]; } line;
unsigned posts;
unsigned sd_buffer_records[2], g_sd_enqueued_records;
void fc_unlock(int *lock) { *lock=0; }
void fc_concede(int *lock) { *lock=0; }
void tx_semaphore_put(int *semaphore) { (void)semaphore; posts++; }
''' + function + r'''
int main(void) {
    line.free=true; line.lock=1;
    sd_release_notify(100,200);
    assert(line.cur==1 && !line.free && posts==1);
    /* Buffer zero is still owned by the SD writer. */
    line.lock=1; sd_release_notify(100,200);
    assert(line.cur==1 && !line.free && posts==1);
    assert(line.off[0]==100 && line.off[1]==100);
    /* SD completes zero; producer can now hand off one safely. */
    line.off[0]=0; line.free=true; line.lock=1;
    sd_release_notify(1,100);
    assert(line.cur==0 && !line.free && posts==2 && !line.lock);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            binary = pathlib.Path(directory) / "buffer-ownership"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(binary)],
                           input=harness, text=True, check=True)
            subprocess.run([str(binary)], check=True)

    def test_actual_sd_append_functions_do_not_invert_network_lock_or_spin(self):
        source = (ROOT / "Core/Src/sdpipeline.c").read_text()
        functions = source[source.index("/* All producers share one bounded append path."):source.index("void sd_conclude(")]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef unsigned char fu8;
typedef unsigned short fu16;
typedef uint32_t fu32;
typedef int SedsDataType;
#include "sd_record_format.h"
#include "sd_vector_format.h"
struct { uint32_t sd; } rates = {5};
#define F32_TO_STR_MAX_SIZE 128
#define SD_BUFFER_SIZE 1024
#define load(p,o) stop_flag
#define option(o) 1
struct { int lock; unsigned cur; fu16 off[2]; } line;
char sdbuf[2][SD_BUFFER_SIZE];
unsigned g_sd_format_errors, g_sd_lock_contention_drops, sd_csv_sequence;
bool expect_contention, stop_during_clock, stop_flag;
bool sd_accepting=true;
unsigned g_sd_unavailable_drops;
int clock_calls;
int seds_ftoa4(char *out,fu16 n,const float *data,fu8 count) {
    (void)data; (void)count; return snprintf(out,n,"1.0");
}
const char *seds_f32(SedsDataType ty) { (void)ty; return "IMU"; }
const char *seds_msg(SedsDataType ty) { (void)ty; return "Message"; }
uint64_t telemetry_unix_ms(void) {
    assert(!line.lock); ++clock_calls; if(stop_during_clock) stop_flag=true; return 123000;
}
uint64_t board_uptime_ticks(void) { return 456; }
void fc_lock(int *lock) { assert(!*lock); *lock=1; }
void fc_concede(int *lock) { *lock=0; }
bool fc_trylock(int *lock) {
    if (expect_contention || *lock) return false;
    *lock=1; return true;
}
void sd_release_notify(fu16 written,fu16 rem) {
    assert(line.lock); assert(written < rem);
    line.off[line.cur] += written; line.lock=0;
}
''' + functions + r'''
int main(void) {
    float value=1.0f;
    sd_append_f32(0,&value,1);
    sd_append_string(0,"callback");
    assert(clock_calls==2 && !line.lock);
    assert(strstr(sdbuf[0],"123000,456,IMU,\"1.0\",\"\",\"\",\"\",network,,,456,1000,1"));
    assert(strstr(sdbuf[0],"123000,456,Message,\"callback\",\"\",\"\",\"\",network,,,456,1000,2"));
    fu16 before=line.off[0]; expect_contention=true;
    sd_append_f32(0,&value,1); sd_append_string(0,"busy");
    assert(line.off[0]==before && g_sd_lock_contention_drops==2);
    expect_contention=false; sd_accepting=false;
    sd_append_f32(0,&value,1); sd_append_string(0,"not ready");
    assert(line.off[0]==before && g_sd_unavailable_drops==2 && clock_calls==4);
    sd_accepting=true; stop_during_clock=true;
    sd_append_string(0,"raced stop");
    assert(stop_flag && !line.lock && line.off[0]==before);
    stop_flag=false; stop_during_clock=false;
    float raw[]={-32768.0f,0.0f,32767.0f}, converted[]={-2.0f,0.0f,1.999f};
    sd_append_sensor(1,raw,converted,3);
    assert(!line.lock && line.off[0]>before);
    assert(strstr(sdbuf[0],"gyroscope_sample"));
    before=line.off[0];
    sd_append_sensor(1,raw,converted,3); /* Same acquisition cadence: no duplicate. */
    sd_append_sensor(3,raw,converted,3); /* Invalid sensor cannot index timers. */
    assert(line.off[0]==before);
    expect_contention=true;
    sd_append_sensor(2,raw,converted,3);
    assert(!line.lock && line.off[0]==before && g_sd_lock_contention_drops==3);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            binary = pathlib.Path(directory) / "lock-order"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(binary)],
                           input=harness, text=True, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()

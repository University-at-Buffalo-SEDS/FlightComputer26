"""Actual SDK-free cache core; NVM is an explicit fault-injection boundary.

No SDK-shaped types, router success or recovery authority is fabricated here.
The firmware factory/full recovery tests separately bind the strict journal.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    'main.h': '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
    'persistent_store.h': '''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
''',
    'sedsnet.h': '#error SDK header leaked into the SDK-free local cache\n',
    'sedsnet_config.h': '#error Network configuration leaked into local cache\n',
    'board_recovery_transport.h': '#error Network transport leaked into local cache\n',
}
HARNESS = r'''
#include "flight_state_cache.h"
#include "persistent_store.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>
extern volatile uint32_t g_flight_state_cache_errors,g_flight_state_cache_writes;
static uint8_t stored=5;static unsigned reads,writes;
static bool missing,bad_init,bad_write,bad_read,mismatch,short_read;
uint32_t HAL_GetTick(void){return 123;}
int persistent_store_init(void){return bad_init?2:0;}
int persistent_store_get(uint32_t key,void*out,size_t*size){
 assert(key==0x46535445U&&*size==1);++reads;
 if(bad_read)return 2;
 if(missing)return 1;
 *(uint8_t*)out=mismatch?1:stored;if(short_read)*size=0;return 0;
}
int persistent_store_set(uint32_t key,const void*value,size_t size){
 assert(key==0x46535445U&&size==1);++writes;
 if(bad_write)return 2;
 stored=*(const uint8_t*)value;missing=false;return 0;
}
int main(int argc,char**argv){
 assert(argc==2);bool absent=!strcmp(argv[1],"absent");missing=absent;
 flight_state_cache_restore();
 assert(flight_state_cache_has_value()==!absent);
 if(!absent)assert(flight_state_cache_value()==5);
 unsigned before=reads;
 bad_init=!strcmp(argv[1],"init_failure");
 bad_write=!strcmp(argv[1],"write_failure");
 bad_read=!strcmp(argv[1],"read_failure");
 mismatch=!strcmp(argv[1],"mismatch");
 short_read=!strcmp(argv[1],"short_read");
 bool failed=bad_init||bad_write||bad_read||mismatch||short_read;
 assert((flight_state_cache_reset_startup()!=0)==failed);
 if(failed){
  assert(flight_state_cache_value()==5&&flight_state_cache_has_value());
  assert(g_flight_state_cache_writes==0&&g_flight_state_cache_errors==1);
  if(bad_init)assert(writes==0&&reads==before);
  else if(bad_write)assert(writes==1&&reads==before);
  else assert(writes==1&&reads==before+1);
 }else{
  assert(flight_state_cache_has_value()&&flight_state_cache_value()==0&&stored==0);
  assert(writes==1&&reads==before+1&&g_flight_state_cache_writes==1&&g_flight_state_cache_errors==0);
 }
 return 0;
}
'''


class SdkFreeLocalCacheTests(unittest.TestCase):
    def test_local_restore_and_verified_reset_without_sdk_headers(self):
        with tempfile.TemporaryDirectory(prefix='fc-local-cache-') as tmp:
            p = Path(tmp)
            for name, body in HEADERS.items():
                (p / name).write_text(body)
            (p / 'harness.c').write_text(HARNESS)
            exe = p / 'proof'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                            '-fno-pie', '-no-pie', '-I' + str(p),
                            '-I' + str(ROOT / 'Core/Inc'),
                            str(ROOT / 'Core/Src/flight_state_cache.c'),
                            str(p / 'harness.c'), '-o', str(exe)], check=True)
            for case in ('success', 'absent', 'init_failure', 'write_failure',
                         'read_failure', 'mismatch', 'short_read'):
                with self.subTest(case=case):
                    subprocess.run([str(exe), case], check=True)

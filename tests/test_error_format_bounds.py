import re
import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class ErrorFormatBoundsTests(unittest.TestCase):
    def test_long_message_and_error_paths(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        function=re.search(r'static SedsResult log_error_impl\(.*?\n\}',source,re.S).group()
        code=r'''
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_ERR 1
#define SEDS_BAD_ARG 2
#define SEDS_DT_TELEMETRY_ERROR 3
static struct {void*r;} g_router;
static unsigned locked, calls, queued, fail_init, fail_log, fail_format;
static size_t length;
static uint8_t received[512];
int init_telemetry_router(void){if(fail_init)return SEDS_ERR;g_router.r=&calls;return SEDS_OK;}
void telemetry_lock(void){assert(!locked);locked=1;}
void telemetry_unlock(void){assert(locked);locked=0;}
int seds_router_log_bytes_ex(void*r,int type,const uint8_t*data,size_t n,void*ts,uint8_t queue){
 assert(r&&type==3&&ts==NULL&&locked);assert(n<=512);memcpy(received,data,n);length=n;queued=queue;calls++;return fail_log?SEDS_ERR:SEDS_OK;
}
int format(char*b,size_t n,const char*f,va_list args){if(fail_format)return -1;return vsnprintf(b,n,f,args);}
#define vsnprintf format
''' + function + r'''
static int report(unsigned queue,const char*f,...){va_list a;va_start(a,f);int result=log_error_impl(queue,f,a);va_end(a);return result;}
int main(void){
 char large[2049];memset(large,'X',2048);large[2048]=0;
 assert(report(1,"%s",large)==SEDS_OK);assert(length==512&&queued==1&&!locked);
 for(unsigned i=0;i<512;i++)assert(received[i]=='X');
 large[512]=0;assert(report(0,"%s",large)==SEDS_OK);assert(length==512&&queued==0);
 assert(report(1,"value %d",42)==SEDS_OK);assert(length==8&&!memcmp(received,"value 42",8));
 unsigned before=calls;assert(report(0,NULL)==SEDS_BAD_ARG);assert(calls==before);
 g_router.r=NULL;fail_init=1;assert(report(1,"test")==SEDS_ERR);assert(calls==before&&!locked);
 fail_init=0;fail_log=1;assert(report(0,"test")==SEDS_ERR);assert(!locked);
 fail_log=0;fail_format=1;assert(report(1,"test")==SEDS_OK);assert(length==0&&!locked);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp);(path/'test.c').write_text(code)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(path/'test.c'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

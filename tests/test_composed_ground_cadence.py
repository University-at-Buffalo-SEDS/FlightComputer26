"""Execute the actual producer timer, numeric publishers and generic rate gate."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

def timer(name):
    s=(ROOT/'Core/Inc/fcapi.h').read_text();a=s.index('static inline bool '+name+'(');return s[a:s.index('\n}',a)+2]

def run(rate,duplicate=False):
    platform=(ROOT/'Core/Inc/platform.h').read_text();a=platform.index('#define log_periodic_f32(');macro=platform[a:platform.index('\n\n',a)]
    if duplicate:macro=macro.replace('log_telemetry_synchronous','log_telemetry_asynchronous')
    source=(ROOT/'Core/Src/telemetry.c').read_text();pub=source[source.index('SedsResult log_telemetry_synchronous'):source.index('SedsResult log_telemetry_string_asynchronous')]
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "telemetry_rate.h"
#include "sedsnet_config.h"
#define TELEMETRY_ENABLED 1
typedef uint32_t fu32;
typedef unsigned timer;
typedef int SedsResult;
enum { SEDS_OK, SEDS_BAD_ARG, SEDS_ERR };
volatile fu32 local_time[4];
unsigned source_tick,publish_tick,locks,calls;
static struct {void *r;} g_router={(void*)1};
unsigned now_ms(void){return source_tick;}
unsigned HAL_GetTick(void){return publish_tick;}
void telemetry_lock(void){assert(locks==0);++locks;}
void telemetry_unlock(void){assert(locks==1);--locks;}
int init_telemetry_router(void){return SEDS_OK;}
int seds_router_log_bytes(void *r,unsigned type,const void *p,size_t n){
 assert(r&&type==SEDS_DT_BAROMETER_DATA&&p&&n==3*sizeof(float)&&locks==1);++calls;return SEDS_OK;
}
'''+timer('timer_probe_periodic')+'\n'+timer('timer_probe_ground')+'\n'+pub+'\n'+macro+r'''
static void opportunity(unsigned configured,unsigned tick){
 source_tick=tick;
 if(timer_probe_ground(0,configured)){
  const float value[3]={1.0f,2.0f,3.0f};
  /* Independent send-time jitter after producer scheduling / router lock. */
  publish_tick=tick+((tick/fc_telemetry_period_ms())&1U);
  log_periodic_f32(SEDS_DT_BAROMETER_DATA,3,value);
 }
}
int main(void){
 const unsigned period=fc_telemetry_period_ms();
 /*50ms source polls, with actual publisher calls jittering by1ms. The board
  * cap still applies if the producer requests an incorrectly fast zero rate. */
 for(unsigned t=0;t<=2000;t+=50)opportunity(0,t);
 assert(calls==2000/period);
 /* Recovery's slower configured mode remains slower. */
 calls=0;local_time[0]=0;
 for(unsigned t=0;t<=6*period;t++)opportunity(3*period,t);
 assert(calls==2);
 calls=0;local_time[0]=0;
 opportunity((fu32)UINT_FAST32_MAX,UINT32_MAX);opportunity((fu32)UINT_FAST32_MAX,0);
 assert(calls==0);
 /* A long stall emits one latest report, not one per missed deadline. */
 local_time[0]=0;opportunity(period,10*period+1);unsigned first=calls;
 opportunity(period,10*period+1);assert(calls==first&&first==1);
 calls=0;local_time[0]=UINT32_MAX-10U-period;
 opportunity(period,UINT32_MAX-10U);assert(calls==1);
 opportunity(period,(uint32_t)(UINT32_MAX-10U+period-1U));assert(calls==1);
 opportunity(period,(uint32_t)(UINT32_MAX-10U+period));assert(calls==2);
 assert(locks==0);
}
'''
    with tempfile.TemporaryDirectory() as d:
        d=Path(d);(d/'sedsnet.h').write_text('#include <stdint.h>\ntypedef uint32_t SedsDataType;\ntypedef uint32_t SedsDataEndpoint;\n');exe=d/'composed'
        subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',f'-DFC_TELEMETRY_RATE_HZ={rate}','-I',str(d),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/telemetry_rate.c'),'-x','c','-','-o',str(exe)],input=code,text=True,check=True)
        return subprocess.run([str(exe)],capture_output=True,text=True)

class GroundCadence(unittest.TestCase):
    def test_actual_composed_publishers_preserve_modes_wrap_and_nominal_rates(self):
        for rate in [4,5,20]:
            with self.subTest(rate=rate):
                result=run(rate);self.assertEqual(result.returncode,0,result.stderr)
    def test_duplicate_generic_gate_negative_control_drops_due_reports(self):
        result=run(5,duplicate=True);self.assertNotEqual(result.returncode,0);self.assertIn('calls==2000/period',result.stderr)

if __name__=='__main__':unittest.main()

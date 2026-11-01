"""Actual calibration and acquisition functions: late math cannot commit old Armed.

FC_POSTINIT_SOURCE_DIR optionally selects a preserved baseline for the same
safety assertions; no firmware implementation is recreated by the fixture.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

def function(source, signature):
    start = source.index(signature)
    body = source.index('{', start)
    depth = 1
    end = body + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

STUB = r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#define conditional
#define EKF_STATE 6
#define SEDS_DT_EULER_ANGLES 1
#define SEDS_DT_ASCENT_STATE 2
#define Auxiliary 0
#define POSTINIT_DURATION 0
#define IMU 0
#define Baro 1
#define BOARD_WATCHDOG_ACQUISITION 1
#define BOARD_WATCHDOG_SAFETY 2
#define Acq 0
#define Rel 0
#define Rlx 0
#define store(p,v,o) (*(p)=(v))
#define fetch_add(p,v,o) ((*(p))+=(v),(*(p))-(v))
#define Postinit_Requested 1U
#define option(x) (x)
#define load(p,o) (*(p))
#define fetch_and(p,v,o) (*(p)&=(v))
#define MrAnalog(condition) while (!(condition))
#define id "FC "
typedef uint32_t fu32;
typedef struct {float x,y,z;} f_xyz;
typedef struct {float phi,theta,psi;} eul;
typedef struct {float q0,rho1,rho2,rho3;} quat;
enum {Startup, Postinit, Armed};
struct {unsigned flight;} sm;
struct {f_xyz accl,baro;} meas;
quat qv;
uint32_t g_conf,epoch=1,irq;
unsigned elapsed,logs,messages;
bool cancel_during_math;
uint32_t current(void){return sm.flight;}
uint32_t fc_recovery_epoch(void){return epoch;}
bool fc_recovery_postinit_allowed(uint32_t value){return value==epoch;}
uint32_t __get_PRIMASK(void){return irq;}
void __disable_irq(void){irq=1;}
void __set_PRIMASK(uint32_t value){irq=value;}
float fcos(float x){
 if(cancel_during_math){cancel_during_math=false;++epoch;sm.flight=Startup;g_conf=0;}
 return cosf(x);
}
float fsin(float x){return sinf(x);}
void fatan2(float y,float x,float *out){*out=atan2f(y,x);}
void vnorm2(float y,float z,float *out){*out=sqrtf(y*y+z*z);}
float inorm4(float a,float b,float c,float d){return 1.0f/sqrtf(a*a+b*b+c*c+d*d);}
float deg(float x){return x*57.2957795f;}
void log_f32(unsigned type,unsigned n,const void *v){(void)type;(void)n;(void)v;assert(!irq);++logs;}
void timer_update(unsigned t){(void)t;elapsed=0;}
unsigned timer_fetch(unsigned t){(void)t;return elapsed++;}
void board_watchdog_progress(unsigned m){(void)m;}
bool try_fetch_accl(f_xyz *v){*v=(f_xyz){0,0,1};return true;}
bool try_fetch_baro(f_xyz *v){(void)v;return false;}
bool validate_accl(f_xyz *v,fu32 c){(void)v;(void)c;return true;}
void validate_baro(f_xyz *v,fu32 c){(void)v;(void)c;}
void maybe_log_measm(unsigned t,const void *v){(void)t;(void)v;}
void watch_for_gps_packets(fu32 c,fu32 *a,fu32 *n){(void)c;(void)a;(void)n;}
void tx_thread_relinquish(void){}
void message(const char *text,bool important){(void)text;(void)important;++messages;}
'''
MAIN = r'''
int main(int argc,char **argv){
 assert(argc==2);sm.flight=Postinit;g_conf=Postinit_Requested;
 const quat original={0.25f,0.5f,0.75f,1};qv=original;
 cancel_during_math=!strcmp(argv[1],"cancel");
 post_initialization();
 if(!strcmp(argv[1],"cancel")){
  assert(epoch==2&&sm.flight==Startup&&g_conf==0);
  assert(memcmp(&qv,&original,sizeof qv)==0&&logs==0&&messages==0&&!irq);
 }else{
  assert(epoch==1&&sm.flight==Armed&&!(g_conf&Postinit_Requested));
  assert(fabsf(qv.q0-1.0f)<0.0001f&&qv.rho1==0&&qv.rho2==0&&qv.rho3==0);
  assert(logs==2&&messages==1&&!irq);
 }
 return 0;
}
'''
class PostinitEpochTests(unittest.TestCase):
    def test_actual_calibration_cannot_commit_after_reset(self):
        source_root=Path(os.environ.get('FC_POSTINIT_SOURCE_DIR', str(ROOT)))
        kalman=(source_root/'Core/Src/kalman.c').read_text()
        distribution=(source_root/'Core/Src/distribution.c').read_text()
        signature='bool accel_to_quaternion(' if 'bool accel_to_quaternion(' in kalman else 'void accel_to_quaternion('
        actual='\n'.join([function(kalman,'static inline void euler_to_quat('),
                           function(kalman,signature),
                           function(distribution,'static inline void post_initialization(')])
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);src=p/'postinit.c';src.write_text(STUB+actual+MAIN);exe=p/'postinit'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-lm','-o',str(exe)],check=True)
            for case in ['normal','cancel']:
                with self.subTest(case=case):subprocess.run([str(exe),case],check=True)

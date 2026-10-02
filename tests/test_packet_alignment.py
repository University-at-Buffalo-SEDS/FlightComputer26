"""Network byte payloads must never be dereferenced as aligned float structs."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class PacketAlignmentTests(unittest.TestCase):
    def test_gps_at_every_byte_alignment(self):
        source=(ROOT/'Core/Src/distribution.c').read_text()
        function=source[source.index('static inline SedsResult\nprocess_gps_packet'):source.index('static inline void to_relative_coords')]
        code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define GPS_AVAILABLE 1
#define Rlx 0
#define option(x) 1U
#define fc_mask(x) x
#define TX_NO_WAIT 0
#define sweetbench_catch(x) ((void)0)
#define sweetbench_start(x,y) ((void)0)
#define timer_update(x) ((void)0)
#define fc_lock(x) ((void)0)
#define fc_concede(x) ((void)0)
typedef uint32_t fu32;
typedef int SedsResult;
typedef struct {float x,y,z;} f_xyz;
enum {SEDS_OK,SEDS_ERR,GPS_Malformed,GPS_Data_Code};
static unsigned g_conf,seds_syscall,reports;
static struct {int rflock; f_xyz coords_buf; bool updated;} rfboard;
static unsigned fetch_or(unsigned *p,unsigned v,int m){(void)m;unsigned old=*p;*p|=v;return old;}
static void tx_queue_send(unsigned *q,unsigned *v,int n){(void)q;(void)n;assert(*v==GPS_Malformed);reports++;}
static unsigned validate_coords(const f_xyz *p,size_t n,unsigned cfg){(void)cfg;assert(n==12);assert(p->x==42.5f && p->y==-78.25f && p->z==123.0f);return GPS_Data_Code;}
''' + function + r'''
int main(void){
    _Alignas(8) uint8_t bytes[32]; f_xyz expected={42.5f,-78.25f,123.0f};
    for(unsigned offset=0;offset<8;offset++){
        memcpy(bytes+offset,&expected,sizeof(expected));rfboard.updated=false;
        assert(process_gps_packet(bytes+offset,sizeof(expected))==SEDS_OK);
        assert(rfboard.updated && memcmp(&rfboard.coords_buf,&expected,sizeof(expected))==0);
    }
    rfboard.updated=false;
    assert(process_gps_packet(NULL,12)==SEDS_ERR);
    assert(process_gps_packet(bytes,11)==SEDS_ERR);
    assert(process_gps_packet(bytes,13)==SEDS_ERR);
    assert(!rfboard.updated && reports==3);
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'alignment'
            p=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
            self.assertEqual(p.returncode,0,p.stderr)
            subprocess.run([str(exe)],check=True)
    def test_bias_payload_is_copied_before_typed_access(self):
        source=(ROOT/'Core/Src/distribution.c').read_text()
        self.assertNotIn('*(ekf_bias *)data',source)
        self.assertIn('memcpy(&biases, data, sizeof(biases));',source)
    def test_m33_fault_frame_does_not_skip_core_registers(self):
        source=(ROOT/'Core/Src/stm32h5xx_it.c').read_text()
        handler=source.split('hardfault_capture_and_halt',1)[1].split('void reg_dump',1)[0]
        self.assertNotIn('core_frame +=',handler)
        self.assertIn('g_hardfault_stacked_pc = core_frame[6];',handler)

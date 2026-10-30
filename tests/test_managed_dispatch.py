"""A delivered managed value must not be rejected by FC's generic endpoint."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class ManagedDispatchTests(unittest.TestCase):
    def test_managed_receipts_and_normal_commands(self):
        source = (ROOT / 'Core/Src/distribution.c').read_text()
        start = source.index('static SedsResult on_fc_packet_impl(')
        end = source.index('\n#endif', start)
        code = r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "ground_heartbeat_owner.h"
typedef int SedsResult;
enum {SEDS_OK=0, SEDS_HANDLER_ERROR=-9, SEDS_DT_HEARTBEAT=1,
 SEDS_DT_FLIGHT_STATE, SEDS_DT_GPS_DATA, SEDS_DT_FLIGHT_COMMAND,
 SEDS_DT_ASCENT_BIASES, SEDS_DT_AV_BAY_UNDERGLOW, SEDS_DT_FLIGHT_BUZZER};
typedef struct {unsigned ty;const char *sender;size_t sender_len;
 const uint8_t *payload;size_t payload_len;} SedsPacketView;
static unsigned commands, heartbeats;
static int pulse_ground(void){heartbeats++;return 0;}
static int implicit_postinit(const uint8_t*p,size_t n){(void)p;(void)n;return 0;}
static int process_gps_packet(const uint8_t*p,size_t n){(void)p;(void)n;return 0;}
static int dispatch_flight_cmd(const uint8_t*p,size_t n){assert(p && n==1);commands++;return 42;}
static int update_ascent_biases(const uint8_t*p,size_t n){(void)p;(void)n;return 0;}
""" + source[start:end] + r"""
int main(void){
 uint8_t value=1;SedsPacketView p={SEDS_DT_AV_BAY_UNDERGLOW,"GS",2,&value,1};
 assert(on_fc_packet(&p,NULL)==SEDS_OK);
 p.ty=SEDS_DT_FLIGHT_BUZZER;assert(on_fc_packet(&p,NULL)==SEDS_OK);
 assert(commands==0);
 p.payload_len=2;assert(on_fc_packet(&p,NULL)==SEDS_HANDLER_ERROR);
 p.payload_len=1;p.payload=NULL;assert(on_fc_packet(&p,NULL)==SEDS_HANDLER_ERROR);
 p.payload=&value;p.ty=SEDS_DT_FLIGHT_COMMAND;
 assert(on_fc_packet(&p,NULL)==42 && commands==1);
 p.ty=SEDS_DT_HEARTBEAT;assert(on_fc_packet(&p,NULL)==SEDS_OK && heartbeats==1);
 p.sender="DAQ";p.sender_len=3;assert(on_fc_packet(&p,NULL)==SEDS_OK && heartbeats==1);
 p.sender="@addr:3561204631";p.sender_len=16;assert(on_fc_packet(&p,NULL)==SEDS_OK && heartbeats==2);
 p.sender="GS";p.sender_len=2;
 p.ty=999;assert(on_fc_packet(&p,NULL)==SEDS_HANDLER_ERROR);
 assert(g_fc_endpoint_error_type==999 && g_fc_endpoint_error_result==SEDS_HANDLER_ERROR);
 p.sender_len=0;assert(on_fc_packet(&p,NULL)==SEDS_HANDLER_ERROR);
 assert(on_fc_packet(NULL,NULL)==SEDS_HANDLER_ERROR);
 assert(g_fc_endpoint_error_type==0 && g_fc_endpoint_error_result==SEDS_HANDLER_ERROR);
}
"""
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'managed-dispatch'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter',
                '-I',str(ROOT/'Core/Inc'),'-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True)

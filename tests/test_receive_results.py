from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
PREFIX = '#include <stdint.h>\n#include <stddef.h>\n#include <assert.h>\n#define TELEMETRY_ENABLED 1\n#define SEDS_OK 0\ntypedef int SedsResult;\nstruct {void *r;} g_router={(void*)1};\nint g_can_side_id=0, result_code, locked;\nunsigned g_telemetry_rx_stage,g_telemetry_discovery_seen,g_telemetry_rx_ok,g_telemetry_rx_errors,g_telemetry_rx_error_length;\nint g_telemetry_rx_last_result;\nuint8_t g_telemetry_rx_error_prefix[16];\nint init_telemetry_router(void){return 0;}\nvoid telemetry_lock(void){assert(!locked);locked=1;}\nvoid telemetry_unlock(void){assert(locked);locked=0;}\nint seds_router_receive_packed_from_side(void*r,unsigned i,const uint8_t*b,size_t n){assert(r&&i==0&&b&&n&&locked);return result_code;}\nint seds_router_receive_packed(void*r,const uint8_t*b,size_t n){assert(r&&b&&n&&locked);return result_code;}\n'
MAIN = 'int main(void){uint8_t b[]={1,2,3};result_code=-13;rx_asynchronous(b,3);assert(!locked&&g_telemetry_rx_errors==1&&!g_telemetry_discovery_seen&&g_telemetry_rx_last_result==-13&&g_telemetry_rx_error_length==3&&g_telemetry_rx_error_prefix[2]==3&&g_telemetry_rx_error_prefix[3]==0); result_code=0;rx_asynchronous(b,3);assert(g_telemetry_rx_ok==1&&g_telemetry_discovery_seen==1&&!locked);g_can_side_id=-1;result_code=-18;rx_asynchronous(b,3);assert(g_telemetry_rx_errors==2&&g_telemetry_rx_last_result==-18&&!locked);rx_asynchronous(0,3);rx_asynchronous(b,0);assert(g_telemetry_rx_errors==2);}\n'
class ReceiveResultTests(unittest.TestCase):
    def test_rejected_can_packet_is_not_network_health(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        callback=source[source.index('void rx_asynchronous('):source.index('static UNUSED_FUNCTION void rx_synchronous')]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)
            (path/'test.c').write_text(PREFIX+callback+MAIN)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(path/'test.c'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

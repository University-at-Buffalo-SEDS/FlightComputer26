"""Production numeric publisher preserves little-endian payloads without typed conversion."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TelemetryWireBytesTests(unittest.TestCase):
    def test_payload_bytes_and_overflow_checks(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        functions = source[source.index("SedsResult log_telemetry_synchronous"):
                           source.index("SedsResult log_telemetry_string_asynchronous")]
        code = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#define TELEMETRY_ENABLED 1
typedef int SedsResult;
typedef unsigned SedsDataType;
enum { SEDS_OK, SEDS_BAD_ARG, SEDS_ERR };
static struct { void *r; } g_router={(void*)1};
static unsigned locks, calls;
static uint8_t bytes[32];
static size_t size;
static void telemetry_lock(void) { ++locks; }
static void telemetry_unlock(void) { --locks; }
static int init_telemetry_router(void) { return 0; }
static unsigned HAL_GetTick(void) { return 0; }
static int fc_telemetry_rate_allow(unsigned ty,unsigned now) { (void)ty; (void)now; return 1; }
static int seds_router_log_bytes(void *r,unsigned ty,const void *data,size_t n) {
  assert(r && ty==100 && n<=sizeof(bytes) && locks==1);
  memcpy(bytes,data,n); size=n; ++calls; return 0;
}
''' + functions + r'''
int main(void) {
  float f[3]={1.0f,-2.0f,0.5f};
  const uint8_t expected[]={0,0,128,63,0,0,0,192,0,0,0,63};
  assert(log_telemetry_asynchronous(100,f,3,4)==SEDS_OK);
  assert(size==12 && memcmp(bytes,expected,12)==0 && locks==0);
  double d=1.25;
  const uint8_t expected_double[]={0,0,0,0,0,0,244,63};
  assert(log_telemetry_synchronous(100,&d,1,8)==SEDS_OK);
  assert(size==8 && memcmp(bytes,expected_double,8)==0);
  assert(log_telemetry_synchronous(100,f,SIZE_MAX,4)==SEDS_BAD_ARG);
  assert(log_telemetry_asynchronous(100,f,1,3)==SEDS_BAD_ARG);
  assert(log_telemetry_synchronous(100,NULL,1,4)==SEDS_BAD_ARG);
  assert(calls==2 && locks==0);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "wire-bytes"
            result = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                     "-x", "c", "-", "-o", str(exe)],
                                    input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()

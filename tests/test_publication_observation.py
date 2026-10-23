import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]

class PublicationObservationTests(unittest.TestCase):
    def test_both_paths_share_exact_nonallocating_observations(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        start=source.index('static __attribute__((noinline)) void record_sensor_publication(')
        end=source.index('\n#endif',start)
        helper=source[start:end]
        self.assertEqual(source.count('record_sensor_publication(data_type, result);'),2)
        with tempfile.TemporaryDirectory() as temp:
            tmp=pathlib.Path(temp)
            (tmp/'sedsnet.h').write_text('#include <stdint.h>\ntypedef uint32_t SedsDataType;\ntypedef uint32_t SedsDataEndpoint;\n')
            code='''#include "sedsnet_config.h"
#include <assert.h>
typedef int SedsResult;
#define SEDS_OK 0
volatile uint32_t g_sim_baro_publish_attempts, g_sim_baro_publish_ok;
volatile uint32_t g_sim_imu_publish_attempts, g_sim_imu_publish_ok;
volatile int32_t g_sim_baro_publish_result;
'''+helper+'''
int main(void) {
  for(unsigned i=0;i<1000;++i) {
    record_sensor_publication(SEDS_DT_BAROMETER_DATA, i%2?0:-14);
    record_sensor_publication(SEDS_DT_IMU_DATA, i%2?0:-1);
    record_sensor_publication(SEDS_DT_GPS_DATA, 0);
  }
  assert(g_sim_baro_publish_attempts==1000 && g_sim_baro_publish_ok==500);
  assert(g_sim_imu_publish_attempts==1000 && g_sim_imu_publish_ok==500);
  assert(g_sim_baro_publish_result==0);
  record_sensor_publication(SEDS_DT_BAROMETER_DATA, -14);
  assert(g_sim_baro_publish_attempts==1001 && g_sim_baro_publish_result==-14);
  assert(g_sim_baro_publish_ok==500 && g_sim_imu_publish_attempts==1000);
}
'''
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(tmp),'-I'+str(ROOT/'Core/Inc'),'-x','c','-','-o',str(tmp/'observe')],input=code,text=True,check=True,capture_output=True)
            subprocess.run([str(tmp/'observe')],check=True)

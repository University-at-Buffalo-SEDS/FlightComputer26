"""Buzzer durability retries must retain successful source timestamp ordering."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_underglow_persistence import STUB

ROOT = Path(__file__).resolve().parents[1]

class BuzzerPersistenceRetryTests(unittest.TestCase):
    def test_identical_retry_persists_and_stale_updates_stay_rejected(self):
        stub = STUB.replace('size_t payload_len; }', 'size_t payload_len; uint64_t timestamp; }')
        stub = stub.replace('#define SEDS_DT_AV_BAY_UNDERGLOW 7',
                            '#define SEDS_DT_AV_BAY_UNDERGLOW 7\n#define SEDS_DT_FLIGHT_BUZZER 8\n#define Buzzer_GPIO_Port 0\n#define Buzzer_Pin 0')
        stub = stub.replace('assert(n==1)', 'assert(n==9)')
        source = (ROOT / 'Core/Src/flight_buzzer.c').read_text()
        source = '\n'.join(line for line in source.splitlines()
                           if not line.startswith('#include "'))
        test = r'''
static SedsResult update(uint8_t value,uint64_t at){SedsPacketView p={SEDS_DT_FLIGHT_BUZZER,&value,1,at};return callback(&p,NULL);}
int main(void){
 (void)irq_mask;
 SedsRouter router=0;assert(flight_buzzer_init(&router)==SEDS_OK);assert(callback);
 assert(update(0,100)==SEDS_OK);assert(writes==1&&stored==0&&g_last_source_timestamp_ms==100);
 fail_write=1;
 assert(update(1,200)==SEDS_HANDLER_ERROR);assert(writes==2&&stored==0&&g_last_source_timestamp_ms==100);
 assert(update(1,200)==SEDS_HANDLER_ERROR);assert(writes==3&&stored==0&&g_last_source_timestamp_ms==100);
 fail_write=0;
 assert(update(1,200)==SEDS_OK);assert(writes==4&&stored==1&&g_last_source_timestamp_ms==200);
 assert(update(1,200)==SEDS_OK);assert(writes==4);
 assert(update(0,150)==SEDS_OK);assert(writes==4&&g_flight_buzzer_enabled==1&&g_flight_buzzer_stale_updates==1);
 assert(update(1,300)==SEDS_OK);assert(writes==5&&g_last_source_timestamp_ms==300);
 assert(pin==0); /* setting controls next boot indication, not current output */
}
'''
        with tempfile.TemporaryDirectory(prefix='buzzer-persistence-') as temp:
            executable = str(Path(temp) / 'test')
            subprocess.run(['cc', '-DTELEMETRY_ENABLED=1', '-std=c11', '-Wall',
                            '-Wextra', '-Werror', '-x', 'c', '-', '-o', executable],
                           input=stub + source + test, text=True, check=True)
            subprocess.run([executable], check=True)

if __name__ == '__main__':
    unittest.main()

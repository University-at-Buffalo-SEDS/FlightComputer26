"""BOARD_HEALTH tells the station why the FC refuses flight commands.

distribution.c acknowledges and drops a FLIGHT_COMMAND when
board_recovery_controller_update_allowed() fails its store/floor check, so the
station only saw that nothing happened. board_recovery_controller_gate()
reports the reason in BOARD_HEALTH word 6 bits 24..27 (bits 0..15 stay the
watchdog mask; RF/PB service faults use 16..23):
  bit 24 store not ready, 25 commissioning hold, 26 store ownership fault,
  27 recovery floor failed.
The gate is zero exactly when that check would admit a command, and reading
it never touches storage (it runs on the telemetry thread every 5 s).
"""
from pathlib import Path
import re, subprocess, tempfile, unittest
from test_controller_station_authority import function

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef struct SedsPacketView SedsPacketView;
static bool ready, hold, owner_fault, floor_ok, floor_loads;
static bool persistent_store_controls_ready(void){return ready&&!hold&&!owner_fault;}
static bool persistent_store_commission_pending(void){return hold;}
static bool persistent_store_ownership_faulted(void){return owner_fault;}
static uint8_t board_recovery_floor_cached_health(void){return floor_ok;}
static uint8_t board_recovery_floor_is_healthy(void){floor_loads=true;return floor_ok;}
static uint8_t board_recovery_floor_active(void){return 0;}
static void *recovery_router;
static uint8_t board_recovery_manual_stop_source_allowed(void*r,const SedsPacketView*p){(void)r;(void)p;return 1;}
static uint8_t board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return 0;}
@GATE@
@ALLOWED@
int main(void){
 (void)recovery_router;
 for(unsigned m=0;m<16;m++){
  ready=m&1;hold=m&2;owner_fault=m&4;floor_ok=m&8;floor_loads=false;
  uint32_t gate=board_recovery_controller_gate();
  assert(!floor_loads); /* never the lazy floor loader */
  assert(gate<=15U);
  assert((gate==0U)==(board_recovery_controller_update_allowed(0)!=0));
  bool store=persistent_store_controls_ready();
  assert(((gate&1U)!=0)==(!store&&!hold&&!owner_fault));
  assert(((gate&2U)!=0)==(!store&&hold));
  assert(((gate&4U)!=0)==(!store&&!hold&&owner_fault));
  assert(((gate&8U)!=0)==!floor_ok);
 }
 /* Bench fault: store not ready and floor latched failed. */
 ready=false;hold=false;owner_fault=false;floor_ok=false;assert(board_recovery_controller_gate()==(1U|8U));
 puts("controller gate matches the controller admission check PASS");return 0;
}
'''


class CommandGateHealthTests(unittest.TestCase):
    def test_gate_reports_each_refusal_reason_without_storage_io(self):
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        code = HARNESS.replace('@GATE@', function(transport, 'board_recovery_controller_gate'))
        code = code.replace('@ALLOWED@', function(transport, 'board_recovery_controller_update_allowed'))
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'gate.c'; source.write_text(code); exe = Path(tmp) / 'gate'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            str(source), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_board_health_carries_the_gate_in_word_six_high_bits(self):
        health = (ROOT / 'Core/Src/board_health.c').read_text()
        self.assertRegex(health, r'words\[6\]\s*\|=\s*board_recovery_controller_gate\(\)\s*<<\s*24U')
        self.assertIn('uint32_t board_recovery_controller_gate(void);',
                      (ROOT / 'Core/Inc/board_recovery_transport.h').read_text())
        self.assertIn('bool persistent_store_ownership_faulted(void);',
                      (ROOT / 'Core/Inc/persistent_store.h').read_text())


if __name__ == '__main__':
    unittest.main()

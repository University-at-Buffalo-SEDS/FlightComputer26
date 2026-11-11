"""Network FLIGHT_STATE contract (state-split, 2026-11-11).

Only set (a) may go on the wire: 0 Startup, 1 Idle, 2 Fill, 6 Armed,
7 Launch, 13 Landed, 15 Aborted, 16 InFlight. The FC publishes InFlight for
Ascent..Reefing and Landed for Landed and Recovery; its own BOARD_STATE_STATUS
keeps the detailed phase. A received retired value maps to the nearest network
value (3-5 Fill, 8-12 InFlight, 14 Landed) and is counted; anything above 16
is refused. InFlight (16) is accepted.
"""
from pathlib import Path
import re, subprocess, tempfile, unittest
from test_controller_station_authority import function

ROOT = Path(__file__).resolve().parents[1]
SET_A = {0, 1, 2, 6, 7, 13, 15, 16}


def enum_body(text, start, end):
    return text[text.index(start):text.index(end) + len(end)]


class FlightStateContractTests(unittest.TestCase):
    def harness(self, body):
        structs = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        local = enum_body(structs, 'typedef enum flight_state : fu8 {', '} state;').replace(' : fu8', '')
        ground = enum_body(structs, 'typedef enum global_state : uint8_t {', '} gnd_state;').replace(' : uint8_t', '')
        api = (ROOT / 'Core/Inc/fcapi.h').read_text()
        code = ('#include <assert.h>\n#include <stdint.h>\n#include <stdio.h>\n#include <stdbool.h>\n'
                + local + '\n' + ground + '\n'
                + '__attribute__((unused)) static ' + function(api, 'global_state_of').replace('static inline ', '') + '\n'
                + (ROOT / 'Core/Inc/flight_state_contract.h').read_text() + '\n'
                + body)
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'c.c'; src.write_text(code); exe = Path(tmp) / 'c'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            str(src), '-o', str(exe)], check=True)
            return subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout

    def test_published_values_are_set_a_only(self):
        out = self.harness(r'''
int main(void){
 for(int s=0;s<Flight_States;s++)printf("%d %d\n",s,(int)global_state_of((state)s));
 printf("%d %d\n",99,(int)global_state_of((state)99));
 return 0;}
''')
        table = dict(tuple(map(int, line.split())) for line in out.split('\n') if line)
        for local, value in table.items():
            self.assertIn(value, SET_A, f'local {local} publishes {value}')
        self.assertEqual([table[s] for s in range(4, 9)], [16] * 5, 'Ascent..Reefing -> InFlight')
        self.assertEqual([table[9], table[10]], [13, 13], 'Landed and Recovery -> Landed')
        self.assertEqual([table[0], table[1], table[2], table[3]], [0, 6, 6, 7])

    def test_receipt_maps_retired_values_and_refuses_unknown(self):
        out = self.harness(r'''
int main(void){for(int v=0;v<=20;v++)printf("%d %d\n",v,network_flight_state((uint8_t)v));return 0;}
''')
        table = dict(tuple(map(int, line.split())) for line in out.split('\n') if line)
        expected = {0: 0, 1: 1, 2: 2, 3: 2, 4: 2, 5: 2, 6: 6, 7: 7, 8: 16, 9: 16, 10: 16, 11: 16, 12: 16,
                    13: 13, 14: 13, 15: 15, 16: 16}
        for v in range(21):
            self.assertEqual(table[v], expected.get(v, -1), f'value {v}')

    def test_receivers_use_the_contract_mapping(self):
        dist = function((ROOT / 'Core/Src/distribution.c').read_text(), 'implicit_postinit')
        self.assertNotIn('> G_Aborted', dist, 'implicit_postinit must accept InFlight (16)')
        self.assertIn('network_flight_state(', dist)
        cache = (ROOT / 'Core/Src/flight_state_cache.c').read_text()
        self.assertNotIn('FLIGHT_STATE_MAX_VALUE', cache)
        self.assertGreaterEqual(cache.count('network_flight_state('), 2, 'restore and network update')
        # The FC's own detailed state report stays detailed (BOARD_STATE_STATUS byte 2).
        query = (ROOT / 'Core/Src/board_recovery_query.c').read_text()
        self.assertIn('case Ascent: return G_Ascent;', query)


if __name__ == '__main__':
    unittest.main()

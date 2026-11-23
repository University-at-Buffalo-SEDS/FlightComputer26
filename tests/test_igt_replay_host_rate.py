"""The host IGT replay samples the baro at the board's rate (data-pipeline 9.6).

The flight state machine counts samples, so a faster host schedule would make
every confidence window shorter in time than on the board."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
ODR_PERIOD_MS = {'Baro_ODR_200': 5, 'Baro_ODR_100': 10, 'Baro_ODR_50': 20, 'Baro_ODR_25': 40, 'Baro_ODR_12P5': 80}


class HostReplayRate(unittest.TestCase):
    def test_baro_period_matches_the_flight_configuration(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        conf = recovery[recovery.index('static struct baro_config baro_conf'):]
        odr = re.search(r'\.odr = (Baro_ODR_\w+)', conf).group(1)
        tool = (ROOT / 'tools/igt_replay_host.py').read_text()
        period = int(re.search(r'#define BARO_PERIOD_MS (\d+)U', tool).group(1))
        self.assertEqual(period, ODR_PERIOD_MS[odr])
        self.assertNotRegex(tool, r'host_now % 10U')
        self.assertIn('host_now % BARO_PERIOD_MS == 0U', tool)
        self.assertIn('host_now % BARO_PERIOD_MS == 1U', tool)


if __name__ == '__main__':
    unittest.main()

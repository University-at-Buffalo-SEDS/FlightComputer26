"""Timing diagnostics stay in the FC's local log and off the shared radio.

"Baro interval" fires every 61 barometer samples, about 1.5 times a second.
Sent as MESSAGE_DATA it took about 5% of the RF board's radio frames.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class DiagnosticMetricsStayLocalTests(unittest.TestCase):
    def test_baro_interval_is_logged_locally_only(self):
        source = (ROOT / 'Core/Src/distribution.c').read_text()
        calls = re.findall(r'(\w+)\(id "Baro interval"', source)
        self.assertEqual(calls, ['log_metric_local'])

    def test_local_metric_never_reaches_the_ground_path(self):
        header = (ROOT / 'Core/Inc/fcapi.h').read_text()
        start = header.index('log_metric_local(const char')
        body = header[start:header.index('\n}\n', start)]
        for remote in ('log_msg', 'log_critical', 'message(', 'MessageRemote'):
            self.assertNotIn(remote, body)
        self.assertIn('sd_append_string', body)


if __name__ == '__main__':
    unittest.main()

"""A delivered reliable packet the FC refuses is still acknowledged (real SDK).

A handler error withholds the SDK end-to-end receipt and drops dedupe, so the
station retransmits forever and every copy re-enters the handlers. The FC's
actual ABORT, FLIGHT_CONTROLLER and managed FLIGHT_STATE handlers run inside a
real router; owners are intent counters, not physical output evidence.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r'(?m)^[^;{}\n]*\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', source)
    assert match, name
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


class FCReliableRefusalAckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='fc-refusal-ack-')
        path = Path(cls.tmp.name)
        template = (ROOT / 'tests/native/fc_reliable_refusal_ack.in.c').read_text()
        sources = {'Manual_Abort_handler': 'Core/Src/telemetry.c',
                   'on_fc_packet_impl': 'Core/Src/distribution.c',
                   'implicit_postinit': 'Core/Src/distribution.c',
                   'clear_managed_cache': 'Core/Src/flight_state_cache.c',
                   'persist_update': 'Core/Src/flight_state_cache.c'}
        for name, file in sources.items():
            marker = '/* SOURCE_FUNCTION:' + name + ' */'
            assert template.count(marker) == 1, name
            template = template.replace(marker, function((ROOT / file).read_text(), name))
        source = path / 'refusal.c'
        source.write_text(template)
        cls.exe = path / 'refusal'
        command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                   '-fsanitize=address,undefined', '-I' + str(ROOT / 'Core/Inc'),
                   '-I' + str(Path(cls.sdk['header']).parent), str(source),
                   cls.sdk['archive'], '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        subprocess.run(command + ['-o', str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_mode(self, mode):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), mode],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS ' + mode, result.stdout)

    def test_station_abort_is_acknowledged_and_applied_once(self):
        self.run_mode('abort')

    def test_refused_abort_is_acknowledged_only_when_already_stopped(self):
        self.run_mode('abort_refused_stopped')
        self.run_mode('abort_refused_running')

    def test_refused_flight_state_is_acknowledged_and_not_applied(self):
        self.run_mode('fstate_refused')

    def test_aborted_flight_state_after_the_stop_latches_is_acknowledged(self):
        self.run_mode('fstate_aborted_after_stop')

    def test_armed_flight_state_waits_for_the_stop_to_release(self):
        self.run_mode('fstate_armed_held_after_stop')

    def test_a_flight_command_copy_is_dispatched_once(self):
        # GS heard-gap copies of Deploy/Expand: one effect per command, also
        # after the FC's 16-entry recent-ID ring rolled over.
        self.run_mode('flight_cmd_copy_dedupe')

    def test_refused_controller_command_is_acknowledged_and_not_applied(self):
        self.run_mode('controller_refused')


if __name__ == '__main__':
    unittest.main()

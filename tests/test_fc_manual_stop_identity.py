"""Actual FC handler/CAPI: foreign wire identities admit only explicit station Stop.

Producer and receiver are separate processes to avoid the std wire name cache
hiding the compact identities embedded consumers receive. The real handler and
source/time functions are extracted; the OFF owner is an intent-count stub, not
physical evaluator/pyro completion. The floor boundary denies ordinary traffic.
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
    start = match.start()
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class FCManualStopIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='fc-manual-stop-identity-')
        cls.path = Path(cls.tmp.name)
        template = (ROOT / 'tests/native/fc_manual_stop_identity.in.c').read_text()
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        for name in ('authorized_owner', 'board_recovery_manual_stop_source_allowed',
                     'board_recovery_command_is_stale', 'Manual_Abort_handler'):
            marker = '/* SOURCE_FUNCTION:' + name + ' */'
            assert template.count(marker) == 1, name
            template = template.replace(marker, function(telemetry if name == 'Manual_Abort_handler' else transport, name))
        source = cls.path / 'identity.c'
        source.write_text(template)
        cls.exe = cls.path / 'identity'
        command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=address,undefined', '-I' + str(ROOT / 'Core/Inc'),
                   '-I' + str(Path(cls.sdk['header']).parent), str(source),
                   cls.sdk['archive'], '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        command += ['-o', str(cls.exe)]
        subprocess.run(command, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, mode, sender, form, marker, wire, discovery, expected):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), mode,
                                 sender, form, marker, str(wire), str(discovery), str(expected)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        if mode != 'produce':
            self.assertIn('PASS expected=' + str(expected), result.stdout)
            self.assertIn('floor_calls=0', result.stdout)
        return result.stdout

    def produce(self, sender, form='compact', marker='manual'):
        stem = sender + '-' + form + '-' + marker
        wire, discovery = self.path / (stem + '.bin'), self.path / (stem + '.discovery')
        self.run_case('produce', sender, form, marker, wire, discovery, 0)
        return wire, discovery

    def test_discovered_foreign_compact_primary_and_rrls(self):
        for sender in ('GS', 'RRLS'):
            with self.subTest(sender=sender):
                wire, discovery = self.produce(sender)
                log = self.run_case('receive', sender, 'compact', 'manual', wire, discovery, 1)
                self.assertIn('callback sender=@addr:', log)
                self.assertIn('callbacks=1 stop_calls=1', log)

    def test_discovered_foreign_assigned_primary_and_rrls(self):
        for sender in ('GS', 'RRLS'):
            with self.subTest(sender=sender):
                wire, discovery = self.produce(sender, 'assigned')
                self.run_case('receive', sender, 'assigned', 'manual', wire, discovery, 1)

    def test_unknown_and_wrong_foreign_sources_do_not_stop(self):
        for sender in ('GS', 'RRLS'):
            for form in ('compact', 'assigned'):
                with self.subTest(sender=sender, form=form):
                    wire, discovery = self.produce(sender, form)
                    self.run_case('unknown', sender, form, 'manual', wire, discovery, 0)
        for form in ('compact', 'assigned'):
            wire, discovery = self.produce('VB', form)
            self.run_case('receive', 'VB', form, 'manual', wire, discovery, 0)

    def test_exact_marker_only_and_manual_ignores_ordinary_clock_floor(self):
        for sender in ('GS', 'RRLS'):
            for marker in ('auto', 'legacy', 'long', 'short'):
                with self.subTest(sender=sender, marker=marker):
                    wire, discovery = self.produce(sender, marker=marker)
                    self.run_case('receive', sender, 'compact', marker, wire, discovery, 0)

    def test_named_and_malformed_actual_handler_views(self):
        records = [self.produce(sender) for sender in ('GS', 'RRLS', 'VB')]
        discovery = self.path / 'all.discovery'
        discovery.write_bytes(b''.join(path.read_bytes() for _, path in records))
        allowed = ('GS', 'RRLS', '@addr:101', '@addr:102')
        denied = ('VB', 'gs', '@addr:103', '@addr:0', '@addr:4294967296',
                  '@addr:+101', '@addr:unknown', '@addr:4294967000')
        for sender in allowed + denied:
            with self.subTest(sender=sender):
                self.run_case('direct', sender, 'named', 'manual', records[0][0], discovery, int(sender in allowed))
        self.run_case('direct', 'GS', 'embeddednul', 'manual', records[0][0], discovery, 0)

    def test_actual_discovery_compact_collision_cannot_authorize_station_stop(self):
        names = ('GS', 'RRLS', 'identity-collision-51666', 'identity-collision-117548')
        records = [self.produce(sender) for sender in names]
        discovery = self.path / 'collision.discovery'
        discovery.write_bytes(b''.join(path.read_bytes() for _, path in records))
        self.run_case('collision', names[2], 'compact', 'manual', records[2][0], discovery, 0)


if __name__ == '__main__':
    unittest.main()

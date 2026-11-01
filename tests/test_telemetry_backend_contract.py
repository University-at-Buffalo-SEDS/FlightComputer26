"""Exercise the actual optional-backend CMake guard without compiling hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class TelemetryBackendContractTests(unittest.TestCase):
    def configure(self, telemetry, spinlock):
        source = (ROOT / 'CMakeLists.txt').read_text()
        start = source.index('if(ENABLE_TELEMETRY AND EXPORT_SPINLOCK)')
        end = source.index('endif()', start) + len('endif()')
        guard = source[start:end]
        with tempfile.TemporaryDirectory(prefix='fc-backend-contract-') as directory:
            path = Path(directory)
            (path / 'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.22)\nproject(BackendContract NONE)\n'
                'option(ENABLE_TELEMETRY "Telemetry" OFF)\n'
                'option(EXPORT_SPINLOCK "Legacy spin hook" OFF)\n' + guard + '\n')
            return subprocess.run(['cmake', '-S', str(path), '-B', str(path / 'build'),
                                   '-DENABLE_TELEMETRY=' + telemetry,
                                   '-DEXPORT_SPINLOCK=' + spinlock],
                                  capture_output=True, text=True)

    def test_supported_recursive_mutex_backend_configures(self):
        result = self.configure('ON', 'OFF')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_telemetry_spin_backend_is_rejected_actionably(self):
        result = self.configure('ON', 'ON')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Telemetry requires the recursive ThreadX mutex', result.stderr)
        self.assertIn('EXPORT_SPINLOCK=OFF', result.stderr)

    def test_legacy_nontelemetry_configuration_is_not_relabelled_qualified(self):
        result = self.configure('OFF', 'ON')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

if __name__ == '__main__':
    unittest.main()

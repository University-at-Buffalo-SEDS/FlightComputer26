"""Exercise real CMake cache migration for the captured nested-outliner defect."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CodegenTests(unittest.TestCase):
    def test_cached_unsafe_iterations_are_removed_without_losing_other_flags(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "CMakeLists.txt").write_text(
                'cmake_minimum_required(VERSION 3.24)\nproject(flags NONE)\n'
                f'include("{ROOT / "cmake/sedsnet_codegen.cmake"}")\n')
            for old in [None,
                        '-C target-cpu=cortex-m33 -Cllvm-args=-machine-outliner-reruns=2',
                        '-C target-cpu=cortex-m33 -C llvm-args=-machine-outliner-reruns=12 -Cllvm-args=-inline-threshold=15 -C opt-level=z']:
                command = ['cmake', '-S', str(root), '-B', str(root/'build')]
                if old is not None:
                    command.append('-DSEDSNET_ENV_RUSTFLAGS='+old)
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                flags = next(line.split('=',1)[1] for line in
                             (root/'build/CMakeCache.txt').read_text().splitlines()
                             if line.startswith('SEDSNET_ENV_RUSTFLAGS:STRING='))
                self.assertEqual(flags.count('machine-outliner-reruns='), 1)
                self.assertIn('machine-outliner-reruns=0', flags)
                self.assertIn('target-cpu=cortex-m33', flags)
                if old and 'inline-threshold=15' in old:
                    self.assertIn('inline-threshold=15', flags)
                    self.assertIn('opt-level=z', flags)
                else:
                    self.assertIn('inline-threshold=20', flags)

if __name__ == '__main__':
    unittest.main()

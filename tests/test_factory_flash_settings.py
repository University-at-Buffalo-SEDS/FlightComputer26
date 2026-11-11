"""A wired factory flash erases the persistent store unless asked to keep it.

The factory image ends before the LaunchCore store (0x0807C000), so a reflash
used to keep whatever the part held, including a store left mid-write or bytes
from an older layout that the strict reader refuses for good (the bench FC
refused every flight command). Erasing is the default; --keep-settings (legacy
token keep-settings) keeps the store. The tool says exactly what is wiped.
"""
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build  # noqa: E402

PERSIST_BASE = 0x0807C000
PERSIST_SIZE = 0x4000


class FactoryFlashSettingsTests(unittest.TestCase):
    def plan(self, argv):
        args = build.make_parser().parse_args(argv)
        _, options = build.cli_build_options(args)
        with tempfile.TemporaryDirectory() as directory:
            factory = Path(directory) / 'FlightComputer26.factory.bin'
            factory.write_bytes(b'\0' * 64)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                writes = build.factory_flash_writes(factory, build.FC_ADDR, options, Path(directory))
            erase = [(path, address) for path, address in writes if int(address, 16) == PERSIST_BASE]
            data = erase[0][0].read_bytes() if erase else None
            return writes, erase, data, out.getvalue()

    def test_default_factory_flash_erases_the_store_and_says_what_is_lost(self):
        for method in ('stm32prog-cli', 'st-flash', 'dfu'):
            with self.subTest(method=method):
                writes, erase, data, text = self.plan(['flash', '--release', '--method', method])
                self.assertEqual(len(erase), 1)
                self.assertEqual(data, b'\xff' * PERSIST_SIZE)
                self.assertEqual(writes[0][1], hex(PERSIST_BASE), 'erase before the image boots')
                for wiped in ('boot identity', 'recovery floor', 'PENDING Stop hold', 'flight state',
                              'underglow', 'buzzer', '--keep-settings'):
                    self.assertIn(wiped, text)

    def test_keep_settings_and_non_factory_images_leave_the_store(self):
        _, erase, _, text = self.plan(['flash', '--release', '--keep-settings'])
        self.assertEqual(erase, [])
        self.assertIn('kept', text)
        _, erase, _, _ = self.plan(['flash', '--release', '--image', 'firmware'])
        self.assertEqual(erase, [])
        preset, options = build.parse(['release', 'factory', 'flash-st', 'keep-settings'])
        self.assertTrue(options['keep-settings'])

    def test_every_programmer_writes_the_erase_image(self):
        with tempfile.TemporaryDirectory() as directory:
            factory = Path(directory) / 'f.bin'
            factory.write_bytes(b'\0')
            for key in ('flash-st', 'flash-stlink', 'flash-dfu'):
                options = {name: False for name in build.ALL_OPTIONS}
                options[key] = True
                options['image'] = 'factory'
                commands = []
                with mock.patch.object(build, 'require_tool', side_effect=lambda name: name), \
                     mock.patch.object(build, 'verify_stm32_target'), \
                     mock.patch.object(build, 'run', side_effect=commands.append), \
                     mock.patch.object(build, 'run_dfu', side_effect=commands.append), \
                     contextlib.redirect_stdout(io.StringIO()):
                    build.flash(factory, build.FC_ADDR, options, Path(directory))
                flat = ' '.join(' '.join(c) for c in commands)
                with self.subTest(key=key):
                    self.assertIn('settings-erase.bin', flat)
                    self.assertTrue(flat.index('settings-erase.bin') < flat.index('f.bin'))


if __name__ == '__main__':
    unittest.main()

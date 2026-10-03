import tempfile
import unittest
from pathlib import Path
from unittest import mock
import build

class WatchdogBuildFlagTests(unittest.TestCase):
    def test_watchdog_flag_reaches_cmake_and_default_turns_it_off(self):
        for enabled in (False, True):
            _, options = build.parse(["release"] + (["watchdog"] if enabled else []))
            self.assertEqual(options['watchdog'], enabled)
            if enabled:
                self.assertTrue(build.parse(['release','--watchdog'])[1]['watchdog'])
            commands=[]
            with tempfile.TemporaryDirectory() as directory:
                with mock.patch.object(build,'PROJECT',Path(directory)):
                    with mock.patch.object(build,'run',side_effect=commands.append):
                        build.configure(Path(directory)/'build','Release',options)
            self.assertIn('-DENABLE_BOARD_WATCHDOG='+('ON' if enabled else 'OFF'),commands[0])

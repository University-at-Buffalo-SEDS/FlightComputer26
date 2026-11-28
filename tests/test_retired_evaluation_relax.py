"""Retired operator commands (docs/task-layout.md, docs/rud-abort.md).

EvaluationRelax/EvaluationFocus switched the evaluator's preemption
threshold; the estimator guard (thread_comm.c) replaced that. The operator's
EvaluationAbort is now part of the reset to Startup. The runtime-option
commands (MonitorAltitude, ConsecutiveSamples, ResetFailures, ValidateMeasms,
velocity checks, their revokes) and the failure thresholds (AbortAfter*,
ReinitAfter*) are retired: the FC keeps its compiled defaults. Every id stays
reserved, an old station's copy is refused cleanly (acknowledged, never
executed), and no option bit can be set from the network."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def enum_block(text, start, end):
    return text[text.index(start):text.index(end, text.index(start))]


class RetiredEvaluationRelax(unittest.TestCase):
    def setUp(self):
        self.types = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        self.dist = (ROOT / 'Core/Src/sensor_thread.c').read_text()
        self.recovery = (ROOT / 'Core/Src/control_thread.c').read_text()

    def test_ids_stay_reserved(self):
        self.assertIn('Retired_Evaluation_Relax = Actionable_Decrees + 9,', self.types)
        self.assertIn('Retired_Evaluation_Focus = Actionable_Decrees + 10,', self.types)
        self.assertIn('Retired_Evaluation_Abort = Actionable_Decrees + 11,', self.types)
        self.assertIn('SD_Retry         = Actionable_Decrees + 21,', self.types)
        self.assertIn('Retired_Eval_Successive = Runtime_Configuration | (1u << 1)', self.types)
        self.assertIn('Retired_Revoke_Option = Runtime_Configuration | (1u << 28)', self.types)
        self.assertIn('Retired_Abortion_Thresholds = Runtime_Configuration | (1u << 26)', self.types)
        self.assertIn('Retired_Reinit_Thresholds   = Runtime_Configuration | (1u << 27)', self.types)
        self.assertIn('Retired_Eval_Focused = Runtime_Configuration | (1u << 2)', self.types)
        compat = enum_block(self.types, 'typedef enum remote_cmd_compat', '} compat;')
        names = re.findall(r'\n\s*(\w+),', compat)
        self.assertEqual(names.index('Compat_Retired_Evaluation_Relax'), 14)
        self.assertEqual(names.index('Compat_Retired_Evaluation_Focus'), 15)
        self.assertEqual(names.index('Compat_Retired_Evaluation_Abort'), 16)
        retired = {2: 'Compat_Retired_Vigilant_Mode', 3: 'Compat_Retired_Revoke_Vigilant_Mode',
                   4: 'Compat_Retired_Eval_Successive', 5: 'Compat_Retired_Revoke_Eval_Successive',
                   6: 'Compat_Retired_Reset_Failures', 7: 'Compat_Retired_Revoke_Reset_Failures',
                   8: 'Compat_Retired_Measm_Reports', 9: 'Compat_Retired_Revoke_Measm_Reports',
                   10: 'Compat_Retired_Velocity_Checks', 11: 'Compat_Retired_Revoke_Velocity_Checks',
                   23: 'Compat_Retired_Abort_After_40', 24: 'Compat_Retired_Abort_After_100',
                   25: 'Compat_Retired_Abort_After_250', 26: 'Compat_Retired_Reinit_After_15',
                   27: 'Compat_Retired_Reinit_After_30', 28: 'Compat_Retired_Reinit_After_50'}
        for index, name in retired.items():
            self.assertEqual(names[index], name)
        self.assertEqual(names.index('Compat_SD_Retry'), 33)
        self.assertEqual(len(names), 34)

    def test_one_byte_map_refuses_every_retired_byte(self):
        table = self.dist[self.dist.index('static const fc_msg extmap[Compat_Messages] = {'):]
        table = table[table.index('{') + 1:table.index('};')]
        entries = [e.strip() for e in re.sub(r'/\*.*?\*/', '', table, flags=re.S).split(',') if e.strip()]
        self.assertEqual(entries[13], 'Expand_Parachute')
        self.assertEqual(entries[14], 'Invalid_Message')
        self.assertEqual(entries[15], 'Invalid_Message')
        self.assertEqual(entries[16], 'Invalid_Message')
        for index in list(range(2, 12)) + list(range(23, 29)):
            self.assertEqual(entries[index], 'Invalid_Message', index)
        self.assertEqual(entries[12], 'Deploy_Parachute')
        self.assertEqual(entries[17:23], ['Reinit_Sensors', 'Reinit_Barometer', 'Reinit_IMU',
                                          'Disable_IMU', 'Advance_State', 'Rewind_State'])
        self.assertEqual(entries[29:], ['IGT_Enter', 'IGT_Live_Pyro', 'IGT_Dry_Run', 'IGT_Exit',
                                        'SD_Retry'])
        self.assertNotIn('revoke(', self.dist)
        self.assertNotIn('Thresholds', table)
        # An invalid command is refused for good and acknowledged.
        dispatch = self.dist[self.dist.index('dispatch_flight_cmd(const uint8_t *data, size_t len)'):]
        dispatch = dispatch[:dispatch.index('\n}\n')]
        self.assertIn('msg == Invalid_Message', dispatch)
        self.assertIn('st == INVALID_MESSAGE_STATUS ? SEDS_OK', dispatch)

    def test_full_width_ids_are_not_admissible(self):
        start = self.recovery.index('bool fc_recovery_command_admissible(')
        body = self.recovery[start:self.recovery.index('\n}\n', start)]
        first = body.index('if (retired_decree(cmd)) return false;')
        self.assertLess(first, body.index('fc_recovery_holds_sequence()'))
        self.assertLess(body.index('if (!internal && (cmd & Runtime_Configuration)) return false;'),
                        body.index('fc_recovery_holds_sequence()'))
        retired = self.recovery[self.recovery.index('static inline bool retired_decree('):]
        retired = retired[:retired.index('\n}\n')]
        for name in ('Retired_Evaluation_Relax', 'Retired_Evaluation_Focus', 'Retired_Evaluation_Abort'):
            self.assertIn(name, retired)
        self.assertNotIn('evaluation_configure', self.recovery)
        self.assertNotRegex(self.recovery, r'case\s+(Retired_)?Evaluation_(Relax|Focus|Abort)\s*:')

    def test_no_option_or_threshold_can_be_set_remotely(self):
        for gone in ('update_global_config', 'process_config_update', 'user_options',
                     'confmap', 'Abortion_Thresholds', 'Reinit_Thresholds'):
            self.assertNotRegex(self.recovery, r'\b%s\b' % gone)
        for header in ('Core/Inc/fcconfig.h', 'Core/Test/simulation.h'):
            text = (ROOT / header).read_text()
            self.assertNotIn('Eval_Focused', text, header)
            self.assertNotIn('Eval_Successive', text, header)
        self.assertNotIn('"focused"', self.recovery)
        # Defaults the FC keeps (fcconfig.h): the four options on, the
        # successive-sample option off, the compiled failure thresholds.
        config = (ROOT / 'Core/Inc/fcconfig.h').read_text()
        options = config[config.index('#define USER_OPTIONS'):]
        options = options[:options.index(') )')]
        for name in ('Reset_Failures', 'Measm_Reports', 'Velocity_Checks', 'Vigilant_Mode'):
            self.assertIn('option(%s)' % name, options)
        self.assertIn('static sysmon smon = {TO_ABORT, TO_REINIT, 0, 0, 0};', self.recovery)
        evaluation = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        self.assertNotIn('option(Eval_Successive)', evaluation)


if __name__ == '__main__':
    unittest.main()

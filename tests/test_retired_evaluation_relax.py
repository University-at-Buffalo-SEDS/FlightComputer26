"""EvaluationRelax/EvaluationFocus are retired (docs/task-layout.md).

They switched the evaluator's preemption threshold; the estimator guard
(thread_comm.c) replaced that, so they have no role left. Their ids stay
reserved, an old station's copy is refused cleanly (acknowledged, never
executed), and the Focused option bit can no longer be set."""
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
        self.assertIn('Evaluation_Abort = Actionable_Decrees + 11,', self.types)
        self.assertIn('Retired_Eval_Focused = Runtime_Configuration | (1u << 2)', self.types)
        compat = enum_block(self.types, 'typedef enum remote_cmd_compat', '} compat;')
        names = re.findall(r'\n\s*(\w+),', compat)
        self.assertEqual(names.index('Compat_Retired_Evaluation_Relax'), 14)
        self.assertEqual(names.index('Compat_Retired_Evaluation_Focus'), 15)
        self.assertEqual(names.index('Compat_Evaluation_Abort'), 16)

    def test_one_byte_map_refuses_14_and_15(self):
        table = self.dist[self.dist.index('static const fc_msg extmap[Compat_Messages] = {'):]
        table = table[table.index('{') + 1:table.index('};')]
        entries = [e.strip() for e in re.sub(r'/\*.*?\*/', '', table, flags=re.S).split(',') if e.strip()]
        self.assertEqual(entries[13], 'Expand_Parachute')
        self.assertEqual(entries[14], 'Invalid_Message')
        self.assertEqual(entries[15], 'Invalid_Message')
        self.assertEqual(entries[16], 'Evaluation_Abort')
        # An invalid command is refused for good and acknowledged.
        dispatch = self.dist[self.dist.index('dispatch_flight_cmd(const uint8_t *data, size_t len)'):]
        dispatch = dispatch[:dispatch.index('\n}\n')]
        self.assertIn('msg == Invalid_Message', dispatch)
        self.assertIn('st == INVALID_MESSAGE_STATUS ? SEDS_OK', dispatch)

    def test_full_width_ids_are_not_admissible(self):
        start = self.recovery.index('bool fc_recovery_command_admissible(')
        body = self.recovery[start:self.recovery.index('\n}\n', start)]
        first = body.index('Retired_Evaluation_Relax')
        self.assertLess(first, body.index('fc_recovery_holds_sequence()'))
        self.assertIn('return false;', body[first:first + 200])
        self.assertNotIn('evaluation_configure', self.recovery)
        self.assertNotRegex(self.recovery, r'case\s+(Retired_)?Evaluation_(Relax|Focus)\s*:')

    def test_focused_option_cannot_be_set_or_defaulted(self):
        opts = self.recovery[self.recovery.index('static inline constexpr fc_msg user_options()'):]
        opts = opts[:opts.index('\n}\n')]
        self.assertIn('& ~option(Retired_Eval_Focused)', opts)
        for header in ('Core/Inc/fcconfig.h', 'Core/Test/simulation.h'):
            self.assertNotIn('Eval_Focused', (ROOT / header).read_text(), header)
        self.assertNotIn('"focused"', self.recovery)


if __name__ == '__main__':
    unittest.main()

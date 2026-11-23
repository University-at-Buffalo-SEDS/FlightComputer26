"""The evaluation thread runs each pass without preemption (data-pipeline 9.5).

Pins the facts the design relies on: preemption-threshold is compiled out, so
the evaluation thread's threshold is 0 while Focused; no thread uses
priorities 1-4 (a narrower threshold would equal no threshold); and the
retained checkpoint restores the threshold it raised."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def macro(text, name):
    return int(re.search(r'#define\s+%s\s+(\d+)' % name, text).group(1))


class EvaluationPreemption(unittest.TestCase):
    def test_threshold_is_zero_and_kernel_support_compiled_out(self):
        tx_user = (ROOT / 'Core/Inc/tx_user.h').read_text()
        self.assertRegex(tx_user, r'\n#define TX_DISABLE_PREEMPTION_THRESHOLD\n')
        tasks = (ROOT / 'Core/Inc/fctasks.h').read_text()
        self.assertEqual(macro(tasks, 'EVAL_PREEMPT_THRESHOLD'), 0)
        self.assertEqual(macro(tasks, 'EVAL_PRIORITY'), 5)
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        focus = recovery[recovery.index('static inline void evaluation_configure('):]
        focus = focus[:focus.index('\n}\n')]
        self.assertIn('eval_new_pt = EVAL_PREEMPT_THRESHOLD;', focus)
        self.assertIn('eval_new_pt = EVAL_PRIORITY;', focus)

    def test_no_thread_between_recovery_and_the_application_group(self):
        tasks = (ROOT / 'Core/Inc/fctasks.h').read_text()
        priorities = {name: macro(tasks, name) for name in
                      ('RECV_PRIORITY', 'EVAL_PRIORITY', 'DIST_PRIORITY', 'DMA_PRIORITY', 'TLMT_PRIORITY')}
        self.assertEqual(priorities['RECV_PRIORITY'], 0)
        self.assertFalse([n for n, p in priorities.items() if 1 <= p <= 4], priorities)
        creates = []
        for path in list((ROOT / 'Core/Src').glob('*.c')) + [ROOT / 'FileX/App/app_filex.c']:
            text = path.read_text()
            creates += [path.name for _ in re.finditer(r'\btx_thread_create\(', text)]
        self.assertEqual(sorted(creates), ['app_filex.c', 'distribution.c', 'dma.c',
                                           'evaluation.c', 'recovery.c', 'telemetry.c'])
        filex = (ROOT / 'FileX/App/app_filex.c').read_text()
        self.assertIn('FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD', filex)
        self.assertEqual(macro(filex, 'FX_APP_THREAD_PRIO'), 5)
        self.assertNotRegex((ROOT / 'Core/Inc/tx_user.h').read_text(), r'\n#define TX_TIMER_THREAD_PRIORITY')

    def test_checkpoint_restores_its_threshold(self):
        src = (ROOT / 'Core/Src/fc_watchdog_recovery.c').read_text()
        body = src[src.index('void fc_watchdog_checkpoint(void)'):]
        body = body[:body.index('\n}\n')]
        self.assertIn('tx_thread_preemption_change(thread, 0U, &old_threshold)', body)
        self.assertTrue(body.rstrip().endswith('tx_thread_preemption_change(thread, old_threshold, &discarded);'))


if __name__ == '__main__':
    unittest.main()

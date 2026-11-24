"""An evaluation pass never interleaves a recovery command (data-pipeline 9.5).

A manual Deploy/Expand runs descent_initialize, which rebinds the Kalman
matrices to the 4x4 descent set; an ascent predict/update interrupted by it
would write 6x6 results into them. The protection is the explicit estimator
guard in thread_comm.c, not a scheduler setting: the evaluation thread holds
it for each pass, the recovery thread for each command and control step, and
the evaluator gives it up wherever it sleeps. No operator command turns it
off."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def macro(text, name):
    return int(re.search(r'#define\s+%s\s+(\d+)' % name, text).group(1))


def body(text, signature):
    start = text.index(signature)
    end = text.find('\n}\n', start)
    return text[start:end if end >= 0 else len(text)]


class EvaluationPreemption(unittest.TestCase):
    def test_no_scheduler_trick_left(self):
        tasks = (ROOT / 'Core/Inc/fctasks.h').read_text()
        self.assertNotIn('EVAL_PREEMPT_THRESHOLD', tasks)
        self.assertEqual(macro(tasks, 'EVAL_PRIORITY'), 5)
        evaluation = (ROOT / 'Core/Src/evaluation.c').read_text()
        create = body(evaluation, 'UINT create_evaluation_task(')
        create = re.sub(r'/\*.*?\*/', '', create, flags=re.S)
        self.assertRegex(create, r'EVAL_PRIORITY,\s*EVAL_PRIORITY,\s*TX_NO_TIME_SLICE,')
        for name in ('evaluation.c', 'recovery.c'):
            src = (ROOT / 'Core/Src' / name).read_text()
            self.assertNotIn('tx_thread_preemption_change', src, name)
            self.assertNotIn('tx_thread_priority_change', src, name)

    def test_evaluator_holds_the_guard_for_each_pass_and_sleeps_without_it(self):
        evaluation = (ROOT / 'Core/Src/evaluation.c').read_text()
        loop = body(evaluation, 'void evaluation_entry(ULONG _)')
        start = loop.index('MrAnalog (conf & option(Eval_Abort_Flag))')
        head, passes = loop[:start], loop[start:]
        # Flight-mode setup (ascent_initialize) and each pass are guarded.
        self.assertLess(head.index('thread_comm_estimator_enter();'), head.index('enter_flight_mode(conf);'))
        self.assertLess(head.index('enter_flight_mode(conf);'), head.index('thread_comm_estimator_leave();'))
        # The guard is taken after the pause barrier (a quiescent evaluator never
        # holds it) and given back before the next wait.
        self.assertLess(passes.rindex('evaluation_control_barrier();'), passes.index('thread_comm_estimator_enter();'))
        self.assertLess(passes.index('thread_comm_estimator_enter();'), passes.index('ascent_update();'))
        self.assertLess(passes.index('ascent_predict(dt, conf);'), passes.index('thread_comm_estimator_leave();'))
        self.assertLess(passes.index('tx_event_flags_get(&eval_stage'), passes.index('thread_comm_estimator_enter();'))
        # Every sleep in the state machine releases the guard; the only plain
        # sleep is the pause barrier's, which runs without it.
        barrier = body(evaluation, 'static void evaluation_control_barrier(void)')
        self.assertEqual(len(re.findall(r'\btx_thread_sleep\(', evaluation)), 1)
        self.assertIn('tx_thread_sleep(1U);', barrier)
        self.assertEqual(evaluation.count('thread_comm_estimator_sleep('), 3)

    def test_recovery_runs_commands_and_control_under_the_guard(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        loop = body(recovery, 'void recovery_entry(ULONG st)')
        loop = loop[loop.index('MrAnalog (WE_ARE_SO_BACK)'):]
        lines = [l.strip() for l in loop.splitlines()]
        held = False
        for line in lines:
            if line == 'thread_comm_estimator_enter();':
                self.assertFalse(held); held = True
            elif line == 'thread_comm_estimator_leave();':
                self.assertTrue(held); held = False
            elif re.search(r'\b(decode_flight_message|fc_recovery_control_service)\(', line):
                self.assertTrue(held, line)
            elif re.search(r'\b(tx_queue_receive|tx_thread_sleep)\(', line) or line.startswith('continue'):
                self.assertFalse(held, line)
        self.assertFalse(held)

    def test_no_thread_between_recovery_and_the_application_group(self):
        tasks = (ROOT / 'Core/Inc/fctasks.h').read_text()
        priorities = {name: macro(tasks, name) for name in
                      ('RECV_PRIORITY', 'EVAL_PRIORITY', 'DIST_PRIORITY', 'TLMT_PRIORITY')}
        self.assertEqual(priorities['RECV_PRIORITY'], 0)
        self.assertFalse([n for n, p in priorities.items() if 1 <= p <= 4], priorities)
        creates = []
        for path in list((ROOT / 'Core/Src').glob('*.c')) + [ROOT / 'FileX/App/app_filex.c']:
            text = path.read_text()
            creates += [path.name for _ in re.finditer(r'\btx_thread_create\(', text)]
        self.assertEqual(sorted(creates), ['app_filex.c', 'distribution.c',
                                           'evaluation.c', 'recovery.c', 'telemetry.c'])
        filex = (ROOT / 'FileX/App/app_filex.c').read_text()
        self.assertIn('FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD', filex)
        self.assertEqual(macro(filex, 'FX_APP_THREAD_PRIO'), 5)
        self.assertNotRegex((ROOT / 'Core/Inc/tx_user.h').read_text(), r'\n#define TX_TIMER_THREAD_PRIORITY')

    def test_checkpoint_restores_its_threshold(self):
        src = (ROOT / 'Core/Src/fc_watchdog_recovery.c').read_text()
        cp = body(src, 'void fc_watchdog_checkpoint(void)')
        self.assertIn('tx_thread_preemption_change(thread, 0U, &old_threshold)', cp)
        self.assertTrue(cp.rstrip().endswith('tx_thread_preemption_change(thread, old_threshold, &discarded);'))


if __name__ == '__main__':
    unittest.main()

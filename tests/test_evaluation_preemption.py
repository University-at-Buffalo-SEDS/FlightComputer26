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
        tasks = (ROOT / 'Core/Inc/FC-Threads.h').read_text()
        self.assertNotIn('EVAL_PREEMPT_THRESHOLD', tasks)
        self.assertEqual(macro(tasks, 'ESTIMATOR_PRIORITY'), 5)
        evaluation = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        create = body(evaluation, 'UINT create_estimator_thread(')
        create = re.sub(r'/\*.*?\*/', '', create, flags=re.S)
        self.assertRegex(create, r'ESTIMATOR_PRIORITY,\s*ESTIMATOR_PRIORITY,\s*TX_NO_TIME_SLICE,')
        for name in ('estimator_thread.c', 'control_thread.c'):
            src = (ROOT / 'Core/Src' / name).read_text()
            self.assertNotIn('tx_thread_preemption_change', src, name)
            self.assertNotIn('tx_thread_priority_change', src, name)

    def test_evaluator_holds_the_guard_for_each_pass_and_sleeps_without_it(self):
        evaluation = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        loop = body(evaluation, 'void estimator_thread_entry(ULONG _)')
        start = loop.index('while (!(conf & option(Eval_Abort_Flag)))')
        head, passes = loop[:start], loop[start:]
        # Flight-mode setup (ascent_initialize) and each pass are guarded.
        self.assertLess(head.index('thread_comm_estimator_enter();'), head.index('enter_flight_mode(conf);'))
        self.assertLess(head.index('enter_flight_mode(conf);'), head.index('thread_comm_estimator_leave();'))
        # The guard is taken after the pause barrier (a quiescent evaluator never
        # holds it) and given back before the next wait.
        self.assertLess(passes.rindex('evaluation_control_barrier();'), passes.index('thread_comm_estimator_enter();'))
        self.assertLess(passes.index('thread_comm_estimator_enter();'), passes.index('estimator_ascent_update();'))
        self.assertLess(passes.index('ascent_predict(dt, conf);'), passes.rindex('thread_comm_estimator_leave();'))
        # The descent filter and state machine run in the same guarded pass.
        self.assertLess(passes.index('thread_comm_estimator_enter();'), passes.index('estimator_descent_step(conf, dt);'))
        self.assertLess(passes.index('estimator_descent_step(conf, dt);'), passes.rindex('thread_comm_estimator_leave();'))
        # Every early exit gives the guard back.
        self.assertEqual(passes.count('thread_comm_estimator_enter();'), 1)
        self.assertEqual(passes.count('thread_comm_estimator_leave();'), 2)
        # A manual switch to the descent filter wakes an estimator waiting
        # for an IMU pair.
        recovery = (ROOT / 'Core/Src/control_thread.c').read_text()
        manual = body(recovery, 'static inline void manual_deployment(')
        self.assertLess(manual.index('descent_initialize(g_conf);'),
                        manual.index('tx_event_flags_set(&eval_stage, EVAL_STAGE_WAKE, TX_OR);'))
        self.assertLess(passes.index('tx_event_flags_get(&eval_stage'), passes.index('thread_comm_estimator_enter();'))
        # Every sleep in the state machine releases the guard; the only plain
        # sleep is the pause barrier's, which runs without it.
        barrier = body(evaluation, 'static void evaluation_control_barrier(void)')
        self.assertEqual(len(re.findall(r'\btx_thread_sleep\(', evaluation)), 1)
        self.assertIn('tx_thread_sleep(1U);', barrier)
        self.assertEqual(evaluation.count('thread_comm_estimator_sleep('), 3)

    def test_recovery_runs_commands_and_control_under_the_guard(self):
        recovery = (ROOT / 'Core/Src/control_thread.c').read_text()
        loop = body(recovery, 'void control_thread_entry(ULONG st)')
        loop = loop[loop.index('for (;;)'):]
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

    def test_no_time_slices(self):
        for name in ('sensor_thread.c', 'telemetry.c', 'control_thread.c', 'estimator_thread.c'):
            create = (ROOT / 'Core/Src' / name).read_text()
            create = create[create.index('tx_thread_create('):]
            create = re.sub(r'/\*.*?\*/', '', create[:create.index(');')], flags=re.S)
            self.assertIn('TX_NO_TIME_SLICE', create, name)
        self.assertNotRegex((ROOT / 'Core/Inc/FC-Threads.h').read_text(), r'TIME_SLICE')

    def test_no_thread_between_recovery_and_the_application_group(self):
        tasks = (ROOT / 'Core/Inc/FC-Threads.h').read_text()
        priorities = {name: macro(tasks, name) for name in
                      ('CONTROL_PRIORITY', 'ESTIMATOR_PRIORITY', 'SENSOR_PRIORITY', 'TELEMETRY_PRIORITY')}
        # Control above the application group, nothing in between.
        self.assertEqual(priorities['CONTROL_PRIORITY'], 4)
        self.assertEqual({p for n, p in priorities.items() if n != 'CONTROL_PRIORITY'}, {5})
        creates = []
        for path in list((ROOT / 'Core/Src').glob('*.c')) + [ROOT / 'FileX/App/app_filex.c']:
            text = path.read_text()
            creates += [path.name for _ in re.finditer(r'\btx_thread_create\(', text)]
        self.assertEqual(sorted(creates), ['app_filex.c', 'control_thread.c', 'estimator_thread.c',
                                           'sensor_thread.c', 'telemetry.c'])
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

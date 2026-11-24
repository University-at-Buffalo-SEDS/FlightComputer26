"""SD logging continues through Landed and Recovery (data-pipeline gap 9.1).

The flight FSM must not stop the SD writer at Recovery or sleep the thread
that consumes the sensors; the writer itself ends a bounded time after Landed
and then, with FC_SD_SERVICE, keeps the flushed volume for reads."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class RecoveryLogging(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.eval = (ROOT / 'Core/Src/evaluation.c').read_text()
        cls.sd = (ROOT / 'Core/Src/sdpipeline.c').read_text()

    def test_fsm_never_stops_the_sd_writer(self):
        self.assertNotIn('sd_conclude', self.eval)
        self.assertEqual([p for p in (ROOT / 'Core/Src').glob('*.c')
                          if p.name != 'sdpipeline.c' and 'sd_conclude(' in p.read_text()], [])

    def test_landed_and_recovery_do_not_sleep_the_sensor_thread(self):
        # Descent evaluation runs in the estimator (evaluation thread); the
        # distribution thread only publishes the samples that feed it, the SD
        # log and telemetry.
        publish = function((ROOT / 'Core/Src/distribution.c').read_text(), 'static inline void descent_publish(')
        self.assertNotIn('evaluate_rocket_state', publish)
        self.assertIn('tx_event_flags_set(&eval_stage, stage, TX_OR);', publish)
        self.assertIn('evaluate_rocket_state(conf, dt);', function(self.eval, 'void evaluation_entry('))
        landed = function(self.eval, 'static inline void detect_landed(')
        announce = function(self.eval, 'static inline void announce_recovery(')
        for body in (landed, announce):
            self.assertNotIn('tx_thread_sleep', body)
        self.assertIn('landed_ms = now_ms();', landed)
        self.assertRegex(announce, r'if \(\(uint32_t\)\(now_ms\(\) - landed_ms\) < RECOVERY_ANNOUNCE_DELAY\) return;\s+flight_advance\(Recovery\);')

    def test_writer_ends_itself_then_flushes_and_serves(self):
        task = self.sd[self.sd.index('void sd_pipeline_task()'):]
        self.assertIn('if (sd_recovery_log_expired()) { sd_conclude(); break; }', task)
        # Shutdown after the loop drains, flushes, closes and flushes again
        # with the read service before it starts serving.
        self.assertRegex(task, r'\}\n    sd_pipeline_shutdown\(surprise\);\n    sd_file_service_forever\(\);')
        # Reads and writes share this one thread: the read service runs only
        # between writer passes, so FileX is never entered concurrently.
        loop = task[:task.index('sd_pipeline_shutdown(surprise);')]
        self.assertTrue(loop.rstrip().endswith('sd_file_service_turn();\n    }'))
        self.assertEqual([p.name for p in (ROOT / 'Core/Src').glob('*.c')
                          if 'sd_file_service_turn' in p.read_text()], ['sdpipeline.c'])


if __name__ == '__main__':
    unittest.main()

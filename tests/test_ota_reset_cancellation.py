"""Actual FC reset owner/receiver regression; only hardware and RTOS mocked."""
from pathlib import Path
import importlib.util,json,os,subprocess,tempfile,unittest
BASE=Path(__file__).resolve().parents[1]
FIXTURE=Path(__file__).resolve().parent/'ota_reset'
CASES=['accepted_before_eval', 'checkpoint_owner_restore', 'manual_stop', 'reset_request_same_epoch', 'legacy_eval_stop_same_epoch', 'phase_aba_same_epoch', 'pyro_high_low', 'terminal', 'expired_newjob', 'held_router', 'foreign_worker', 'store_leave_failure', 'time_regression', 'mutation_during_finish', 'mutation_before_finish', 'unchanged_startup', 'finish_leave_failure']
def build(out):
 spec=importlib.util.spec_from_file_location('actual_admitted_builder',BASE/'tests/test_admitted_flight.py')
 a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
 base=a.build(out).read_text().replace('int main(int argc,char **argv)','int admitted_original_main(int argc,char **argv)').replace('enum state {','enum state : uint8_t {')
 # The existing admitted fixture starts Armed; this test starts at the ACTUAL
 # production Startup initializer and preserves mocked boot readiness boundaries.
 evaluation=(BASE/'Core/Src/evaluation.c').read_text()
 assert 'sv_meta sm = {Startup, G_Startup, 0, 0, 0};' in evaluation
 base=base.replace('.flight=Armed','.flight=Startup')
 pre='#include <stdint.h>\n#include <stdbool.h>\n#define TELEMETRY_ENABLED 1\nstatic uint32_t g_last_network_flight_command_msg;\nstatic void NVIC_SystemReset(void);\nvoid ota_stream_retire_reset_locked(void);\nbool ota_stream_take_reset_locked(uint32_t epoch);\n'
 base=pre+base
 base=base.replace('static uint32_t __get_PRIMASK(void)','static unsigned ipsr;\nstatic unsigned __get_IPSR(void){return ipsr;}\nstatic uint32_t __get_PRIMASK(void)')
 ota='\n'.join(l for l in (BASE/'Core/Src/ota_stream.c').read_text().splitlines() if not l.startswith('#include'))
 tele=(BASE/'Core/Src/telemetry.c').read_text()
 boundary=a.extract(tele,'bool fc_recovery_ota_reset_boundary_ready(')
 wrapper=a.extract(tele,'SedsResult telemetry_poll_discovery(')
 rec=(BASE/'Core/Src/recovery.c').read_text()
 fwd=rec[rec.index('case Advance_State:'):rec.index('case Rewind_State:')].split(':',1)[1].replace('break;','')
 rev=rec[rec.index('case Rewind_State:'):rec.index('case Log_Rate_Limit:')].split(':',1)[1].replace('break;','')
 common=(BASE/'Core/Inc/fccommon.h').read_text();macros=''
 for name in ['saturating_incr','saturating_decr']:
  st=common.index('#define '+name);macros+=common[st:common.index('\n\n',st)]+'\n'
 body=(FIXTURE/'body.c').read_text().replace('/*FORWARD*/',fwd).replace('/*REVERSE*/',rev)
 code=base+'\n#undef id\n'+(FIXTURE/'transport_stub.c').read_text()+'\n'+ota+'\n'+(FIXTURE/'telemetry_stub.c').read_text()+'\n'+boundary+'\n'+wrapper+'\n'+macros+'\n'+body
 src=out/'ota-reset.c';src.write_text(code);exe=out/'ota-reset'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-g','-I',str(BASE/'Core/Inc'),str(src),'-o',str(exe)],check=True)
 return exe
class OtaResetCancellation(unittest.TestCase):
 def test_actual_reset_cancellation_and_released_owner_boundary(self):
  with tempfile.TemporaryDirectory() as d:
   exe=build(Path(d));env=dict(os.environ,ASAN_OPTIONS='halt_on_error=1:detect_leaks=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
   for case in CASES:
    with self.subTest(case=case):subprocess.run([str(exe),case],check=True,env=env,timeout=5)
if __name__=='__main__':unittest.main()

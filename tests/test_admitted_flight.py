"""Actual recovery/pulse owner and evaluator regression; hardware/RTOS boundaries only."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
BASE=Path(__file__).resolve().parents[1]
FIXTURE=Path(__file__).resolve().parent/'admitted_flight'
def extract(s,signature):
 start=s.index(signature);end=s.index('{',start)+1;depth=1
 while depth:
  depth+=(s[end]=='{')-(s[end]=='}');end+=1
 return s[start:end]
def build(OUT):
 recovery=(BASE/'Core/Src/recovery.c').read_text();evaluation=(BASE/'Core/Src/evaluation.c').read_text();types=(BASE/'Core/Inc/fcstructs.h').read_text();enum=types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;')+len('} fc_msg;')].replace('flight_message : fu32','flight_message').replace('UINT_FAST32_MAX','UINT32_MAX')
 hooks=['bool fc_recovery_control_ready(','bool fc_recovery_control_is_startup(','void fc_recovery_evaluation_running(','void fc_recovery_evaluation_quiescent(','void fc_recovery_control_wake(','bool fc_recovery_control_quiesce(','void fc_recovery_control_mark_eval_stopped(','bool fc_recovery_control_finalize_startup(','bool fc_recovery_control_new_flight_ready(','void fc_recovery_control_new_flight(','static inline void enter_launch(','static inline bool flight_mission_active(','static inline void abortion_due_failures(','static inline void process_sensor_report(']
 actual_hooks='\n'.join(extract(recovery,sig)for sig in hooks)
 owner_pulse='\n'.join(line for line in ((BASE/'Core/Src/fc_recovery_owner.c').read_text()+'\n'+(BASE/'Core/Src/deployment_pulse.c').read_text()).splitlines()if not line.startswith('#include'))
 stub=(FIXTURE/'stub.c').read_text();body=(FIXTURE/'body.c').read_text()
 branch=extract(recovery,'if (fc_recovery_take_sensor_restart())')
 assert branch.count('decode_flight_message(fc_mask(Launch_Signal));')==1
 branch=branch.replace('decode_flight_message(fc_mask(Launch_Signal));','enter_launch(true);').replace('continue;', 'return;')
 body=body.replace('/*ACTUAL_SENSOR_RESTART_BRANCH*/',branch)
 source=stub.replace('/*ACTUAL_ENUM*/',enum)+owner_pulse+'\n'+actual_hooks+'\n'+extract(evaluation,'static void evaluation_control_barrier(')+'\n'+extract(evaluation,'static inline void enter_flight_mode(')+'\n'+body
 (OUT/'probe.c').write_text(source)
 return OUT/'probe.c'
class AdmittedFlightTests(unittest.TestCase):
 def test_actual_owner_sensor_handoff_and_epoch_controls(self):
  with tempfile.TemporaryDirectory() as directory:
   out=Path(directory);src=build(out);exe=out/'probe'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(BASE/'Core/Inc'),str(src),'-o',str(exe)],check=True)
   env=dict(os.environ,ASAN_OPTIONS='detect_leaks=1')
   for case in ['prelaunch','accepted_before_eval','accepted_preboost','accepted_then_manual_stop','accepted_then_reset','storage_unready_seed','commissioning_seed','verified_watchdog_seed','stale_reset_no_revoke','manual_eval_stop','internal_not_authority','reset_then_fresh_launch']:
    with self.subTest(case=case):subprocess.run([str(exe),case],env=env,check=True)

"""Actual FC owner/pulse source under post-admission storage loss; no output stubs replace owner effects."""
from pathlib import Path
import importlib.util, os, subprocess, tempfile, unittest
ROOT=Path(os.environ.get('FC_OWNER_SOURCE_DIR',str(Path(__file__).resolve().parents[1])))
spec=importlib.util.spec_from_file_location('existing_owner_fixture',ROOT/'tests/test_fc_recovery_owner.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
EXTRA=r'''
 if(!strcmp(argv[1],"postlaunch_explicit")||!strcmp(argv[1],"postlaunch_internal_resume")){
  bool internal=!strcmp(argv[1],"postlaunch_internal_resume");
  assert(fc_recovery_admit_new_flight(false));
  if(internal)assert(fc_recovery_admit_new_flight(true));
  assert(fc_recovery_flight_is_admitted());
  store_ready=false;
  assert(!fc_recovery_holds_sequence()&&!fc_recovery_inhibits_pyro());
  assert(fc_recovery_flight_is_admitted());
  assert(fc_recovery_admit_new_flight(true)); /* existing admitted mission resume */
  deployment_pulse_start(false,false);deployment_pulse_start(true,false);
  assert(highs==2&&gpio.ODR==3&&co2_ticks&&reef_ticks);
  fc_recovery_manual_stop();
  assert(fc_recovery_abort_latched()&&fc_recovery_inhibits_pyro());
  assert(!fc_recovery_flight_is_admitted()&&!gpio.ODR&&!co2_ticks&&!reef_ticks&&seal);
  deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(highs==2);
 }else if(!strcmp(argv[1],"first_internal_not_authority")){
  assert(fc_recovery_admit_new_flight(true));
  assert(!fc_recovery_flight_is_admitted());
  store_ready=false;assert(fc_recovery_holds_sequence()&&fc_recovery_inhibits_pyro());
  deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(!highs&&!gpio.ODR);
 }else if(!strcmp(argv[1],"prelaunch_store_failure")){
  store_ready=false;assert(!fc_recovery_admit_new_flight(false));
  assert(!fc_recovery_admit_new_flight(true)&&!fc_recovery_flight_is_admitted());
  deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(!highs&&!gpio.ODR);
  fc_recovery_manual_stop();assert(fc_recovery_abort_latched()&&seal&&!gpio.ODR);
 }else if(!strcmp(argv[1],"new_internal_still_refused")){
  reset_complete();arm_ready=true;
  assert(!fc_recovery_admit_new_flight(true)&&!fc_recovery_flight_is_admitted());
  assert(fc_recovery_admit_new_flight(false)&&fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"postlaunch_reset_failed_store")){
  assert(fc_recovery_admit_new_flight(false));store_ready=false;
  uint32_t epoch=fc_recovery_epoch();assert(fc_recovery_request_startup(epoch)==-14);
  assert(!fc_recovery_flight_is_admitted()&&fc_recovery_inhibits_pyro());
  quiet=true;io_error=true;fc_recovery_control_service();fc_recovery_network_service();fc_recovery_control_service();
  assert(fc_recovery_reset_failed()&&finals==0&&seal);
  arm_ready=true;assert(!fc_recovery_admit_new_flight(false)&&!fc_recovery_admit_new_flight(true));
  deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(!highs&&!gpio.ODR);
 }else
'''
class PostlaunchStoreContinuityTests(unittest.TestCase):
 def test_actual_owner_and_pyro_storage_fault(self):
  actual='\n'.join(line for name in ['fc_recovery_owner.c','deployment_pulse.c'] for line in (ROOT/'Core/Src'/name).read_text().splitlines() if not line.startswith('#include'))
  main=fixture.MAIN.replace(' if(!strcmp(argv[1],"store_unready")){',EXTRA+' if(!strcmp(argv[1],"store_unready")){',1)
  self.assertNotEqual(main,fixture.MAIN)
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);c=p/'actual.c';c.write_text(fixture.STUB+actual+main);exe=p/'actual'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),str(c),'-o',str(exe)],check=True)
   selected=os.environ.get('FC_STORAGE_CONTINUITY_CASE')
   for case in ([selected] if selected else ['postlaunch_explicit','postlaunch_internal_resume','first_internal_not_authority','prelaunch_store_failure','new_internal_still_refused','postlaunch_reset_failed_store']):
    with self.subTest(case=case): subprocess.run([str(exe),case],check=True)
if __name__=='__main__':unittest.main()

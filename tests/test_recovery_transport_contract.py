"""Actual receiver/strict journal/priority-zero FC owner; commanded pyro LOW proof."""
from pathlib import Path
from native_sdk import configured_dependency_source
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryTransportTests(unittest.TestCase):
 def test_actual_shared_transport_floor_and_result_mapping(self):
  with tempfile.TemporaryDirectory(prefix='strict-recovery-contract-')as temp:
   p=Path(temp);fixture=ROOT/'tests/fixtures/recovery_wire';sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE');exe=p/'proof'
   assert 'launchcore_persist_get_strict' in (lc/'bootloader/include/launchcore/persist.h').read_text()
   names=['board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_commissioning.c']
   rendered=(fixture/'harness.in.c').read_text()
   for name,tag in [('fc_recovery_owner.c','ACTUAL_FC_OWNER'),('deployment_pulse.c','ACTUAL_PULSE')]:
    actual='\n'.join(line for line in (ROOT/'Core/Src'/name).read_text().splitlines()if not line.startswith('#include'))
    rendered=rendered.replace('@'+tag+'@',actual)
   (p/'harness.c').write_text(rendered)
   subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/name)for name in names],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'harness.c'),'-o',str(exe)],check=True)
   for case in ['io','pending','stale','query_fault','authority','duplicate','unsupported_idle','virgin','floor']:
    with self.subTest(case=case):subprocess.run([str(exe),case],check=True)

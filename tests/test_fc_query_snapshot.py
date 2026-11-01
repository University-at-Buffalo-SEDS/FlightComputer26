"""Actual FC QUERY mapping follows evaluator/GPIO; cache contents are not state."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
STUB=r'''
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#include "board_recovery.h"
#define In_Aborted_State 1U
#define option(x) (x)
static uint32_t phase,irq,epoch=7,g_conf,outputs;
static bool ready,latched;
static unsigned requests;
static int32_t outcome=-14;
uint32_t current(void){return phase;}
uint32_t __get_PRIMASK(void){return irq;}
void __disable_irq(void){irq=1;}
void __set_PRIMASK(uint32_t value){irq=value;}
uint32_t fc_recovery_epoch(void){return epoch;}
bool fc_recovery_control_ready(void){return ready;}
bool deployment_pulse_safe_outputs_locked(uint32_t *mask){assert(irq);*mask=outputs;return outputs==0;}
bool fc_recovery_abort_latched(void){return latched;}
int32_t fc_recovery_request_startup(uint32_t expected){assert(expected==epoch);++requests;return outcome;}
'''
MAIN=r'''
int main(void){
 BoardRecoverySnapshot out;
 const uint8_t mapping[]={G_Startup,G_Armed,G_Armed,G_Launch,G_Ascent,G_Coast,G_Apogee,G_Descent,G_Reefing,G_Landed,G_Recovery};
 for(phase=Startup;phase<Flight_States;phase++){
  assert(board_recovery_snapshot(&out)==0&&out.actual_state==mapping[phase]&&out.abort_epoch==7);
  assert(out.flags==BOARD_RECOVERY_STATE_VALID&&!requests&&!irq); /* No invented output or support flags */
 }
 ready=true;phase=Launch;outputs=1;
 assert(board_recovery_snapshot(&out)==0&&out.actual_state==G_Launch&&out.actual_outputs_mask==1);
 assert((out.flags&(BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_OUTPUTS_VALID))==(BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_OUTPUTS_VALID));
 assert(!(out.flags&BOARD_RECOVERY_SAFE_OUTPUTS));
 outputs=0;latched=true;assert(board_recovery_snapshot(&out)==0);
 assert(out.flags&BOARD_RECOVERY_SAFE_OUTPUTS);assert(out.flags&BOARD_RECOVERY_ABORT_LATCHED);
 assert(board_recovery_apply_safe(1,7,&out)==-10&&!requests&&phase==Launch);
 assert(board_recovery_apply_safe(0,7,&out)==-14&&requests==1&&phase==Launch&&out.actual_state==G_Launch);
 outcome=-9;assert(board_recovery_apply_safe(0,7,&out)==-9&&requests==2&&out.actual_state==G_Launch);
 phase=Startup;outcome=0;assert(board_recovery_apply_safe(0,7,&out)==0&&out.actual_state==G_Startup);
 irq=1;assert(board_recovery_snapshot(&out)==0&&irq);irq=0;
 assert(board_recovery_snapshot(NULL)==-10&&!irq);
 return 0;
}
'''
class FCQuerySnapshotTests(unittest.TestCase):
 def test_actual_evaluator_mapping_and_async_apply_readback(self):
  types=(ROOT/'Core/Inc/fcstructs.h').read_text();enums=[]
  for start,end in [('typedef enum flight_state : fu8 {','} state;'),('typedef enum global_state : uint8_t {','} gnd_state;')]:
   block=types[types.index(start):types.index(end)+len(end)]
   enums.append(block.replace(' : fu8','').replace(' : uint8_t',''))
  actual='\n'.join(l for l in (ROOT/'Core/Src/board_recovery_query.c').read_text().splitlines()if not l.startswith('#include'))
  with tempfile.TemporaryDirectory()as directory:
   p=Path(directory);src=p/'query.c';src.write_text('\n'.join(enums)+STUB+actual+MAIN);exe=p/'query'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),str(src),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

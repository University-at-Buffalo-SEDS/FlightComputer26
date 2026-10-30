"""Execute production timer code with delayed observations and a wrapping clock."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


def timer_function(name):
    source=(ROOT/'Core/Inc/fcapi.h').read_text()
    start=source.index('static inline bool '+name+'(')
    end=source.index('\n}',start)+2
    return source[start:end]


def harness(function):
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t fu32;
typedef unsigned timer;
volatile fu32 local_time[4];
uint32_t tick;
/* A second observation costs a tick, as can happen across an IRQ boundary. */
uint32_t now_ms(void){return tick++;}
'''+function+r'''
int main(void){
 unsigned reports=0;
 for(unsigned opportunity=0;opportunity<=2000;opportunity+=50){
  tick=opportunity;
  if(timer_probe_periodic(0,200))reports++;
 }
 /* Keep the intended5Hz from a20Hz source despite time spent in the call. */
 assert(reports==10);
 local_time[1]=0;tick=1950;
 assert(timer_probe_periodic(1,200));
 tick=1950;assert(!timer_probe_periodic(1,200)); /* no catch-up burst */
 tick=2000;assert(timer_probe_periodic(1,200)); /* original phase */
 local_time[2]=UINT32_MAX-99;
 tick=99;assert(!timer_probe_periodic(2,200));
 tick=100;assert(timer_probe_periodic(2,200));
 assert(local_time[2]==100); /* correct phase through32-bit tick wrap */
 tick=250;assert(!timer_probe_periodic(2,200));
 tick=300;assert(timer_probe_periodic(2,200));
 local_time[3]=0;tick=8;assert(timer_probe_periodic(3,0));
 tick=8;assert(timer_probe_periodic(3,0)); /* preserve zero-period semantics */
}
'''


def execute(function):
    with tempfile.TemporaryDirectory() as directory:
        executable=Path(directory)/'timer'
        subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-x','c','-','-o',str(executable)],input=harness(function),text=True,check=True)
        return subprocess.run([str(executable)],capture_output=True,text=True)


class PeriodicTelemetry(unittest.TestCase):
    def test_phase_wrap_and_no_catchup_burst(self):
        result=execute(timer_function('timer_probe_periodic'))
        self.assertEqual(result.returncode,0,result.stderr)

    def test_reset_to_now_negative_control_loses_intended_rate(self):
        source=(ROOT/'Core/Inc/fcapi.h').read_text()
        start=source.index('static inline fu32 timer_fetch(')
        end=source.index('/* Periodic telemetry',start)
        legacy=source[start:end]
        # Supply the original update helper, then execute the old scheduling
        # policy under exactly the same timing assertions as the new helper.
        update='static inline void timer_update(timer u){local_time[u]=now_ms();}\n'
        result=execute(update+legacy.replace('timer_probe(', 'timer_probe_periodic('))
        self.assertNotEqual(result.returncode,0)
        self.assertIn('reports==10',result.stderr)

    def test_only_periodic_ground_reports_use_new_helper(self):
        distribution=(ROOT/'Core/Src/distribution.c').read_text()
        evaluation=(ROOT/'Core/Src/evaluation.c').read_text()
        storage=(ROOT/'Core/Src/sdpipeline.c').read_text()
        self.assertIn('timer_probe_ground(mems[dev].tim_gnd, ground_period)',distribution)
        self.assertIn('timer_probe(mems[dev].tim_sd, rates.sd)',distribution)
        self.assertIn('timer_probe_ground(KFRemote, rates.gnd)',evaluation)
        self.assertNotIn('timer_probe_periodic(',storage)

if __name__=='__main__':unittest.main()

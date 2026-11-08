"""Actual decoder/policy/sensor-failure functions; old autonomous intents stay fenced."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

def extract(source, signature):
    start=source.index(signature);end=source.index('{',start)+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

STUB=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#define id "FC "
#define TO_ABORT 4
#define option(x) ((x)&~Runtime_Configuration)
#define fc_mask(x) ((x)|FlightComputer_Mask)
#define fc_unmask(x) ((x)&~FlightComputer_Mask)
typedef int32_t fi32;
enum {Startup,Postinit,Armed,Flight};
static bool held,new_flight,restart_allowed=true;
static unsigned reports,gps,actions,configs,unknown,restarts,messages;
static uint32_t last_command,g_conf,phase=Flight;
static bool last_internal;
static struct {unsigned gps_delayed,gps_malform,to_abort;} smon;
static bool manual_inhibited=true;
bool fc_recovery_holds_sequence(void){return held;}
bool fc_recovery_inhibits_manual_pyro(void){return manual_inhibited;}
bool fc_recovery_flight_is_admitted(void){return false;}
bool fc_recovery_requires_new_flight(void){return new_flight;}
bool fc_recovery_request_sensor_restart(void){++restarts;return restart_allowed;}
bool beyond(unsigned state){return phase>state;}
void process_sensor_report(uint32_t msg){++reports;last_command=msg;}
void process_gps_code(uint32_t msg){++gps;last_command=msg;}
void process_action(uint32_t msg,bool internal){++actions;last_command=msg;last_internal=internal;}
void process_config_update(uint32_t msg){++configs;last_command=msg;}
void log_metric(const char *text,fi32 value,bool important){(void)text;(void)value;(void)important;++unknown;}
void message(const char *text,bool important){(void)text;(void)important;++messages;}
'''
MAIN=r'''
int main(void){
 held=true;new_flight=true;
 uint32_t forbidden[]={Launch_Signal,Deploy_Parachute,Expand_Parachute,Postinit_Signal,Bad_Altitude,GPS_Delayed};
 for(unsigned i=0;i<sizeof forbidden/sizeof forbidden[0];i++){
  decode_flight_message(forbidden[i]);decode_flight_message(fc_mask(forbidden[i]));
 }
 assert(!reports&&!gps&&!actions&&!configs&&!unknown);
 decode_flight_message(Evaluation_Abort);assert(actions==1&&last_command==Evaluation_Abort&&!last_internal);
 /* Held only by a rejected watchdog resume: operator deploy commands pass,
  * internal deploys and every other flight command stay fenced. */
 new_flight=false;manual_inhibited=false;
 decode_flight_message(fc_mask(Deploy_Parachute));decode_flight_message(fc_mask(Expand_Parachute));
 decode_flight_message(Launch_Signal);decode_flight_message(Postinit_Signal);decode_flight_message(Advance_State);
 assert(actions==1);
 decode_flight_message(Deploy_Parachute);assert(actions==2&&last_command==Deploy_Parachute&&!last_internal);
 decode_flight_message(Expand_Parachute);assert(actions==3&&last_command==Expand_Parachute);
 manual_inhibited=true;decode_flight_message(Deploy_Parachute);assert(actions==3);
 new_flight=true;actions=1;
 restart_allowed=false;smon.gps_delayed=7;smon.gps_malform=8;g_conf=123;
 abortion_due_failures();assert(restarts==1&&smon.gps_delayed==7&&smon.gps_malform==8&&g_conf==123&&!messages);
 held=false;
 decode_flight_message(fc_mask(Launch_Signal));assert(actions==1); /* old sensor restart */
 decode_flight_message(Deploy_Parachute);assert(actions==1);
 decode_flight_message(Launch_Signal);assert(actions==2&&!last_internal);
 decode_flight_message(fc_mask(Postinit_Signal));assert(actions==3&&last_internal);
 new_flight=false;restart_allowed=true;g_conf=option(Defer_Baro_Fallback);
 abortion_due_failures();assert(restarts==2&&!smon.gps_delayed&&!smon.gps_malform&&smon.to_abort==TO_ABORT*10);
 assert(g_conf==option(Graceful_Reset)&&!messages);
 decode_flight_message(fc_mask(Bad_Altitude));assert(reports==1&&last_command==Bad_Altitude);
 decode_flight_message(fc_mask(GPS_Delayed));assert(gps==1);
 decode_flight_message(fc_mask(Launch_Signal));assert(actions==4&&last_internal);
 decode_flight_message(Vigilant_Mode);assert(configs==1&&!unknown);
 return 0;
}
'''
class ControlEpochGateTests(unittest.TestCase):
    def test_actual_legacy_decoder_and_sensor_recovery(self):
        source=(ROOT/'Core/Src/recovery.c').read_text()
        types=(ROOT/'Core/Inc/fcstructs.h').read_text()
        enum=types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;')+len('} fc_msg;')]
        enum=enum.replace('flight_message : fu32','flight_message').replace('UINT_FAST32_MAX','UINT32_MAX')
        actual='\n'.join(extract(source,signature)for signature in [
            'bool fc_recovery_command_admissible(',
            'static inline void decode_flight_message(',
            'static inline bool flight_mission_active(',
            'static inline void abortion_due_failures('])
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);src=p/'gates.c';src.write_text('#include <stdint.h>\n'+enum+STUB+actual+MAIN);exe=p/'gates'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

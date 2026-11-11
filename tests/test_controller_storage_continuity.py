"""Actual receiver→queue→FC admission/launch owner after durable recovery.

Sensor readiness/watchdog peripherals retain the existing controlled fixture
boundary. No physical deployment or radio-network delivery is claimed.
"""
from pathlib import Path
from native_sdk import configured_dependency_source
import os,subprocess,tempfile,unittest,re
from test_controller_station_authority import function
ROOT=Path(os.environ.get("FC_RRLS_POLICY_SOURCE",str(Path(__file__).resolve().parents[1])))
class ControllerStorageContinuityTests(unittest.TestCase):
 def test_actual_recovery_then_station_controller_owner_and_guards(self):
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE');fixture=ROOT/'tests/fixtures/recovery_wire';text=(fixture/'harness.in.c').read_text();types=(ROOT/'Core/Inc/fcstructs.h').read_text();enum=types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;')+len('} fc_msg;')].replace('flight_message : fu32','flight_message').replace('UINT_FAST32_MAX','UINT32_MAX');compat=types[types.index('typedef enum remote_cmd_compat : uint8_t'):types.index('} compat;')+len('} compat;')].replace('remote_cmd_compat : uint8_t','remote_cmd_compat')
  text='#include <stdint.h>\n'+enum+'\n'+compat+'\n'+text
  text=text.replace('static int seds_syscall,eval_stage,g_last_network_flight_command_msg;\n#define Invalid_Message 255','static int seds_syscall,eval_stage;static fc_msg g_last_network_flight_command_msg;')
  text=text.replace('static uint64_t clock_ms=10000;','static uint64_t clock_ms=10000;static bool controller_clock_ok=true;').replace('*out=clock_ms;return SEDS_OK;','*out=clock_ms;return controller_clock_ok?SEDS_OK:SEDS_ERR;')
  for name,tag in [('fc_recovery_owner.c','ACTUAL_FC_OWNER'),('deployment_pulse.c','ACTUAL_PULSE')]:text=text.replace('@'+tag+'@','\n'.join(line for line in(ROOT/'Core/Src'/name).read_text().splitlines()if not line.startswith('#include')))
  distribution=(ROOT/'Core/Src/distribution.c').read_text();recovery=(ROOT/'Core/Src/recovery.c').read_text();begin=distribution.index('#if defined (TELEMETRY_ENABLED) || defined(FAKESTATION)');end=distribution.index('#endif /* TELEMETRY_ENABLED + FAKESTATION */',begin)+len('#endif /* TELEMETRY_ENABLED + FAKESTATION */');decoder=distribution[begin:end]
  support=r'''
#define TELEMETRY_CMD_COMPAT
#define id "FC "
#define revoke(x) ((x)|Revoke_Option)
#define fc_mask(x) ((x)|FlightComputer_Mask)
#define fc_unmask(x) ((x)&~FlightComputer_Mask)
#define INVALID_MESSAGE_STATUS 1
#define message(...) ((void)0)
typedef int32_t fi32;
static bool queued;static fc_msg queued_message;static unsigned owner_deliveries;
static unsigned g_network_flight_commands_received,g_network_flight_commands_accepted,g_last_network_flight_command_id;
static unsigned devctr_if[2];static struct{unsigned gps_delayed,gps_malform,failures;}smon;
static struct Thread distribution_task;
static bool beyond(unsigned state){return current()>state;}
static void log_metric(const char*name,fi32 value,bool important){(void)name;(void)value;(void)important;}
static UINT tx_queue_send(int*q,const fc_msg*message,ULONG wait){assert(q==&seds_syscall&&wait==TX_NO_WAIT&&!queued);queued=true;queued_message=*message;return TX_SUCCESS;}
static SedsResult pulse_ground(void){return SEDS_OK;}
static SedsResult implicit_postinit(const uint8_t*p,size_t n){(void)p;(void)n;return SEDS_HANDLER_ERROR;}
static SedsResult process_gps_packet(const uint8_t*p,size_t n){(void)p;(void)n;return SEDS_HANDLER_ERROR;}
static SedsResult update_ascent_biases(const uint8_t*p,size_t n){(void)p;(void)n;return SEDS_HANDLER_ERROR;}
#include "ground_heartbeat_owner.h"
'''
  support+='\nstatic struct { SedsRouter *r; } g_router;\n'+function((ROOT/'Core/Src/telemetry.c').read_text()[(ROOT/'Core/Src/telemetry.c').read_text().rindex('static SedsResult Manual_Abort_handler('):],'Manual_Abort_handler')+'\n'
  actual=support+'\n'+decoder+'\n'+function(recovery,'fc_recovery_command_admissible')+'\n'+'static SedsResult\n'+function(distribution,'dispatch_flight_cmd')+'\n'+function(distribution,'on_fc_packet_impl')+'\n'+function(recovery,'enter_launch')+r'''
static void process_action(fc_msg message,bool internal){assert(message==Launch_Signal);owner_deliveries++;enter_launch(internal);}
static void process_sensor_report(fc_msg message){(void)message;assert(false);}
static void process_gps_code(fc_msg message){(void)message;assert(false);}
static void process_config_update(fc_msg message){(void)message;assert(false);}
'''+function(recovery,'decode_flight_message')+r'''
static void run_queued_owner(void){assert(queued&&queued_message==Launch_Signal);queued=false;decode_flight_message(queued_message);}
static void controller_probe(SedsRouter*r,const char*kind){
 (void)r;uint8_t command=1;SedsPacketView packet={.ty=SEDS_DT_FLIGHT_COMMAND,.sender="RRLS",.sender_len=4,.payload=&command,.payload_len=1,.timestamp=10003};clock_ms=10003;unsigned reads=media_reads,writes=media_writes;uint32_t epoch=fc_recovery_epoch();assert(epoch==2&&!fc_recovery_abort_latched()&&persistent_store_controls_ready());
 const char*bad[]={"PB","@addr:9999"};for(unsigned i=0;i<2;i++){packet.sender=bad[i];packet.sender_len=strlen(bad[i]);assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued);}
 packet.sender="RRLS";packet.sender_len=4;const uint64_t stale[]={0,UINT64_MAX,10001,4999,11004};for(unsigned i=0;i<5;i++){packet.timestamp=stale[i];assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued);}
 /* Without a network clock, freshness falls back to the station clock (the latest GS QUERY time), which still bounds future-dated commands. */
 packet.timestamp=11004;controller_clock_ok=false;assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued);controller_clock_ok=true;packet.timestamp=10003;
 uint8_t aborted=15;packet.ty=SEDS_DT_FLIGHT_STATE;packet.payload=&aborted;packet.sender="FC";packet.sender_len=2;/* a non-station FSTATE report is ignored (delivery not failed), never applied */assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!fc_recovery_abort_latched()&&!queued);packet.ty=SEDS_DT_FLIGHT_COMMAND;packet.payload=&command;packet.sender="RRLS";packet.sender_len=4;
 if(!strcmp(kind,"held")){fc_recovery_manual_stop();assert(fc_recovery_abort_latched());assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued&&owner_deliveries==0&&media_reads==reads&&media_writes==writes&&!highs);puts("actual current manual Stop blocks RRLS ordinary launch without NVM/high PASS");return;}
 if(!strcmp(kind,"pending")){uint8_t body[32]={1,0};put64(body+4,101);put64(body+24,88);uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;SedsPacketView q={.ty=200,.sender="GS",.sender_len=2,.payload=body,.payload_len=32,.timestamp=10003,.endpoints=&endpoint,.num_endpoints=1};request(r,&q,false);body[1]=1;put64(body+4,102);put64(body+12,board_recovery_boot_current());body[20]=(uint8_t)epoch;q.timestamp=clock_ms=10004;unsigned replies=app_replies;assert(board_recovery_handle(r,&q,endpoint,123)==SEDS_OK&&app_replies==replies&&!persistent_store_controls_ready());packet.timestamp=10004;assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued&&owner_deliveries==0&&!highs);puts("actual durable-recovery Pending gate blocks RRLS ordinary launch PASS");return;}
 if(!strcmp(kind,"GS")){packet.sender="GS";packet.sender_len=2;}
 assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&queued&&g_network_flight_commands_accepted==1);run_queued_owner();assert(owner_deliveries==1&&fc_recovery_epoch()==epoch&&fc_recovery_requires_new_flight());
 sm.flight=Postinit;packet.timestamp=clock_ms=10004;assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK);run_queued_owner();assert(owner_deliveries==2&&fc_recovery_epoch()==epoch&&fc_recovery_requires_new_flight());
 arm_ready=true;packet.timestamp=clock_ms=10005;assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK);run_queued_owner();assert(owner_deliveries==3&&fc_recovery_epoch()==epoch+1&&!fc_recovery_requires_new_flight()&&!fc_recovery_abort_latched()&&arms==1&&!highs&&media_reads==reads&&media_writes==writes);

 sm.flight=Launch;
 persistent_store_note_failure();
 assert(!persistent_store_controls_ready());
 assert(fc_recovery_flight_is_admitted()&&!fc_recovery_holds_sequence()&&!fc_recovery_inhibits_pyro());
 deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(highs==2&&gpio.ODR==3);
 /* Actual distribution callback keeps ordinary controller mutation closed. */
 assert(on_fc_packet_impl(&packet,NULL)==SEDS_OK&&!queued);
 /* Actual manual-ABORT transport remains separate from storage/controller gates. */
 static const uint8_t marker[]="SEDS_MANUAL_ABORT_V1:K";
 SedsPacketView stop={.ty=SEDS_DT_ABORT,.sender="PB",.sender_len=2,.payload=marker,.payload_len=sizeof(marker)-1U,.timestamp=10006};
 g_router.r=r;
 assert(Manual_Abort_handler(&stop,NULL)==SEDS_PERMISSION_DENIED&&gpio.ODR==3);
 stop.sender="GS";stop.sender_len=2;
 assert(Manual_Abort_handler(&stop,NULL)==SEDS_OK);
 assert(fc_recovery_abort_latched()&&!fc_recovery_flight_is_admitted()&&fc_recovery_inhibits_pyro());
 assert(!gpio.ODR&&!co2_ticks&&!reef_ticks&&seal);
 deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(highs==2);
 puts("actual strict-store runtime fault preserves admitted pyro; ordinary controller denied; marked manual GS Abort LOW PASS");
 return;

 printf("actual %s post-recovery packet->queue->decode->admissibility->launch owner PASS; Postinit/readiness/Stop retained; no pyro HIGH/noNVM\n",kind);
}
'''
  text=text.replace('int main(int argc,char**argv){',actual+'\nint main(int argc,char**argv){',1);anchor='  puts("actual FC normal one-request durable-owner COMPLETE PASS");return 0;';assert anchor in text;text=text.replace(anchor,'  controller_probe(r,argv[1]);'+anchor,1)
  marker_fault=r"""
  if(!strcmp(argv[1],"marker_failure")){
   assert(response[1]==BOARD_RECOVERY_RESULT_IO&&!persistent_store_controls_ready()&&!gpio.ODR&&!highs);
   arm_ready=true;sm.flight=Postinit;
   assert(!fc_recovery_admit_new_flight(false)&&!fc_recovery_admit_new_flight(true)&&!fc_recovery_flight_is_admitted());
   uint8_t launch=1;SedsPacketView command={.ty=SEDS_DT_FLIGHT_COMMAND,.sender="GS",.sender_len=2,.payload=&launch,.payload_len=1,.timestamp=10002};
   assert(on_fc_packet_impl(&command,NULL)==SEDS_OK&&!queued&&!highs);
   deployment_pulse_start(false,false);deployment_pulse_start(true,false);assert(!highs&&!gpio.ODR);
   static const uint8_t marker[]="SEDS_MANUAL_ABORT_V1:K";
   command.ty=SEDS_DT_ABORT;command.payload=marker;command.payload_len=sizeof(marker)-1U;g_router.r=r;
   assert(Manual_Abort_handler(&command,NULL)==SEDS_OK&&fc_recovery_abort_latched()&&!gpio.ODR);
   puts("actual strict commissioning marker write failure prevents first Launch/HIGH; manual Abort accepted PASS");return 0;
  }
"""
  old='  if(!strcmp(argv[1],"io"))fail_key=0x4252464cU;'
  assert old in text;text=text.replace(old,old+'\n  if(!strcmp(argv[1],"marker_failure"))fail_key=COMMISSION_RECORD_KEY;',1)
  old='  request(r,&packet,true);assert(!gpio.ODR&&!highs&&irq==0);'
  assert old in text;text=text.replace(old,old+'\n'+marker_fault,1)
  names=['board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_commissioning.c']
  with tempfile.TemporaryDirectory()as tmp:
   d=Path(tmp);p=d/'owner.c';p.write_text(text);exe=d/'owner';cmd=['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-std=c17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/x)for x in names],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p),'-o',str(exe)];subprocess.run(cmd,check=True)
   for case in ['RRLS','GS','marker_failure']:
    with self.subTest(case=case):subprocess.run([str(exe),case],check=True)

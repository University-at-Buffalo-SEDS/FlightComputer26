"""Recovery freshness without a synced SEDSnet clock (Flight Computer).

Bench evidence (Actuator Board): the network clock was unsynced or anchored on
an uptime announce, so every GS QUERY looked stale and RECOVER always failed.
The FC shares the commissioning transport, so the same failure applied.

The actual recovery transport, commissioning service, strict journal, boot
identity, floor, cache owner and FC recovery owner run over the recovery_wire
fixture; only the SDK clock and the board tick are replaced. The refused-reset
change is not ported here: FC staging already performs the owner Stop.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def launchcore_source():
 try:
  from native_sdk import configured_dependency_source
 except ImportError:
  return Path(os.environ['SDK_FREE_LAUNCHCORE_SOURCE'])
 return configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
CLOCK_OLD='SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}'
CLOCK_NEW=r'''static int net_mode; /* 0 synced (clock_ms), 1 unavailable, 2 valid but uptime-like */
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);
 if(net_mode==1)return SEDS_IO;
 *out=net_mode==2?5006972U:clock_ms;return SEDS_OK;}'''
TICK_OLD='ULONG tx_time_get(void){return 1;}'
TICK_NEW='static ULONG ticks=1;ULONG tx_time_get(void){return ticks;}'
MAIN=r'''
#define T0 UINT64_C(1794183589988)
static uint8_t body[32];static uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
static SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32};
static void set_req(uint8_t op,uint64_t nonce,uint64_t gen,uint32_t epoch,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=op;put64(body+4,nonce);put64(body+12,gen);
 for(unsigned i=0;i<4;i++){body[20+i]=(uint8_t)(epoch>>(8*i));}
 put64(body+24,77);packet.timestamp=time;
}
static uint8_t query(SedsRouter*r,uint64_t nonce,uint64_t time){set_req(0,nonce,0,0,time);request(r,&packet,false);return response[1];}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t gen,uint32_t epoch,uint64_t time,bool service){set_req(1,nonce,gen,epoch,time);request(r,&packet,service);return response[1];}
static void heartbeat(SedsRouter*r,uint64_t time){SedsPacketView p={.sender="GS",.sender_len=2,.timestamp=time};board_recovery_observe_station_time(r,&p);}
static uint32_t live_epoch(void){BoardRecoverySnapshot s={0};assert(board_recovery_snapshot(&s)==0);return s.abort_epoch;}
static bool latched(void){BoardRecoverySnapshot s={0};assert(board_recovery_snapshot(&s)==0);return (s.flags&BOARD_RECOVERY_ABORT_LATCHED)!=0;}
static uint64_t boot(void){
 sm.flight=Recovery;evaluation_task.tx_thread_state=0;fc_recovery_evaluation_running();
 memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);uint64_t prior=41;uint8_t original=1;uint64_t floor[2]={77,100};
 assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);assert(launchcore_persist_set(0x4252464cU,floor,16)==0);
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();if(board_commissioning_pending())fc_recovery_restore_commissioning_hold();flight_state_cache_restore();
 assert(persistent_store_runtime_start()==0);runtime=true;assert(persistent_store_controls_ready()&&generation&&generation!=UINT64_MAX);return generation;
}
int main(int argc,char**argv){
 assert(argc==2);const char*scenario=argv[1];(void)send;(void)response_epoch;(void)quiet;(void)ready;(void)durable;(void)io_error;(void)stop_in_io;(void)commits;(void)finals;(void)latched;
 uint64_t gen=boot();int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);
 if(!strncmp(scenario,"unsynced",8)){
  net_mode=!strcmp(scenario,"unsynced_uptime")?2:1;unsigned writes=media_writes;
  /* No GS time seen yet: nothing is fresh, nothing is written or held. */
  assert(recover(r,2,gen,1,T0,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&!persistent_store_commission_pending());
  /* The station clock advanced by a newer GS heartbeat: a 30 s old request
   * is refused even though the board's own network clock is unusable. */
  assert(query(r,3,T0+1)==0);heartbeat(r,T0+30001);
  assert(recover(r,4,gen,1,T0+2,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&!persistent_store_commission_pending());
  /* The board tick advances the station clock without new traffic. */
  assert(query(r,5,T0+30002)==0);ticks+=16000;
  assert(recover(r,6,gen,1,T0+30003,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  /* An older QUERY never moves the clock back. */
  assert(query(r,7,T0+30004)==0);assert(recover(r,8,gen,1,T0+30005,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  /* Fresh station proof: recovery completes. */
  assert(query(r,11,T0+50000)==0);assert(recover(r,12,gen,1,T0+50001,true)==BOARD_RECOVERY_RESULT_OK);
  assert(persistent_store_controls_ready()&&!persistent_store_commission_pending()&&live_epoch()==2&&!gpio.ODR&&!highs);
  /* The replay floor is active; ordinary commands use the same clock. */
  SedsPacketView command={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.timestamp=T0+50002};
  assert(board_recovery_floor_active()&&!board_recovery_command_is_stale(&command));
  command.timestamp=T0+52000;assert(board_recovery_command_is_stale(&command));
  ticks+=20000;command.timestamp=T0+50002;assert(board_recovery_command_is_stale(&command));
  command.timestamp=T0+60000;assert(!board_recovery_command_is_stale(&command));
  /* An uptime-based network clock is flagged once for a timesync restart; the
   * station clock then restarts from the next GS packet. */
  heartbeat(r,T0+80000);assert(!board_recovery_network_clock_resync_needed(T0+80000+59000));
  assert(board_recovery_network_clock_resync_needed(5006972U));assert(!board_recovery_network_clock_resync_needed(5006972U));
  heartbeat(r,T0+80001);assert(!board_recovery_network_clock_resync_needed(T0+80001));
  puts("recovery and command freshness from the station clock without network time PASS");return 0;
 }
 if(!strcmp(scenario,"query_only")){
  /* No heartbeat reaches this board: the GS QUERY alone sets the station
   * clock, and an uptime-anchored network clock does not refuse RECOVER. */
  net_mode=2;assert(query(r,3,T0)==0);assert(recover(r,4,gen,1,T0+1,true)==BOARD_RECOVERY_RESULT_OK);
  assert(persistent_store_controls_ready()&&!persistent_store_commission_pending()&&live_epoch()==2);
  puts("QUERY alone anchors the station clock for RECOVER PASS");return 0;
 }
 return 2;
}
'''
class UnsyncedRecovery(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='unsynced-recovery-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=launchcore_source()
  fixture=ROOT/'tests/fixtures/recovery_wire'
  rendered=(fixture/'harness.in.c').read_text()
  for name,tag in [('fc_recovery_owner.c','ACTUAL_FC_OWNER'),('deployment_pulse.c','ACTUAL_PULSE')]:
   actual='\n'.join(line for line in (ROOT/'Core/Src'/name).read_text().splitlines()if not line.startswith('#include'))
   rendered=rendered.replace('@'+tag+'@',actual)
  harness=rendered.split('int main(int argc,char**argv){',1)[0]
  assert CLOCK_OLD in harness and TICK_OLD in harness
  (p/'unsynced.c').write_text(harness.replace(CLOCK_OLD,CLOCK_NEW).replace(TICK_OLD,TICK_NEW)+MAIN)
  names=['board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_commissioning.c']
  cls.exe=p/'unsynced'
  subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/n)for n in names],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'unsynced.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def run_case(self,name):
  out=subprocess.run([str(self.exe),name],capture_output=True,text=True,timeout=60)
  self.assertEqual(out.returncode,0,out.stdout+out.stderr)
 def test_recovery_without_network_time_uses_station_clock(self):self.run_case('unsynced')
 def test_recovery_with_uptime_based_network_clock(self):self.run_case('unsynced_uptime')
 def test_query_alone_anchors_station_clock(self):self.run_case('query_only')
if __name__=='__main__':unittest.main()

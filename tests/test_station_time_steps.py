"""Station requests across station clock steps (Flight Computer, SEDSnet 6).

Bench (2026-11-16): the GS network clock stepped about 3 h 49 min back. Every
station request was judged by its raw timestamp against the board's clock and
the persisted recovery floor (the last Reset's station stamp), so FLIGHT_STATE,
Launch and recovery requests were refused for hours while Abort still went
through.

Every station stamp now carries its time-base epoch. The actual recovery
transport, commissioning service, strict journal, boot identity, floor, cache
owner and FC recovery owner run over the recovery_wire fixture; only the SDK's
network-stamp API is modelled (tests/fixtures/recovery_wire/harness.in.c: one
remembered step, a newer epoch first heard newer_age ms ago, no network time).
"""
from pathlib import Path
import os, subprocess, tempfile, unittest

ROOT = Path(__file__).resolve().parents[1]


def launchcore_source():
    try:
        from native_sdk import configured_dependency_source
    except ImportError:
        return Path(os.environ['SDK_FREE_LAUNCHCORE_SOURCE'])
    return configured_dependency_source(ROOT, 'sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')


CLOCK_OLD = 'SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}'
CLOCK_NEW = r'''static bool net_ok=true;
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);
 if(!net_ok)return SEDS_IO;
 *out=clock_ms;return SEDS_OK;}'''
MAIN = r'''
#define T0 UINT64_C(1794183589988)
#define H4 INT64_C(14400000)
static uint8_t body[32];static uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
static SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32};
static void set_req(uint8_t op,uint64_t nonce,uint64_t gen,uint32_t epoch,uint16_t te,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=op;put64(body+4,nonce);put64(body+12,gen);
 for(unsigned i=0;i<4;i++){body[20+i]=(uint8_t)(epoch>>(8*i));}
 put64(body+24,77);packet.timestamp=time;packet.time_epoch=te;
}
static uint8_t query(SedsRouter*r,uint64_t nonce,uint16_t te,uint64_t time){set_req(0,nonce,0,0,te,time);request(r,&packet,false);return response[1];}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t gen,uint32_t epoch,uint16_t te,uint64_t time,bool service){set_req(1,nonce,gen,epoch,te,time);request(r,&packet,service);return response[1];}
static uint32_t live_epoch(void){BoardRecoverySnapshot s={0};assert(board_recovery_snapshot(&s)==0);return s.abort_epoch;}
/* A station FLIGHT_STATE/command as the board admits it while the floor is active. */
static bool admitted(uint16_t te,uint64_t time){
 SedsPacketView c={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.timestamp=time,.time_epoch=te};
 return !board_recovery_command_is_stale(&c);
}
/* Reads the floor record directly (fixture mode: outside the storage owner). */
static uint64_t stored_floor(void){uint8_t w[16];size_t n=16;fixture=true;assert(launchcore_persist_get(0x4252464cU,w,&n)==0&&n==16);fixture=false;return get64(w+8);}
static uint64_t boot(void){
 sm.flight=Recovery;evaluation_task.tx_thread_state=0;fc_recovery_evaluation_running();
 memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);uint64_t prior=41;uint8_t original=1;uint64_t floor[2]={77,100};
 assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);assert(launchcore_persist_set(0x4252464cU,floor,16)==0);
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();if(board_commissioning_pending())fc_recovery_restore_commissioning_hold();flight_state_cache_restore();
 assert(persistent_store_runtime_start()==0);runtime=true;assert(persistent_store_controls_ready()&&generation&&generation!=UINT64_MAX);return generation;
}
/* The first Reset, at T0 on the station clock (epoch 0). The floor record
 * written before SEDSnet 6 ({77,100}) reads as epoch 0. */
static void first_reset(SedsRouter*r,uint64_t gen){
 clock_ms=T0;assert(query(r,11,0,T0)==0);assert(recover(r,12,gen,1,0,T0+1,true)==BOARD_RECOVERY_RESULT_OK);
 assert(persistent_store_controls_ready()&&!persistent_store_commission_pending()&&live_epoch()==2&&!gpio.ODR&&!highs);
 assert(board_recovery_floor_active()&&stored_floor()==network_stamp_key(0,T0+1));
 clock_ms=T0+2000;assert(admitted(0,T0+1990));assert(!admitted(0,T0+1));
}
/* An operator Stop, then a Reset with a stamp in epoch te; returns the result. */
static uint8_t reset_again(SedsRouter*r,uint64_t gen,uint64_t nonce,uint16_t te,uint64_t time){
 fc_recovery_manual_stop();const uint32_t e=live_epoch();
 if(query(r,nonce,te,time)!=0)return 255;
 return recover(r,nonce+1,gen,e,te,time+1,true);
}
static void step(int64_t delta){prev_epoch=clock_epoch;clock_epoch++;step_delta=delta;clock_ms=(uint64_t)((int64_t)clock_ms+delta);}
int main(int argc,char**argv){
 assert(argc==2);const char*scenario=argv[1];(void)send;(void)response_epoch;(void)quiet;(void)ready;(void)durable;(void)io_error;(void)stop_in_io;(void)commits;(void)finals;
 uint64_t gen=boot();int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);
 if(!strcmp(scenario,"step_back")||!strcmp(scenario,"step_forward")){
  const bool back=!strcmp(scenario,"step_back");const int64_t delta=back?-H4:INT64_C(13680000); /* 4 h back or 3.8 h forward */
  first_reset(r,gen);const uint64_t before=clock_ms;
  step(delta);const uint64_t now=clock_ms;
  /* The station's next request, on the new clock, is admitted at once. */
  assert(admitted(1,now));assert(admitted(1,now-14000));assert(!admitted(1,now-16000));assert(!admitted(1,now+2000));
  /* A late copy stamped before the step: judged by its age across the step. */
  assert(admitted(0,before-1000));assert(!admitted(0,before-20000));
  /* A replay stamped before the Reset stays below the floor. */
  assert(!admitted(0,T0+1));assert(!admitted(0,T0));
  /* A stamp from an epoch this board cannot translate is stale. */
  clock_epoch=2;prev_epoch=1;assert(!admitted(0,before-1000));clock_epoch=1;prev_epoch=0;
  /* Reset after the step: newer than the floor, whatever its raw stamp. */
  assert(reset_again(r,gen,21,1,now+10)==BOARD_RECOVERY_RESULT_OK);
  assert(live_epoch()==4&&!gpio.ODR&&!highs&&stored_floor()==network_stamp_key(1,now+11));
  clock_ms=now+20;
  /* Every request stamped before that Reset, in either epoch, is refused. */
  assert(!admitted(1,now+5));assert(!admitted(0,before-1000));assert(admitted(1,now+15));
  /* A replay of the pre-step Reset request, or a new one stamped before the
   * floor, changes nothing. */
  unsigned writes=media_writes;
  assert(recover(r,31,gen,live_epoch(),1,now+5,true)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  assert(recover(r,12,gen,1,0,T0+1,true)!=BOARD_RECOVERY_RESULT_OK&&media_writes==writes&&live_epoch()==4);
  printf("%s: requests admitted across the step, Reset accepted, replays refused PASS\n",scenario);return 0;
 }
 if(!strcmp(scenario,"reanchor")){
  /* The GS stepped back (epoch 1); this board has heard of epoch 1 but not
   * re-anchored yet. A station request in the newer epoch is fresh while the
   * board has known of it for at most the 15 s window. */
  first_reset(r,gen);const uint64_t gs_now=clock_ms-(uint64_t)H4;
  newer_age=2000;assert(admitted(1,gs_now));
  assert(reset_again(r,gen,41,1,gs_now)==BOARD_RECOVERY_RESULT_OK&&live_epoch()==4);
  newer_age=16000;assert(!admitted(1,gs_now+50));
  unsigned writes=media_writes;fc_recovery_manual_stop();
  assert(recover(r,51,gen,live_epoch(),1,gs_now+60,true)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  puts("newer-epoch request admitted while re-anchoring, refused once overdue PASS");return 0;
 }
 if(!strcmp(scenario,"no_time")){
  first_reset(r,gen);net_ok=false;
  /* No network time: no station request is fresh while the floor is active,
   * and a Reset neither writes nor latches anything. */
  assert(!admitted(0,T0+2000));unsigned writes=media_writes;fc_recovery_manual_stop();
  assert(query(r,61,0,T0+2001)==0);assert(recover(r,62,gen,live_epoch(),0,T0+2002,true)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  net_ok=true;clock_ms=T0+2003;assert(admitted(0,T0+2003));
  puts("no network time: station requests refused PASS");return 0;
 }
 return 2;
}
'''


class StationTimeSteps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='station-time-steps-')
        p = Path(cls.tmp.name)
        sdk = Path(os.environ['SEDSNET_TEST_SOURCE'])
        lc = launchcore_source()
        fixture = ROOT / 'tests/fixtures/recovery_wire'
        rendered = (fixture / 'harness.in.c').read_text()
        for name, tag in [('fc_recovery_owner.c', 'ACTUAL_FC_OWNER'), ('deployment_pulse.c', 'ACTUAL_PULSE')]:
            actual = '\n'.join(line for line in (ROOT / 'Core/Src' / name).read_text().splitlines()
                               if not line.startswith('#include'))
            rendered = rendered.replace('@' + tag + '@', actual)
        harness = rendered.split('int main(int argc,char**argv){', 1)[0]
        assert CLOCK_OLD in harness
        (p / 'steps.c').write_text(harness.replace(CLOCK_OLD, CLOCK_NEW) + MAIN)
        names = ['board_recovery_wire.c', 'board_recovery_boot.c', 'board_recovery_transport.c',
                 'flight_state_cache.c', 'persistent_store.c', 'board_commissioning.c']
        cls.exe = p / 'steps'
        subprocess.run(['cc', '-DEXPECTED_CAP=32', '-DTELEMETRY_ENABLED=1', '-std=c11', '-Wall', '-Wextra',
                        '-Werror', '-Wno-unused-function', '-fsanitize=address,undefined',
                        '-I' + str(fixture), '-I' + str(ROOT / 'Core/Inc'), '-I' + str(ROOT / 'Core/Src'),
                        '-I' + str(sdk / 'C-Headers'), '-I' + str(lc / 'bootloader/include'),
                        *[str(ROOT / 'Core/Src' / n) for n in names], str(ROOT / 'Bootloader/storage_dispatch.c'),
                        str(lc / 'bootloader/src/persist.c'), str(lc / 'bootloader/src/crc32.c'),
                        str(p / 'steps.c'), '-o', str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, name):
        out = subprocess.run([str(self.exe), name], capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
        self.assertIn('PASS', out.stdout)

    def test_backward_4_h_step(self):
        self.run_case('step_back')

    def test_forward_3_8_h_step(self):
        self.run_case('step_forward')

    def test_newer_epoch_request_while_reanchoring(self):
        self.run_case('reanchor')

    def test_no_network_time_refuses_station_requests(self):
        self.run_case('no_time')


if __name__ == '__main__':
    unittest.main()

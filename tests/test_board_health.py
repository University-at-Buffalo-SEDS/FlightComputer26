from pathlib import Path
import json,re,subprocess,tempfile,unittest
from checked_router_fixture import checked_hooks
ROOT=Path(__file__).resolve().parents[1]
class BoardHealthTests(unittest.TestCase):
 def test_actual_producer_is_bounded_exact_and_optional(self):
  source=(ROOT/'Core/Src/board_health.c').read_text()
  source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
  harness=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
typedef int SedsRouter;
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_DT_BOARD_HEALTH 42
#define SEDS_EK_UNSIGNED 0
#define BOARD_UPTIME_FREQUENCY_HZ 1000U
#define SD_AVAILABLE
uint64_t uptime;
unsigned calls,lock_depth,mask,refuse;
uint32_t captured[24];
volatile uint32_t g_watchdog_reset_flags=123,g_watchdog_missing_mask=5;
volatile uint32_t g_telemetry_tlsf_live_bytes=100,g_telemetry_tlsf_peak_bytes=200,g_telemetry_tlsf_region_bytes=4096,g_telemetry_tlsf_failures=7;
volatile uint32_t g_can_tx_rejected=11,g_can_tx_high_water=12,g_gateway_uart_rx_ring_drops=13;
volatile uint32_t g_sd_raw_records_enqueued_count=0xf00d1234U,g_sd_raw_records_written_count=15,g_sd_raw_records_dropped_count=16,g_sd_raw_records_flushed_count=17,g_sd_raw_records_io_lost_count=18,g_sd_write_error_count=19,g_sd_max_write_ms=20;
volatile uint32_t g_sd_enqueued_records=0xf00d1234U,g_sd_written_records=15,g_sd_flushed_records=17,g_sd_format_errors=16,g_sd_lock_contention_drops=18,g_sd_unavailable_drops=19,g_sd_shutdown_lost_records=20,g_sd_write_errors=21;
volatile uint8_t g_fc_sd_state;
volatile uint32_t g_av_bay_underglow_enabled=1,g_flight_buzzer_enabled=0,g_av_bay_underglow_network_seen=1,g_flight_buzzer_network_seen=0;
void board_temperature_poll(void){}
uint32_t board_temperature_health_flags(void){return 0x17bd0100U;}
uint64_t board_uptime_ticks(void){return uptime;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t value){mask=value;}
uint32_t board_health_can_rx_drops(void){return 31;}
uint32_t board_health_uart_rx_drops(void){return 32;}
uint32_t board_health_adc_overruns(void){return 33;}
size_t seds_memory_admission_rejected(void){return 34;}
uint32_t gate;uint32_t board_recovery_controller_gate(void){assert(!mask);return gate;}
void telemetry_lock(void){assert(!mask);++lock_depth;}
void telemetry_unlock(void){assert(lock_depth==1);--lock_depth;}
typedef struct{uint8_t stratum;uint8_t health_byte;}SedsTimeSyncStatus;
uint8_t ts_health,ts_stratum;
SedsResult seds_router_get_timesync_status(SedsRouter *router,SedsTimeSyncStatus *out){assert(router&&out&&lock_depth==1);out->health_byte=ts_health;out->stratum=ts_stratum;return SEDS_OK;}
SedsResult seds_router_log_bytes_ex(SedsRouter *router,int dtype,const uint8_t *data,size_t length,const uint64_t *stamp,bool queue){
 assert(router&&dtype==42&&length==96&&!stamp&&queue);
 assert(!mask&&lock_depth==1);++calls;memcpy(captured,data,sizeof(captured));return refuse?1:0;
}
''' +checked_hooks(ROOT)+'\n'+(ROOT/'Core/Inc/fc_estimator_status.h').read_text()+r'''
volatile uint8_t g_fc_est_phase;volatile uint32_t g_fc_est_cycles,g_fc_est_start;
uint32_t baro_bits;uint32_t baro_ref_bits(void){return baro_bits;}
'''+source+r'''
int main(void){
 SedsRouter router=1;
 board_health_poll(NULL);assert(!calls);
#ifdef TELEMETRY_ENABLED
 uptime=100;board_health_poll(&router);assert(!calls);
 uptime=4999;board_health_poll(&router);assert(!calls);
 uptime=5000;fixture_get_failure=1;board_health_poll(&router);assert(!calls&&!lock_depth&&lock_fails==1);fixture_get_failure=0;uptime=10000;refuse=1;board_health_poll(&router);assert(calls==1);
 for(unsigned i=0;i<1000;i++)board_health_poll(&router);
 assert(calls==1&&!lock_depth&&!mask); /* no pressure retry storm */
 uptime=15000;refuse=0;board_health_poll(&router);
 assert((captured[1]&0xffffff00U)==0x17bd0100U);
 assert(calls==2&&captured[23]==2&&captured[0]==1&&captured[4]==1000);
 assert(captured[5]==123&&captured[6]==5&&captured[11]==34&&captured[12]==31);
 /* Applied cosmetic outputs: tag 0x4F55, underglow on from a network write. */
 assert(!(captured[1]&8U)&&captured[14]==((0x4F55U<<16)|1U|4U));
#ifdef TELEMETRY_USE_TLSF
 assert((captured[1]&1)&&captured[7]==100&&captured[8]==200&&captured[9]==4096&&captured[10]==7);
#else
 assert(!(captured[1]&1)&&!captured[7]&&!captured[8]&&!captured[9]&&!captured[10]);
#endif
 if(captured[1]&16){assert(captured[16]==0xf00d1234U);assert((captured[1]&128U)&&captured[21]==20U);} /* no float rounding */
 /* Flight-command refusal reasons ride above the watchdog mask. */
 gate=9;uptime=16000+5000;board_health_poll(&router);assert(calls==3&&captured[6]==(5U|(9U<<24)));gate=0;calls=2;
 uptime=(UINT64_C(1)<<42)+20000;board_health_poll(&router);
 assert(calls==3&&captured[2]==20000&&captured[3]==1024);
 uptime=(UINT64_C(1)<<42)+UINT32_MAX-2U;board_health_poll(&router);assert(calls==4);
 uptime+=4999;board_health_poll(&router);assert(calls==4);
 ++uptime;board_health_poll(&router);assert(calls==5); /* 32-bit schedule wrap */
 g_flight_buzzer_enabled=1;g_flight_buzzer_network_seen=1;g_av_bay_underglow_enabled=0;
 uptime+=5000;board_health_poll(&router);assert(calls==6&&captured[14]==((0x4F55U<<16)|2U|4U|8U));
 /* Network time quality: SEDSnet health byte in word 22, tagged "SY". */
 {unsigned before=calls;ts_health=0x8AU;uptime+=5000;board_health_poll(&router);
  assert(calls==before+1&&captured[22]==((0x5359U<<16)|0x8AU));ts_health=0;}
 /* Stratum (2: RF re-serving the GS over CAN) in word 22 bits 8..15. */
 {unsigned before=calls;ts_health=0x85U;ts_stratum=2;uptime+=5000;board_health_poll(&router);
  assert(calls==before+1&&captured[22]==((0x5359U<<16)|(2U<<8)|0x85U));ts_health=0;ts_stratum=0;}
 /* Kalman filter status in words 13/15: idle until Launch. */
 assert(captured[13]==(0x4BU<<24)&&captured[15]==0U);
 /* Launch: a phase change is sent at once, not after the 5 s cadence. */
 {unsigned before=calls;uint64_t t=uptime+300U;
  g_fc_est_phase=FC_EST_INIT;uptime=t;board_health_poll(&router);
  assert(calls==before+1&&captured[13]==((0x4BU<<24)|(1U<<20)));
  /* Running 5 ms later: held to the 250 ms minimum spacing, then sent. */
  g_fc_est_phase=FC_EST_RUNNING;g_fc_est_cycles=3;g_fc_est_start=(uint32_t)t|1U;
  uptime=t+5U;board_health_poll(&router);assert(calls==before+1);
  uptime=t+250U;board_health_poll(&router);
  assert(calls==before+2&&captured[13]==((0x4BU<<24)|(2U<<20)|3U)&&captured[15]==((((uint32_t)t|1U)>>4)&0x3FFFFU));
  /* Time since start from one report, to 16 ticks, modulo 2^22 ticks. */
  {uint32_t since=(captured[2]-((captured[15]&0x3FFFFU)<<4))&0x3FFFFFU;assert(since>=250U&&since<250U+16U);}
  /* Repeated 1 s after the change, then every 5 s. */
  uptime=t+1249U;board_health_poll(&router);assert(calls==before+2);
  g_fc_est_cycles=0x100005U; /* the cycle field wraps at 20 bits */
  uptime=t+1250U;board_health_poll(&router);assert(calls==before+3&&captured[13]==((0x4BU<<24)|(2U<<20)|5U));
  uptime=t+6249U;board_health_poll(&router);assert(calls==before+3);
  uptime=t+6250U;board_health_poll(&router);assert(calls==before+4);
  g_fc_est_phase=FC_EST_DIVERGED;uptime=t+6500U;board_health_poll(&router);
  assert(calls==before+5&&(captured[13]>>20&0xFU)==6U);
  /* A failed send is not retried until the 1 s repeat. */
  g_fc_est_phase=FC_EST_STOPPED;refuse=1;uptime=t+6800U;board_health_poll(&router);
  for(unsigned i=0;i<100;i++)board_health_poll(&router);
  assert(calls==before+6);refuse=0;
  uptime=t+7800U;board_health_poll(&router);assert(calls==before+7&&(captured[13]>>20&0xFU)==5U);
  assert(!(captured[1]&(4U|64U))); /* words 13/15 are not the generic counters */
  g_fc_est_phase=0;g_fc_est_cycles=0;g_fc_est_start=0;}
 /* Word 15 bits 18..31: the barometer ground reference (mode, shift in Pa). */
 {unsigned before=calls;baro_bits=((0xFF6U)<<2)|1U; /* tracking, -10 Pa */
  uptime+=5000;board_health_poll(&router);
  assert(calls==before+1&&captured[15]==(baro_bits<<18)&&captured[13]==(0x4BU<<24));
  /* Freezing at Launch is sent at once, and so is thawing for a new flight. */
  baro_bits=(0xFF6U<<2)|3U;uptime+=300;board_health_poll(&router);
  assert(calls==before+2&&(captured[15]>>18)==baro_bits);
  baro_bits=(0xFF6U<<2)|2U;uptime+=300;board_health_poll(&router);assert(calls==before+3);
  /* Tracking <-> paused on the pad waits for the cadence. */
  baro_bits=(0xFF6U<<2)|1U;uptime+=300;board_health_poll(&router);
  uptime+=300;board_health_poll(&router);assert(calls==before+3);
  /* A start stamp in the filter's field keeps the reference bits. */
  g_fc_est_start=0x12345671U;uptime+=5000;board_health_poll(&router);
  assert(calls==before+4&&captured[15]==((baro_bits<<18)|(0x1234567U&0x3FFFFU)));
  g_fc_est_start=0;baro_bits=0;}
#else
 uptime=999999;board_health_poll(&router);assert(!calls);
#endif
 return 0;
}
'''
  with tempfile.TemporaryDirectory() as tmp:
   for label,flags in [('tlsf',['-DTELEMETRY_ENABLED','-DTELEMETRY_USE_TLSF']),('threadx',['-DTELEMETRY_ENABLED']),('disabled',[])]:
    binary=Path(tmp)/label
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',*flags,'-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
    subprocess.run([str(binary)],check=True)
 def test_schema_and_header_use_matching_canonical_u32_identity(self):
  definitions=json.loads((ROOT/'config/sedsnet.json').read_text())
  report=next(t for t in definitions['types'] if t['name']=='BOARD_HEALTH')
  self.assertEqual(report['element'],dict(kind='Static',data_type='UInt32',count=24))
  self.assertEqual((report['class'],report['reliable'],report['priority']),('Data','Off',0))
  hashed=0x811c9dc5 ^ 0x5459
  for byte in b'BOARD_HEALTH':hashed=((hashed ^ byte)*0x01000193)&0xffffffff
  expected=0x10000 | (hashed & 0x0fffffff)
  header=(ROOT/'Core/Inc/sedsnet_config.h').read_text()
  actual=int(re.search(r'#define SEDS_DT_BOARD_HEALTH \(\(SedsDataType\)(\d+)U\)',header)[1])
  self.assertEqual(actual,expected)

from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class WatchdogRecoveryTests(unittest.TestCase):
    def test_faults_and_resume_each_flight_phase(self):
        source = (ROOT/'Core/Src/fc_watchdog_recovery.c').read_text()
        source = source.replace('g_pfnVectors, (uint32_t)((uintptr_t)__data_source_end - (uintptr_t)g_pfnVectors)', 'test_image, sizeof(test_image)')
        source = source.replace('section(".fc_watchdog_retained"), ', '')
        source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
        stub = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define BOARD_WATCHDOG_ENABLE 1
#define RECORD_TEST 1
#define STATE_HISTORY 8
#define Time_Users 6
#define Global_States 16
#define Armed 2
#define Descent 7
#define Reefing 8
#define Recovery 10
#define Launch_Requested 1U
#define CO2_Asserted 2U
#define REEF_Asserted 4U
#define Ascent_KF_Staged 8U
#define Using_Ascent_KF 16U
#define GPS_Available 128U
#define Parachute_Deployed 32U
#define Parachute_Expanded 64U
#define option(x) (x)
#define current() (sm.flight)
#define RCC_RSR_IWDGRSTF 1U
#define RCC_RSR_BORRSTF 2U
#define PWR_DBPCR_DBP 1U
#define RCC_APB3ENR_RTCAPBEN 1U
#define RCC_BDCR_LSION 1U
#define RCC_BDCR_LSIRDY 2U
#define RCC_BDCR_RTCSEL 12U
#define RCC_BDCR_RTCSEL_1 8U
#define RCC_BDCR_RTCEN 16U
#define RTC_ICSR_BIN 3U
#define RTC_ICSR_BIN_0 1U
#define RTC_ICSR_INIT 4U
#define RTC_ICSR_INITF 8U
#define RTC_PRER_PREDIV_A_Pos 16
#define RTC_CR_BYPSHAD 1U
static struct {uint32_t DBPCR;} pwr;
static struct {uint32_t APB3ENR,BDCR;} rcc;
static struct {uint32_t ICSR,PRER,WPR,CR,SSR;} rtc;
#define PWR (&pwr)
#define RCC (&rcc)
#define RTC (&rtc)
static uint8_t test_image[]={1,2,3,4,5};
static uint32_t mask, now, frozen, bindings;
static void __NOP(void) {if (!frozen) rtc.SSR--; rtc.ICSR |= RTC_ICSR_INITF;}
static void __DMB(void) {}
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __set_PRIMASK(uint32_t v) {mask=v;}
static uint32_t now_ms(void) {return now;}
static uint32_t g_conf,local_time[Time_Users];
static struct {uint32_t flight,idx,global_state,confidence;} sm;
static float states[8], covariance[6], baro[3], statistics[2];
typedef struct {void *data; uint32_t size;} fc_resume_region;
#define R(x) {(void*)&(x),sizeof(x)}
const fc_resume_region fc_resume_evaluation[]={R(sm),R(states),R(statistics),{NULL,0}};
const fc_resume_region fc_resume_kalman[]={R(covariance),{NULL,0}};
const fc_resume_region fc_resume_recovery[]={R(g_conf),R(local_time),{NULL,0}};
const fc_resume_region fc_resume_barometer[]={{baro,sizeof(baro)},{NULL,0}};
static void kalman_resume_bindings(bool ascent) {bindings=ascent?1:2;}
'''
        main = r'''
static void reset_runtime(void) {
 mask=0;now=25;g_conf=0;memset(&sm,0,sizeof(sm));
 memset(states,0,sizeof(states));memset(covariance,0,sizeof(covariance));
 memset(baro,0,sizeof(baro));memset(local_time,0,sizeof(local_time));
 image_id=actions=next_slot=sequence=action_slot=action_sequence=0;
 busy=clock_ready=false;g_fc_watchdog_resumed=g_fc_watchdog_resume_rejected=0;
}
static void cold_boot(void) {
 reset_runtime(); frozen=0; rcc.BDCR=RCC_BDCR_LSIRDY|RCC_BDCR_RTCSEL_1;
 rtc.ICSR=RTC_ICSR_BIN_0;rtc.PRER=31U<<16;rtc.SSR=500000;
 fc_watchdog_recovery_boot(RCC_RSR_BORRSTF);
 assert(clock_ready && !g_fc_watchdog_resumed && fc_watchdog_can_actuate());
}
static void flight(unsigned phase) {
 g_conf=Launch_Requested | GPS_Available | (phase<Descent?Using_Ascent_KF:0);
 sm.flight=phase;sm.idx=5;sm.global_state=9;sm.confidence=77;
 baro[0]=101000;baro[1]=123;baro[2]=120;
 states[0]=321.5f;covariance[0]=0.125f;now=1000;local_time[0]=200;
 fc_watchdog_checkpoint();assert(!mask && g_fc_watchdog_checkpoints);
}
int main(void) {
 for(unsigned phase=Armed;phase<=Recovery;phase++) {
  cold_boot();flight(phase);rtc.SSR-=16000;
  reset_runtime();fc_watchdog_recovery_boot(RCC_RSR_IWDGRSTF);
  assert(g_fc_watchdog_resumed && !g_fc_watchdog_resume_rejected);
  assert(sm.flight==phase && sm.idx==5 && sm.global_state==9 && sm.confidence==0);
  assert(!(g_conf & GPS_Available));
  assert(states[0]==321.5f && covariance[0]==0.125f && baro[0]==101000);
  assert((uint32_t)(now-local_time[0])==16801); /* RTC init advances one tick */
  assert(bindings==(phase<Descent?1:2));
  assert(!(g_conf & (CO2_Asserted|REEF_Asserted|Ascent_KF_Staged)));
 }
 /* Interrupted new checkpoint cannot displace the last committed record. */
 cold_boot();flight(3);flight(4);retained.flight[1].seal=0;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(sm.flight==3 && g_fc_watchdog_resumed);
 /* Deployment journal is newer than an estimator record: no repeat pulse. */
 cold_boot();flight(4);fc_watchdog_record_deployment(false);
 reset_runtime();fc_watchdog_recovery_boot(1);
 assert(sm.flight==Descent && (g_conf & Parachute_Deployed) && !(g_conf & CO2_Asserted));
 fc_watchdog_record_deployment(true);reset_runtime();fc_watchdog_recovery_boot(1);
 assert(sm.flight==Reefing && (g_conf & Parachute_Expanded));
 /* Corrupt journal must not fall back to pre-deployment history. */
 cold_boot();flight(4);fc_watchdog_record_deployment(false);
 retained.action[action_slot^1U].crc^=1;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 /* Wrong firmware / corrupt payload / missing record / stale record all inhibit. */
 cold_boot();flight(4);test_image[0]^=1;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 cold_boot();flight(4);retained.flight[0].payload[0]^=1;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 cold_boot();reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 cold_boot();flight(4);rtc.SSR-=60001;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 /* Even a valid CRC cannot legitimize an invalid phase or NaN estimator. */
 cold_boot();flight(20);reset_runtime();fc_watchdog_recovery_boot(1);
 assert(!fc_watchdog_can_actuate() && sm.flight==0);
 cold_boot();flight(4);uint32_t nan=0x7fc00000;memcpy(states,&nan,4);
 fc_watchdog_checkpoint();retained.flight[0].seal=0;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(!fc_watchdog_can_actuate());
 /* A BOR/power reset cannot resume stale in-flight state. */
 cold_boot();flight(4);reset_runtime();fc_watchdog_recovery_boot(1|2);
 assert(!g_fc_watchdog_resumed && sm.flight==0);
 /* Counter wrap keeps elapsed timer age; bounded clock failure inhibits. */
 cold_boot();rtc.SSR=10;flight(4);rtc.SSR=UINT32_MAX-19;
 reset_runtime();fc_watchdog_recovery_boot(1);assert(g_fc_watchdog_resumed && g_fc_watchdog_resume_gap_ms==31);
 cold_boot();frozen=1;reset_runtime();fc_watchdog_recovery_boot(1);
 assert(!fc_watchdog_can_actuate());
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory)/'recovery')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],input=stub+source+main,text=True,check=True)
            subprocess.run([exe],check=True,timeout=10)

    def test_boot_and_resume_do_not_replay_startup_or_ignition(self):
        source=(ROOT/'Core/Src/distribution.c').read_text()
        entry=source[source.index('void distribution_entry'):]
        self.assertIn('if (!g_fc_watchdog_resumed && fc_watchdog_can_actuate())',entry)
        recovery=(ROOT/'Core/Src/recovery.c').read_text()
        self.assertIn('baro_conf.rezero = 0; /* retain the launch pressure',recovery)
        pulse=(ROOT/'Core/Src/deployment_pulse.c').read_text()
        self.assertLess(pulse.index('fc_watchdog_record_deployment'),pulse.index('reef_high()'))
        self.assertLess(pulse.index('fc_watchdog_record_deployment'),pulse.index('co2_high()'))
        for name in ['STM32H523xx_FLASH.ld','STM32H523xx_RAM.ld']:
            text=(ROOT/name).read_text()
            self.assertIn('.fc_watchdog_retained (NOLOAD)',text)
            self.assertGreater(text.index('.fc_watchdog_retained'),text.index('_ebss ='))
        self.assertNotIn('board_watchdog_progress',pulse)
    def test_fsm_waits_for_an_entire_fresh_history_after_reset(self):
        source=(ROOT/'Core/Src/evaluation.c').read_text()
        function=source[source.index('void evaluate_rocket_state'):source.index('static inline void enter_flight_mode')]
        stub=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define BOARD_WATCHDOG_ENABLE 1
#define NDEBUG 1
#define STATE_HISTORY 8
#define Vigilant_Mode 1
#define BOARD_WATCHDOG_SAFETY 4
#define option(x) (x)
typedef uint32_t fu32;
typedef enum {Armed,Launch,Ascent,Coast,Apogee,Descent,Reefing,Landed,Recovery} state;
static unsigned g_fc_watchdog_resumed=1,decisions,published,checkpoints,progress;
static state current(void) {return Armed;}
#define DECISION(name) static void name(fu32 c) {(void)c;decisions++;}
DECISION(detect_boost) DECISION(detect_ascent) DECISION(detect_coast)
DECISION(detect_apogee) DECISION(detect_descent) DECISION(detect_reefing)
DECISION(detect_landed) DECISION(announce_recovery) DECISION(report_lowpass_gps)
static void log_metric(const char *s,int v,int b) {(void)s;(void)v;(void)b;}
static void vigilant_watchdog(fu32 c,state s,float dt) {(void)c;(void)s;(void)dt;}
static void propel_kalman_state(fu32 c) {(void)c;published++;}
static void fc_watchdog_checkpoint(void) {checkpoints++;}
static void board_watchdog_progress(unsigned m) {assert(m==4);progress++;}
#define id "test"
'''
        main=r'''
int main(void) {
 for(unsigned i=0;i<STATE_HISTORY;i++) {
  evaluate_rocket_state(0,0.04f);
  assert(decisions==0 && published==i+1 && progress==i+1);
 }
 evaluate_rocket_state(0,0.04f);
 assert(decisions==1 && published==9 && checkpoints==9);
 g_fc_watchdog_resumed=0;evaluate_rocket_state(0,0.04f);
 assert(decisions==2); /* a normal startup has no recovery gate */
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe=str(Path(directory)/'fresh-history')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-x','c','-','-o',exe],input=stub+function+main,text=True,check=True)
            subprocess.run([exe],check=True,timeout=5)

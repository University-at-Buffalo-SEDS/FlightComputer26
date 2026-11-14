"""A fresh Abort latch on the FC pushes one BOARD_STATE_STATUS to the station.

The station learns the FC's Abort latch from its ordered recovery QUERY,
which can wait behind a degraded radio or relay. The first edge of a latch
episode (fc_recovery_abort_latched() or In_Aborted_State) now owes exactly
one unsolicited status: nonce 0, ABORT_LATCHED, the live abort epoch, the
QUERY reply's generation and the last valid QUERY's issuer. A latch present
at the first pass (restored at boot), one set by recovery commissioning and
repeated passes owe none; a refused push is retried while the latch holds.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


def _build_and_run(code: str) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp)
        (p / 'harness.c').write_text(code)
        exe = p / 'test'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                        '-I', str(ROOT / 'Core/Inc'), str(p / 'harness.c'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


EDGE_HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult; typedef int SedsRouter;
#define SEDS_OK 0
#define SEDS_ERR -1
#define TELEMETRY_ENABLED 1
#define In_Aborted_State (1u << 17)
#define option(o) (o)
static uint32_t g_conf;
static SedsRouter router=1; static struct{SedsRouter*r;}g_router={&router};
static bool latch,commissioning;static int entered,pushes,refuse;
static bool fc_recovery_abort_latched(void){return latch;}
static bool board_commissioning_pending(void){return commissioning;}
static int telemetry_router_enter(void){++entered;return 1;}
static int telemetry_router_leave(void){--entered;return 1;}
static SedsResult board_recovery_push_abort_status(SedsRouter*r){assert(r&&entered==1);++pushes;return refuse?SEDS_ERR:SEDS_OK;}
/* SDK pending rows and the tick, as the push sees them: status_pending and
 * status_retries model the one BOARD_STATE_STATUS awaiting its receipt. */
#define SEDS_DT_BOARD_STATE_STATUS 77
#define TELEMETRY_RELIABLE_MAX_RETRIES 3U
#define ABORT_PUSH_WINDOW_MS 5000U
#define ABORT_PUSH_UNSEEN_MS 1000U
typedef struct {uint64_t packet_id,last_send_ms;uint32_t data_type,pending_count,retries,flags;uint64_t pending_owners[4];} SedsReliablePendingRow;
static bool status_pending;static uint32_t status_retries,now_ms;
static uint32_t HAL_GetTick(void){return now_ms;}
static int32_t seds_router_copy_reliable_pending(SedsRouter*r,size_t first,SedsReliablePendingRow*rows,size_t cap){
 assert(r&&entered==1&&cap>=1);if(!status_pending)return 0;if(first==0){rows[0]=(SedsReliablePendingRow){0};rows[0].data_type=SEDS_DT_BOARD_STATE_STATUS;rows[0].retries=status_retries;}return 1;}
@PUSH@
int main(void){
 /* A latch already present at the first pass (restored at boot) owes none. */
 latch=true;telemetry_push_abort_status();telemetry_push_abort_status();assert(pushes==0);
 /* A fresh 0->1 latch owes exactly one push. */
 latch=false;telemetry_push_abort_status();assert(pushes==0);
 latch=true;telemetry_push_abort_status();assert(pushes==1);
 telemetry_push_abort_status();telemetry_push_abort_status();assert(pushes==1);
 /* In_Aborted_State alone opens an episode too; the other source joining
  * inside the same episode owes nothing more. */
 latch=false;telemetry_push_abort_status();
 g_conf=In_Aborted_State;telemetry_push_abort_status();assert(pushes==2);
 latch=true;telemetry_push_abort_status();assert(pushes==2);
 /* A latch set by recovery commissioning owes none. */
 latch=false;g_conf=0;telemetry_push_abort_status();
 commissioning=true;latch=true;telemetry_push_abort_status();assert(pushes==2);
 commissioning=false;
 /* A refused push is retried on the next pass while the latch holds. */
 latch=false;telemetry_push_abort_status();
 refuse=1;latch=true;telemetry_push_abort_status();assert(pushes==3&&entered==0);
 refuse=0;telemetry_push_abort_status();assert(pushes==4);
 telemetry_push_abort_status();assert(pushes==4);
 /* A refused push is dropped if the latch clears first. */
 latch=false;telemetry_push_abort_status();refuse=1;latch=true;telemetry_push_abort_status();
 refuse=0;latch=false;telemetry_push_abort_status();assert(pushes==5);
 return 0;
}
'''

LOST_CASES = r'''
/* Pass the telemetry loop for ms milliseconds, 10 ms per pass. */
static void run(uint32_t ms){for(uint32_t t=0;t<ms;t+=10){now_ms+=10;telemetry_push_abort_status();}}
/* The station never acknowledges a copy: the SDK retries it 3 times, 400 ms apart, then drops it. */
static void lose_copy(void){status_pending=true;for(status_retries=0;status_retries<=3;++status_retries)run(400);status_pending=false;status_retries=0;}
int main(void){
 now_ms=1000;latch=false;telemetry_push_abort_status();
 /* Edge: one push. Its first 4 copies (the push and 3 retries) are all lost. */
 latch=true;telemetry_push_abort_status();assert(pushes==1);
 const uint32_t edge=now_ms;lose_copy();run(20);
 /* Pushed again while latched, well inside the station's 4.8 s Abort window. */
 assert(pushes==2&&now_ms-edge<2000);
 /* This copy is acknowledged on its first try: no more pushes. */
 status_pending=true;status_retries=0;run(100);status_pending=false;run(6000);
 assert(pushes==2);
 /* A latch whose every copy is lost is pushed until 5 s after its edge, then no more. */
 latch=false;run(20);latch=true;run(10);const int first=pushes;const uint32_t edge2=now_ms;
 while(now_ms-edge2<7000){lose_copy();run(20);}
 assert(pushes>first+1&&pushes<=first+4);
 const int last=pushes;run(3000);assert(pushes==last);
 return 0;
}
'''

PUSH_HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "board_recovery.h"
#include "board_recovery_wire.h"
typedef int SedsResult; typedef int SedsRouter; typedef int SedsDataType; typedef unsigned long ULONG;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_DT_BOARD_STATE_STATUS 77
#define SEDS_EK_UNSIGNED 1
#define TX_TIMER_TICKS_PER_SECOND 1000U
static ULONG tx_time_get(void){return 777000U;}
static uint8_t sent[64]; static size_t sent_len; static unsigned sends;
static SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType t,const void*d,size_t n,size_t e,int k){
 assert(r&&t==SEDS_DT_BOARD_STATE_STATUS&&e==1&&k==SEDS_EK_UNSIGNED);
 memcpy(sent,d,n);sent_len=n;++sends;return SEDS_OK;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*o){
 *o=(BoardRecoverySnapshot){.actual_state=15U,.flags=BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_STATE_VALID|BOARD_RECOVERY_OUTPUTS_VALID,
  .abort_epoch=11U,.actual_outputs_mask=0x3U};return 0;}
static uint64_t board_commissioning_query_generation(void){return 0x0A0B0C0D0E0F1011ULL;}
static bool board_commissioning_uncertain_first(void){return false;}
static bool board_commissioning_pending(void){return false;}
static uint64_t last_query_issuer=0xC1C2C3C4C5C6C7C8ULL;
@WIRE@
@STATUS@
@PUSH@
static uint64_t le64(const uint8_t*p){uint64_t v=0;for(int i=7;i>=0;--i)v=v<<8|p[i];return v;}
static uint32_t le32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
int main(void){
 SedsRouter router=1;
 assert(board_recovery_push_abort_status(NULL)==SEDS_BAD_ARG&&sends==0);
 assert(board_recovery_push_abort_status(&router)==SEDS_OK&&sends==1&&sent_len==40U);
 assert(sent[0]==BOARD_RECOVERY_VERSION&&sent[1]==BOARD_RECOVERY_RESULT_OK&&sent[2]==15U);
 assert((sent[3]&BOARD_RECOVERY_ABORT_LATCHED)&&(sent[3]&BOARD_RECOVERY_STATE_VALID));
 assert(le64(sent+4)==0U);                              /* nonce 0: unsolicited */
 assert(le64(sent+12)==0x0A0B0C0D0E0F1011ULL);          /* the QUERY reply's generation */
 assert(le32(sent+20)==11U&&le32(sent+24)==777000U&&le32(sent+28)==3U);
 assert(le64(sent+32)==0xC1C2C3C4C5C6C7C8ULL);          /* last valid QUERY issuer */
 return 0;
}
'''


class AbortStatusPushTests(unittest.TestCase):
    def test_a_lost_abort_push_is_pushed_again_until_acknowledged(self):
        # T30: the first 4 copies of the Abort status were all lost at 30%
        # loss and nothing re-pushed it. A later copy delivers it within 5 s;
        # an acknowledged copy ends the pushes; pushes stop 5 s after the edge.
        _, push = self._push_source()
        prelude = EDGE_HARNESS[:EDGE_HARNESS.index('int main(void){')].replace('@PUSH@', push)
        _build_and_run(prelude + LOST_CASES)

    def test_routine_statuses_keep_the_three_retry_budget(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        self.assertIn('#define TELEMETRY_RELIABLE_MAX_RETRIES 3U', telemetry)
        self.assertIn('tuning.reliable_max_retries = TELEMETRY_RELIABLE_MAX_RETRIES;', telemetry)

    def _push_source(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        helper = (_function(telemetry, 'static bool abort_status_pending(') + '\n'
                  if 'static bool abort_status_pending(' in telemetry else '')
        return telemetry, helper + _function(telemetry, 'static void telemetry_push_abort_status(void)')

    def test_first_edge_of_each_latch_episode_pushes_once(self):
        telemetry, push = self._push_source()
        _build_and_run(EDGE_HARNESS.replace('@PUSH@', push))
        loop = _function(telemetry, 'void telemetry_entry(ULONG _)')
        self.assertIn('telemetry_republish_flight_state();\n    telemetry_push_abort_status();', loop)

    def test_push_is_a_nonce_zero_abort_latched_status(self):
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        wire = (ROOT / 'Core/Src/board_recovery_wire.c').read_text()
        wire = '\n'.join(l for l in wire.splitlines() if not l.startswith('#include'))
        _build_and_run(PUSH_HARNESS.replace('@WIRE@', wire)
                       .replace('@STATUS@', _function(transport, 'static SedsResult commissioning_status('))
                       .replace('@PUSH@', _function(transport, 'SedsResult board_recovery_push_abort_status(')))

    def test_issuer_is_cached_from_each_valid_query(self):
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        handler = _function(transport, 'SedsResult board_recovery_handle(')
        query = handler.index('if(request.operation==BOARD_RECOVERY_QUERY){')
        self.assertLess(handler.index('authorized_primary(router,packet)'), query)
        self.assertLess(handler.index('board_recovery_decode_request('), query)
        self.assertIn('last_query_issuer=request.issuer_session;', handler[query:query + 100])


if __name__ == '__main__':
    unittest.main()

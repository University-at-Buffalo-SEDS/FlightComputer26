"""Execute the actual FC managed-state callback and its recovery-queue contract."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ManagedPostinitTests(unittest.TestCase):
    def test_managed_armed_reaches_recovery_once_and_retries_queue_pressure(self):
        source = (ROOT / "Core/Src/distribution.c").read_text()
        start = source.index("static inline SedsResult\nimplicit_postinit(")
        end = source.index("static SedsResult on_fc_packet_impl(", start)
        body = source[start:end]
        if "flight_state_cache_apply_network_update(" not in body:
            # The previous actual default hook acknowledged the value but did
            # not reach the recovery queue. Keep that negative control usable.
            cache = (ROOT / "Core/Src/flight_state_cache.c").read_text()
            a = cache.index("__attribute__((weak)) SedsResult")
            body += cache[a:cache.index("void flight_state_cache_restore", a)]
        code = r"""
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;
typedef uint32_t fc_msg;
enum { SEDS_OK=0, SEDS_ERR=-1, SEDS_HANDLER_ERROR=-9, SEDS_IO=-5,
 SEDS_DT_FLIGHT_STATE=23, TX_SUCCESS=0, TX_NO_WAIT=0,
 G_Idle=1, G_Armed=6, G_Aborted=15, Postinit=1, Postinit_Signal=100 };
typedef struct { unsigned ty; const uint8_t *payload; size_t payload_len; } SedsPacketView;
static int seds_syscall, queue_result, queued, current_state;
static uint32_t g_conf;
#define Postinit_Requested (1u<<8)
#define option(x) (x)
#define current() (current_state)
#define fc_mask(x) ((fc_msg)(x))
static int tx_queue_send(int *queue, fc_msg *message, int wait) {
 assert(queue == &seds_syscall && *message == Postinit_Signal && wait == TX_NO_WAIT);
 if (queue_result == TX_SUCCESS) queued++;
 return queue_result;
}
""" + body + r"""
int main(void) {
 uint8_t value=G_Armed;
 SedsPacketView p={SEDS_DT_FLIGHT_STATE,&value,1};
 assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==1);
 assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==1);
 value=G_Idle; assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==1);
 value=G_Armed; queue_result=2;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_IO && queued==1);
 queue_result=TX_SUCCESS;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==2);
 value=G_Idle; assert(flight_state_cache_apply_network_update(&p)==SEDS_OK);
 value=G_Armed; current_state=Postinit;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==2);
 current_state=0; g_conf=Postinit_Requested;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_OK && queued==2);
 assert(flight_state_cache_apply_network_update(NULL)==SEDS_HANDLER_ERROR);
 p.payload=NULL; assert(flight_state_cache_apply_network_update(&p)==SEDS_HANDLER_ERROR);
 p.payload=&value; p.payload_len=2;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_HANDLER_ERROR);
 p.payload_len=1; value=16;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_HANDLER_ERROR);
 value=G_Armed; p.ty=999;
 assert(flight_state_cache_apply_network_update(&p)==SEDS_HANDLER_ERROR);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "managed-postinit"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-x", "c", "-", "-o", str(exe)],
                           input=code, text=True, check=True)
            subprocess.run([str(exe)], check=True)

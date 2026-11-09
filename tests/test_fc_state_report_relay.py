"""FC flight-state reports must leave the FC and cross relays (no hardware).

FLIGHT_STATE is a managed network variable that only the primary station may
write on a board. The SDK runs each board's update callback inside its own
transmit and inside every relay; failing the FC's report there aborted the
FC transmit and the RF relay, so the GS never saw an FC flight state. A report
that is not a station write is ignorable: not applied, but never a failure.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(text, name):
    i = text.index(name + '(')
    start = text.rfind('\n', 0, i) + 1
    j = text.index('{', i)
    depth, k = 1, j + 1
    while depth:
        depth += (text[k] == '{') - (text[k] == '}')
        k += 1
    return text[start:k]


class FcStateReportRelayTests(unittest.TestCase):
    def test_only_non_station_flight_state_reports_are_ignorable(self):
        text = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        code = r'''#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef int SedsRouter;
typedef struct { uint32_t ty; const char *sender; size_t sender_len; } SedsPacketView;
#define SEDS_DT_FLIGHT_STATE 11U
static int router; static SedsRouter *recovery_router = &router;
bool seds_router_sender_matches_hostname(SedsRouter *r, const char *s, size_t len, const char *n, size_t nl)
{ assert(r); return s && len == nl && !memcmp(s, n, nl); }
''' + function(text, 'authorized_primary') + '\n' + function(text, 'board_recovery_managed_report_ignorable') + r'''
int main(void) {
  SedsPacketView p = { SEDS_DT_FLIGHT_STATE, "FC", 2 };
  assert(board_recovery_managed_report_ignorable(&p));
  p.sender = "RF"; assert(board_recovery_managed_report_ignorable(&p));
  p.sender = "GS"; assert(!board_recovery_managed_report_ignorable(&p));
  p.sender = NULL; p.sender_len = 0; assert(!board_recovery_managed_report_ignorable(&p));
  p.sender = "FC"; p.sender_len = 2; p.ty = 12U; assert(!board_recovery_managed_report_ignorable(&p));
  p.ty = SEDS_DT_FLIGHT_STATE; recovery_router = NULL; assert(!board_recovery_managed_report_ignorable(&p));
  assert(!board_recovery_managed_report_ignorable(NULL));
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'probe.c'
            exe = Path(tmp) / 'probe'
            src.write_text(code)
            subprocess.run(['cc', '-std=c17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_flight_state_callback_never_fails_a_refusal(self):
        # Reports and refused station writes alike: retired, counted, acknowledged.
        text = (ROOT / 'Core/Src/flight_state_cache.c').read_text()
        start = text.index('static SedsResult persist_update(const SedsPacketView *packet, void *user)\n{')
        body = function(text[start:], 'persist_update')
        denied = body.index('if (!board_recovery_managed_update_allowed(packet))')
        refusal = body[denied:body.index('}', denied)]
        self.assertIn('++g_reliable_refusal_count;', refusal)
        self.assertIn('return SEDS_OK;', refusal)
        self.assertNotIn('SEDS_HANDLER_ERROR', refusal)

    def test_fc_endpoint_handler_does_not_fail_its_own_report(self):
        body = function((ROOT / 'Core/Src/distribution.c').read_text(), 'on_fc_packet_impl')
        gate = body.index('if ((pkt->ty == SEDS_DT_FLIGHT_STATE && !board_recovery_managed_update_allowed(pkt))')
        self.assertIn('++g_reliable_refusal_count;\n    return SEDS_OK;', body[gate:gate + 300])

    def test_flight_state_report_is_queued_off_the_evaluation_stack(self):
        # Sent directly from the evaluation task, the now-delivered report ran
        # the SDK's managed-variable and route work on its 12 KiB stack and
        # hard-faulted the FC at T+6 s in simulation.
        platform = (ROOT / 'Core/Inc/platform.h').read_text()
        start = platform.index('#ifdef TELEMETRY_ENABLED') if '#ifdef TELEMETRY_ENABLED' in platform else 0
        macro = platform[platform.index('#define log_flight_state(state)', start):][:200]
        self.assertIn('log_telemetry_queued_u8(SEDS_DT_FLIGHT_STATE', macro)
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        body = function(telemetry, 'log_telemetry_queued_u8')
        self.assertIn('seds_router_log_queue_typed(', body)
        self.assertNotIn('seds_router_log_bytes(', body)

    def test_flight_state_is_republished_from_launch_on(self):
        # One-shot ordered reports lose their queued tail when a predecessor
        # exhausts end-to-end retries; the GS must still converge on Landed.
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        body = function(telemetry, 'telemetry_republish_flight_state')
        self.assertIn('if (local < Launch || local > Recovery) return;', body)
        self.assertIn('FC_STATE_REPUBLISH_MS', body)
        self.assertIn('log_telemetry_queued_u8(SEDS_DT_FLIGHT_STATE, &value, 1U)', body)
        self.assertIn('global_state_of(local)', body)
        loop = telemetry[telemetry.index('void telemetry_entry(ULONG _)'):]
        self.assertIn('telemetry_republish_flight_state();', loop)
        fcapi = (ROOT / 'Core/Inc/fcapi.h').read_text()
        mapping = function(fcapi, 'global_state_of')
        self.assertNotIn('sm.global_state', mapping)


if __name__ == '__main__':
    unittest.main()

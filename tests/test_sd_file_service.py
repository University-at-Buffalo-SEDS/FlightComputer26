"""FC SD transfer service v2 (P2P port 4511): wire, gating, session and FileX side.

Runs the production sources on the host: Core/Inc/sd_file_wire.h directly,
Core/Src/sd_file_stream.c with SDK/RTOS stubs, and the FC_SD_SERVICE block of
Core/Src/sdpipeline.c against a small FileX model. The streaming engine itself
(sd_xfer.c) is covered by tests/test_sd_xfer.py."""
import hashlib
import pathlib
import sys
import re
import subprocess
import tempfile
import unittest
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
from native_sdk import configured_dependency_source  # noqa: E402
ENGINE = [str(ROOT / 'Core/Src/sd_xfer.c'), str(ROOT / 'Core/Src/sd_xfer_codec.c')]
CFLAGS = ['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
          '-Wno-unused-parameter', '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc')]


def run_c(code, extra=()):
    if extra:
        code = '#include <stddef.h>\n#include <stdint.h>\n' + crc_source() + code
    with tempfile.TemporaryDirectory() as tmp:
        exe = pathlib.Path(tmp) / 'check'
        subprocess.run(CFLAGS + ['-x', 'c', '-', '-x', 'none', *extra, '-o', str(exe)], input=code, text=True, check=True)
        return subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout


def strip_includes(text):
    return '\n'.join(l for l in text.splitlines() if not l.startswith('#include'))


def crc_source():
    src = (ROOT / 'Core/Src/board_crc32.c').read_text()
    table = src[src.index('static const uint32_t k_crc_nibble'):]
    table = table[:table.index('};') + 2]
    fn = src[src.index('uint32_t board_crc32_software('):]
    return table + '\n' + fn[:fn.index('\n}\n') + 3]


FRAME_PRINT = r'''
static void hex(const char *tag, const uint8_t *p, size_t n)
{ printf("%s ", tag); for (size_t i = 0; i < n; ++i) printf("%02x", p[i]); printf("\n"); }
'''


def frames(out, tag):
    return [bytes.fromhex(l.split(' ', 1)[1]) for l in out.splitlines() if l.startswith(tag + ' ')]


class WireTests(unittest.TestCase):
    def test_request_parsing_and_bounds(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <assert.h>
#include <stdio.h>
static uint8_t parse(const uint8_t *p, size_t n) { sd_file_request_t r; return sd_file_parse_request(p, n, &r); }
static size_t open_req(uint8_t *p, uint8_t xfer, uint8_t dir, const char *name)
{
    size_t n = strlen(name);
    const uint8_t fixed[19] = {0x25, xfer, dir, 1, 0x10, 0, 0, 0, 0, 0, 0, 0, 96, 0, 0xcc, 1, 8, 16, (uint8_t)n};
    memcpy(p, fixed, 19); memcpy(p + 19, name, n); return 19 + n;
}
static uint8_t hash_req(uint8_t dir, const char *name, int len_adjust)
{
    uint8_t p[100]; size_t n = strlen(name);
    p[0] = 0x27; p[1] = dir; p[2] = (uint8_t)n; memcpy(p + 3, name, n);
    return parse(p, 3 + n + len_adjust);
}
int main(void)
{
    const uint8_t cancel[] = {0x23}, cancel_long[] = {0x23, 0};
    assert(parse(cancel, 1) == SD_FILE_STATUS_OK && parse(cancel_long, 2) == SD_FILE_STATUS_BAD_MESSAGE);
    const uint8_t list[] = {0x21, 0, 0x34, 0x12}, list_dir1[] = {0x21, 1, 0, 0}, list_short[] = {0x21, 0, 0};
    sd_file_request_t r;
    assert(sd_file_parse_request(list, 4, &r) == 0 && r.op == 0x21 && r.index == 0x1234 && r.dir == 0);
    assert(parse(list_dir1, 4) == 1 && parse(list_short, 3) == 1);
    const uint8_t unknown[] = {0x24}, empty[] = {0}, read_v1[] = {0x22, 0, 0, 0, 0, 0, 8, 'i','r','e','c','.','c','s','v'};
    assert(parse(unknown, 1) == 1 && parse(empty, 0) == 1 && parse(NULL, 1) == 1);
    assert(parse(read_v1, sizeof read_v1) == 1);         /* v1 READ is gone */
    uint8_t p[120];
    size_t n = open_req(p, 7, 0, "irec-1795392000-000.csv");
    assert(sd_file_parse_request(p, n, &r) == 0 && r.op == 0x25 && r.open.xfer == 7 && r.open.flags == 1 &&
           r.open.offset == 16 && r.open.length == 0 && r.open.rate == 96 * 16 && r.open.msg == 0x1cc &&
           r.open.window == 8 && r.open.block_kib == 16 && !strcmp(r.name, "irec-1795392000-000.csv"));
    assert(parse(p, n - 1) == 1 && parse(p, n + 1) == 1);  /* name length mismatch */
    n = open_req(p, 0, 0, "irec.csv"); assert(parse(p, n) == 1);           /* xfer 0 is reserved */
    n = open_req(p, 1, 1, "irec.csv"); assert(parse(p, n) == 1);           /* only dir 0 */
    n = open_req(p, 1, 0, "IGT/IMU0.CSV"); assert(parse(p, n) == 1);
    n = open_req(p, 1, 0, "../irec.csv"); assert(parse(p, n) == 1);
    n = open_req(p, 1, 0, "notes.txt"); assert(parse(p, n) == 1);
    n = open_req(p, 1, 0, "IREC_LAUNCH-5-001.CSV"); assert(parse(p, n) == 0);  /* case-insensitive */
    assert(hash_req(0, "irec-1-000.csv", 0) == 0 && hash_req(0, "irec-1-000.csv", 1) == 1);
    assert(hash_req(0, "irec-1-000.csv", -1) == 1 && hash_req(2, "irec-1-000.csv", 0) == 1);
    assert(hash_req(0, "IMU0.CSV", 0) == 1);
    char longname[65]; memset(longname, 'a', 64); memcpy(longname, "irec", 4); memcpy(longname + 60, ".csv", 4); longname[64] = 0;
    assert(hash_req(0, longname, 0) == 1);               /* 64 > SD_FILE_NAME_MAX */
    longname[59] = 0; memcpy(longname + 55, ".csv", 5);
    assert(hash_req(0, longname, 0) == 0);
    memset(longname, 'a', 63); memcpy(longname, "irec", 4); memcpy(longname + 59, ".csv", 4); longname[63] = 0;
    n = open_req(p, 1, 0, longname); assert(n == SD_FILE_REQUEST_MAX && parse(p, n) == 0);
    assert(parse(p, SD_FILE_REQUEST_MAX + 1) == 1);      /* over the request bound */
    const uint8_t ack[11] = {0x26, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1};
    assert(parse(ack, 11) == 1);                         /* ACK is not a request */
    puts("ok");
}
''', ENGINE)
        self.assertEqual(out.strip(), 'ok')

    def test_log_name_filter(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
int main(void)
{
    const char *names[] = {"irec-1-000.csv", "IREC_LAUNCH-1-000.CSV", "IrEc-x.CsV", "irec.csv", "irec-1.csv.bak",
                           "rec-1.csv", "irec-1.cs", "irec", ".csv", "irec_.csv", "irex-1.csv", "irec-1.csw",
                           "IGT", "irec\x01.csv", "jrec-1.csv", "hrec-1.csv"};
    for (unsigned i = 0; i < sizeof names / sizeof *names; ++i)
        printf("%d\n", sd_file_fc_log_name(names[i], strlen(names[i])));
}
''')
        self.assertEqual(out.split(), ['1', '1', '1', '1', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0'])

    def test_answer_encoding(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
''' + FRAME_PRINT + r'''
int main(void)
{
    uint8_t f[SD_FILE_ANSWER_MAX];
    hex("L", f, sd_file_encode_list(f, 0, 0, 0x0102, 0x11223344u, SD_FILE_ENTRY_OPEN, "irec-1-000.csv", 14));
    hex("L", f, sd_file_encode_list(f, 6, 0, 3, 99, 1, "ignored", 7));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_OPEN, 2));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_HASH, 3));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_CANCEL, 0));
    hex("S", f, sd_file_encode_status(f, 0x22, 1));
}
''', ENGINE)
        l = frames(out, 'L')
        self.assertEqual(l[0], bytes([0xa1, 0, 0, 2, 1, 0x44, 0x33, 0x22, 0x11, 1, 14, 0]) + b'irec-1-000.csv')
        self.assertEqual(l[1], bytes([0xa1, 6, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0]))
        st = frames(out, 'S')
        self.assertEqual(st[0], bytes([0xa5, 2] + [0] * 17))
        self.assertEqual(st[1], bytes([0xa7, 3] + [0] * 37))
        self.assertEqual(st[2], bytes([0xa3, 0]))
        self.assertEqual(st[3], bytes([0xa2, 1]))

    def test_gating_table(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
int main(void)
{
    for (unsigned local = 0; local <= 11; ++local) {
        printf("%d", sd_file_fc_state_allowed((uint8_t)local, false, 0));
        for (unsigned net = 0; net <= 17; ++net) printf("%d", sd_file_fc_state_allowed((uint8_t)local, true, (uint8_t)net));
        printf("\n");
    }
}
''')
        rows = out.split()
        for local, row in enumerate(rows):
            for col, bit in enumerate(row):
                known, net = col != 0, col - 1
                # Landed 9 / Recovery 10 regardless of network; Startup 0 with a known
                # network Startup 0 / Idle 1. Postinit, Armed, Launch..Reefing never.
                expect = local in (9, 10) or (local == 0 and known and net in (0, 1))
                self.assertEqual(bit == '1', expect, (local, known, net))


STREAM_STUBS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
typedef struct { char opaque[400]; } FX_FILE;
typedef struct { uint32_t state[8]; uint64_t bits; uint8_t block[64]; size_t used; } launchcore_sha256_ctx_t;
#include "sd_file_stream.h"
typedef enum { SEDS_OK = 0, SEDS_ERR = -1, SEDS_BAD_ARG = -2 } SedsResult;
typedef struct SedsRouter { int unused; } SedsRouter;
typedef struct {
    uint8_t kind; uint32_t stream_id, peer_stream_id, sequence;
    const char *peer_hostname; size_t peer_hostname_len; uint32_t peer_address;
    uint16_t local_port, peer_port; const uint8_t *payload; size_t payload_len;
} SedsP2pStreamEventView;
typedef SedsResult (*SedsP2pStreamHandlerFn)(const SedsP2pStreamEventView *, void *);
#define SEDS_P2P_STREAM_ACCEPTED 1U
#define SEDS_P2P_STREAM_DATA 3U
#define SEDS_P2P_STREAM_CLOSED 4U
#define SEDS_P2P_STREAM_RESET 5U
static uint32_t primask, tick, wakes, resets, queued, allocs, frees, send_fail;
static uint8_t local_state; static bool net_known; static uint8_t net_state;
static uint8_t sent[64][SD_XFER_MSG_MAX]; static size_t sent_len[64]; static unsigned nsent;
static SedsP2pStreamHandlerFn handler;
static SedsRouter router;
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; }
static void __set_PRIMASK(uint32_t p) { primask = p; }
static uint32_t HAL_GetTick(void) { return tick; }
#define current() local_state
bool flight_state_cache_has_value(void) { return net_known; }
uint8_t flight_state_cache_value(void) { return net_state; }
bool telemetry_tlsf_admit(size_t a, size_t l) { return a > l; }
void *telemetry_tlsf_malloc(size_t n) { ++allocs; return malloc(n); }
void telemetry_tlsf_free(void *p) { ++frees; free(p); }
void sd_pipeline_wake(void) { assert(!primask); ++wakes; }
bool seds_router_sender_matches_hostname(const SedsRouter *r, const char *s, size_t n, const char *h, size_t hn)
{ (void)r; return n == hn && !memcmp(s, h, n); }
SedsResult seds_router_send_p2p_stream(SedsRouter *r, uint32_t id, const uint8_t *p, size_t n)
{ (void)r; assert(id == 7 && n <= SD_XFER_MSG_MAX && !primask); if (send_fail) return SEDS_ERR;
  memcpy(sent[nsent], p, n); sent_len[nsent++] = n; return SEDS_OK; }
SedsResult seds_router_reset_p2p_stream(SedsRouter *r, uint32_t id) { (void)r; assert(id == 7); ++resets; return SEDS_OK; }
int32_t seds_router_has_queued_work(SedsRouter *r) { (void)r; return (int32_t)queued; }
SedsResult seds_router_bind_p2p_stream_port(SedsRouter *r, uint16_t port, SedsP2pStreamHandlerFn cb, void *u)
{ (void)r; (void)u; assert(port == 4511); handler = cb; return SEDS_OK; }
static uint8_t file_bytes[6000];
static bool file_read(void *ctx, uint32_t off, uint8_t *dst, uint32_t len)
{ (void)ctx; if (off + len > sizeof file_bytes) return false; memcpy(dst, file_bytes + off, len); return true; }
'''

STREAM_MAIN = FRAME_PRINT + r'''
static SedsResult ev(uint8_t kind, const char *peer, const uint8_t *p, size_t n)
{
    SedsP2pStreamEventView e = {.kind = kind, .stream_id = 7, .peer_stream_id = 9, .peer_hostname = peer,
        .peer_hostname_len = strlen(peer), .peer_address = 0x1234, .local_port = 4511, .peer_port = 4511,
        .payload = p, .payload_len = n};
    return handler(&e, NULL);
}
/* OPEN xfer 5, SDX, offset 0, whole file, rate 100*16 B/s, msg 460, window 2, 16 KiB blocks. */
static const uint8_t OPEN[] = {0x25, 5, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0xcc, 1, 2, 16, 14,
                               'i','r','e','c','-','1','-','0','0','0','.','c','s','v'};
static const uint8_t LIST[] = {0x21, 0, 0, 0}, CANCEL[] = {0x23};
static const uint8_t READ_V1[] = {0x22, 0, 0, 0, 0, 0, 8, 'i','r','e','c','.','c','s','v'};
static void pump(void) { for (unsigned i = 0; i < 400; ++i) { tick += 1; sd_file_stream_poll(); } }
int main(void)
{
    for (unsigned i = 0; i < sizeof file_bytes; ++i) file_bytes[i] = (uint8_t)('a' + i % 26);
    assert(sd_file_stream_init(&router) == SEDS_OK && handler);
    local_state = 9;                                   /* Landed */
    /* Peer check: GS and RRLS only. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "VB", NULL, 0) == SEDS_BAD_ARG && allocs == 0);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "RRLS", NULL, 0) == SEDS_OK && allocs == 1);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 1); /* replaces it */
    assert(sd_file_stream_session_active() && !sd_file_stream_transfer_active());
    /* OPEN: the router callback only queues and wakes the SD thread. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && wakes == 1 && nsent == 0);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && nsent == 1);
    hex("BUSY", sent[0], sent_len[0]); nsent = 0;
    sd_file_request_t rq; uint32_t ticket;
    assert(sd_file_stream_claim(&rq, &ticket) && rq.op == 0x25 && rq.open.xfer == 5 && !strcmp(rq.name, "irec-1-000.csv"));
    assert(!sd_file_stream_claim(&rq, &ticket) && sd_file_stream_ticket_current(ticket));
    uint8_t *block = sd_file_stream_block();
    assert(block && sd_file_stream_handle() == block + 512 && sd_file_stream_scratch());
    sd_file_stream_open(ticket, &rq.open, sizeof file_bytes, false);
    assert(sd_file_stream_transfer_active());
    tick = 1000; sd_file_stream_poll(); assert(nsent == 1);
    hex("OPENED", sent[0], sent_len[0]); nsent = 0;
    /* DATA: one message per produce; paced at the station's rate and window. */
    assert(sd_file_stream_produce(file_read, NULL, tick));
    assert(!sd_file_stream_produce(file_read, NULL, tick));  /* slot still full */
    tick += 2; sd_file_stream_poll(); assert(nsent == 1);
    const uint32_t first = tick;
    assert(sd_file_stream_produce(file_read, NULL, tick));
    while (nsent == 1 && tick < first + 1000) { tick += 1; sd_file_stream_poll(); }
    printf("GAP %u\n", tick - first);                  /* 460 B at 1600 B/s */
    assert(!sd_file_stream_produce(file_read, NULL, tick));  /* window of 2 */
    /* ACK from the station: window opens again; stale ids are ignored. */
    const uint8_t stale[] = {0x26, 4, 2, 0, 0, 0, 0, 0, 100, 0, 2};
    const uint8_t ack[] = {0x26, 5, 2, 0, 0, 0, 0, 0, 100, 0, 2};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", stale, sizeof stale) == SEDS_OK && !sd_file_stream_produce(file_read, NULL, tick));
    const uint32_t w = wakes;
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", ack, sizeof ack) == SEDS_OK && wakes == w + 1);
    assert(sd_file_stream_produce(file_read, NULL, tick));
    /* Busy router: at least 25 ms between frames. */
    queued = 1; tick += 400; const uint32_t before = nsent;
    sd_file_stream_poll(); assert(nsent == before + 1);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    uint8_t answer[12] = {0xa1, 6};
    sd_file_stream_answer(ticket, answer, sizeof answer);
    tick += 10; sd_file_stream_poll(); assert(nsent == before + 1);
    tick += 16; sd_file_stream_poll(); assert(nsent == before + 2);
    queued = 0; send_fail = 1;
    assert(sd_file_stream_produce(file_read, NULL, tick) || 1);
    for (unsigned i = 0; i < nsent; ++i) hex("DATA", sent[i], sent_len[i]);
    nsent = 0; send_fail = 0;
    /* CANCEL ends the transfer and drops the queued message. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", CANCEL, 1) == SEDS_OK);
    assert(nsent == 1 && sent_len[0] == 2 && sent[0][0] == 0xa3 && sent[0][1] == 0); nsent = 0;
    assert(!sd_file_stream_transfer_active()); pump(); assert(nsent == 0);
    /* Bad requests answer at once with BAD_MESSAGE, v1 READ included. */
    const uint8_t bad[] = {0x21, 1, 0, 0};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", bad, 4) == SEDS_OK && nsent == 1);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ_V1, sizeof READ_V1) == SEDS_OK && nsent == 2);
    hex("BAD", sent[0], sent_len[0]); hex("BAD", sent[1], sent_len[1]); nsent = 0;
    /* A CANCEL stales a busy answer. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", CANCEL, 1) == SEDS_OK); nsent = 0;
    sd_file_stream_open(ticket, &rq.open, sizeof file_bytes, false);
    pump(); assert(nsent == 0 && !sd_file_stream_transfer_active());
    /* Leaving the allowed states: a request is refused, the session is reset. */
    local_state = 2;                                   /* Armed */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && nsent == 1);
    hex("STATE", sent[0], sent_len[0]); nsent = 0;
    sd_file_stream_handle_open(true);                  /* SD thread still owns the handle */
    sd_file_stream_poll();
    assert(resets == 1 && !sd_file_stream_session_active() && frees == 0);
    sd_file_stream_poll(); assert(resets == 1 && frees == 0);
    sd_file_stream_handle_open(false); sd_file_stream_poll(); assert(frees == 1 && !sd_file_stream_block());
    /* A new session is refused outside the allowed states and after an unknown network. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR);
    local_state = 0; net_known = false;
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR);
    net_known = true; net_state = 1;                   /* Startup + network Idle */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 2);
    /* A transfer in flight ends with the session; a busy SD turn finishing
     * after the reset publishes nothing. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, sizeof file_bytes, false); pump(); nsent = 0;
    assert(sd_file_stream_produce(file_read, NULL, tick));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    net_state = 6;                                     /* network Armed */
    sd_file_stream_poll(); assert(resets == 2 && !sd_file_stream_transfer_active());
    sd_file_stream_answer(ticket, answer, sizeof answer);
    net_state = 1; assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK);
    pump(); assert(nsent == 0);
    assert(ev(SEDS_P2P_STREAM_CLOSED, "GS", NULL, 0) == SEDS_OK && !sd_file_stream_session_active());
    sd_file_stream_poll(); assert(frees == 2);
    /* A remount during an active transfer: DATA stops, the queued message and
     * a busy answer are dropped, the stream is reset and the area released. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 3);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, sizeof file_bytes, false); pump(); nsent = 0;
    assert(sd_file_stream_produce(file_read, NULL, tick) && sd_file_stream_transfer_active());
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_handle_open(true);
    sd_file_stream_media_remounted();
    assert(!sd_file_stream_transfer_active() && !sd_file_stream_ticket_current(ticket));
    sd_file_stream_answer(ticket, answer, sizeof answer);
    sd_file_stream_poll();
    assert(resets == 3 && !sd_file_stream_session_active() && nsent == 0);
    assert(!sd_file_stream_produce(file_read, NULL, tick));
    sd_file_stream_poll(); assert(frees == 3 && !sd_file_stream_block());
    /* Without a session a remount resets nothing. */
    sd_file_stream_media_remounted(); sd_file_stream_poll(); assert(resets == 3);
    puts("done");
}
'''


class StreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = strip_includes((ROOT / 'Core/Src/sd_file_stream.c').read_text())
        cls.out = run_c(STREAM_STUBS + src + STREAM_MAIN, ENGINE)

    def test_session_gating_and_answers(self):
        self.assertIn('done', self.out)
        self.assertEqual(frames(self.out, 'BUSY')[0], bytes([0xa1, 3] + [0] * 10))
        self.assertEqual(frames(self.out, 'BAD')[0], bytes([0xa1, 1] + [0] * 10))
        self.assertEqual(frames(self.out, 'BAD')[1], bytes([0xa2, 1]))
        self.assertEqual(frames(self.out, 'STATE')[0], bytes([0xa5, 2, 5] + [0] * 16))
        opened = frames(self.out, 'OPENED')[0]
        self.assertEqual(opened[:4], bytes([0xa5, 0, 5, 1]))
        self.assertEqual(opened[4:16], (6000).to_bytes(4, 'little') + bytes(4) + (6000).to_bytes(4, 'little'))
        self.assertEqual(opened[16:], bytes([0xcc, 1, 16]))

    def test_data_is_paced_windowed_and_decodable(self):
        gap = int(next(l.split()[1] for l in self.out.splitlines() if l.startswith('GAP ')))
        data = frames(self.out, 'DATA')
        self.assertEqual(gap, -(-len(data[0]) * 1000 // 1600), 'first message length at 1600 B/s')
        self.assertEqual([d[0] for d in data[:2]], [0xa6, 0xa6])
        self.assertEqual([int.from_bytes(d[3:5], 'little') for d in data if d[0] == 0xa6][:3], [0, 1, 2])
        self.assertTrue(all(len(d) <= 460 for d in data))
        self.assertEqual(data[0][1:3], bytes([5, 0x05]))   # xfer 5, FIRST | SDX


FILEX_STUBS = r'''
#define FC_SD_SERVICE 1
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "sd_file_stream.h"
#include "launchcore/image.h"
typedef unsigned UINT; typedef unsigned long ULONG; typedef unsigned long long ULONG64; typedef char CHAR;
#define FX_SUCCESS 0U
#define FX_NOT_FOUND 4U
#define FX_INVALID_PATH 0x0DU
#define FX_NO_MORE_ENTRIES 0x0FU
#define FX_IO_ERROR 0x90U
#define FX_OPEN_FOR_READ 0U
#define FX_DIRECTORY 0x10U
#define FX_VOLUME 0x08U
#define FX_HIDDEN 0x02U
#define FX_SYSTEM 0x04U
#define FX_MAX_LONG_NAME_LEN 256U
#define TX_TIMER_TICKS_PER_SECOND 1000U
typedef struct { CHAR *fx_file_name; CHAR fx_file_name_buffer[FX_MAX_LONG_NAME_LEN]; ULONG64 fx_file_current_file_size;
                 int entry; ULONG pos; } FX_FILE;
typedef struct { struct { ULONG fx_path_current_entry; } fx_media_default_path; } FX_MEDIA;
static FX_MEDIA sdio_disk;
static FX_FILE file;               /* the writer's handle */
static uint8_t sd_file_open;
static bool sd_prepared;
static struct { unsigned full; } line;
static uint32_t clock_ms, opens, closes, io_fail, prepare_fail;
static uint32_t now_ms(void) { return clock_ms; }
static bool flight_sd_io_verified(void) { return !io_fail; }
static UINT flight_sd_prepare_io(void) { return prepare_fail ? FX_IO_ERROR : FX_SUCCESS; }
static UINT tx_semaphore_put(unsigned *s) { ++*s; return 0; }
#define TX_SEMAPHORE_ERROR 0x0CU
static UINT tx_semaphore_get(unsigned *s, ULONG t) { (void)s; (void)t; return 0; }
static void tx_thread_sleep(ULONG t) { (void)t; }
struct entry { const char *name; UINT attr; ULONG size; };
static struct entry dir[] = {
    {"SYSTEM~1", FX_DIRECTORY | FX_HIDDEN, 0}, {"IGT", FX_DIRECTORY, 0},
    {"irec-1795392000-000.csv", 0, 1300}, {"notes.txt", 0, 5}, {"irec-hidden.csv", FX_HIDDEN, 9},
    {"IREC_LAUNCH-1795392100-000.CSV", 0, 512}, {"irecdir.csv", FX_DIRECTORY, 0},
    {"rec-1.csv", 0, 4}, {"irec-1795392200-000.csv", 0, 0}, {"vol.csv", FX_VOLUME, 0},
};
#define NDIR (sizeof dir / sizeof *dir)
static UINT fx_directory_next_full_entry_find(FX_MEDIA *m, CHAR *name, UINT *attr, ULONG *size,
    UINT *y, UINT *mo, UINT *d, UINT *h, UINT *mi, UINT *s)
{
    (void)y; (void)mo; (void)d; (void)h; (void)mi; (void)s;
    ULONG i = m->fx_media_default_path.fx_path_current_entry;
    if (io_fail) return FX_IO_ERROR;
    if (i >= NDIR) return FX_NO_MORE_ENTRIES;
    strcpy(name, dir[i].name); *attr = dir[i].attr; *size = dir[i].size;
    m->fx_media_default_path.fx_path_current_entry = i + 1;
    return FX_SUCCESS;
}
static UINT fx_file_open(FX_MEDIA *m, FX_FILE *f, CHAR *name, UINT mode)
{
    (void)m; assert(mode == FX_OPEN_FOR_READ);
    for (unsigned i = 0; i < NDIR; ++i)
        if (!strcasecmp(dir[i].name, name) && !(dir[i].attr & FX_DIRECTORY)) {
            ++opens; strcpy(f->fx_file_name_buffer, dir[i].name); f->fx_file_name = f->fx_file_name_buffer;
            f->fx_file_current_file_size = dir[i].size; f->entry = (int)i; return FX_SUCCESS;
        }
    return FX_NOT_FOUND;
}
static UINT fx_file_close(FX_FILE *f) { (void)f; ++closes; return FX_SUCCESS; }
static UINT fx_file_seek(FX_FILE *f, ULONG at) { f->pos = at; return FX_SUCCESS; }
static UINT fx_file_read(FX_FILE *f, void *buf, ULONG want, ULONG *actual)
{
    assert(want <= 512 && f->pos % 512 == 0);
    ULONG size = (ULONG)f->fx_file_current_file_size, n = f->pos + want > size ? size - f->pos : want;
    for (ULONG i = 0; i < n; ++i) ((uint8_t *)buf)[i] = (uint8_t)((f->pos + i) * 31 + f->entry);
    *actual = n; return io_fail ? FX_IO_ERROR : FX_SUCCESS;
}
/* sd_file_stream.c side */
static uint8_t area[512 + sizeof(FX_FILE)]; static launchcore_sha256_ctx_t sha_ctx;
static bool have_area = true, allowed = true, session = true, handle_open, xfer_active;
static sd_file_request_t pending; static bool has_pending; static uint32_t ticket_now = 1;
static uint8_t last[SD_FILE_ANSWER_MAX]; static size_t last_len; static unsigned answered;
static uint32_t opened_size; static bool opened_growing; static uint8_t produced[2048]; static bool produce_ok;
bool sd_file_stream_claim(sd_file_request_t *r, uint32_t *t) { if (!has_pending) return false; *r = pending; *t = ticket_now; has_pending = false; return true; }
bool sd_file_stream_ticket_current(uint32_t t) { return t == ticket_now; }
uint8_t *sd_file_stream_block(void) { return have_area ? area : NULL; }
void *sd_file_stream_handle(void) { return have_area ? area + 512 : NULL; }
void *sd_file_stream_scratch(void) { return &sha_ctx; }
void sd_file_stream_handle_open(bool open) { handle_open = open; }
void sd_file_stream_answer(uint32_t t, const uint8_t *f, size_t n) { if (t != ticket_now) return; memcpy(last, f, n); last_len = n; ++answered; }
void sd_file_stream_open(uint32_t t, const sd_xfer_open_t *o, uint32_t size, bool growing)
{ (void)o; opened_size = size; opened_growing = growing; xfer_active = true; uint8_t a[SD_XFER_OPEN_ANSWER] = {0xa5}; sd_file_stream_answer(t, a, sizeof a); }
bool sd_file_stream_transfer_active(void) { return xfer_active; }
bool sd_file_stream_produce(sd_xfer_read_fn read, void *ctx, uint32_t now)
{   /* reads the whole (small) file the way the engine does: 1024, then the rest */
    (void)now; const uint32_t first = opened_size < 1024U ? opened_size : 1024U;
    produce_ok = read(ctx, 0, produced, first) && (opened_size <= 1024U || read(ctx, 1024, produced + 1024, opened_size - 1024U));
    return produce_ok;
}
bool sd_file_stream_session_active(void) { return session; }
bool sd_file_stream_state_allowed(void) { return allowed; }
static unsigned remounts_seen; static bool sd_retry_pending;
void sd_file_stream_media_remounted(void) { ++remounts_seen; xfer_active = false; ++ticket_now; }
'''

FILEX_MAIN = FRAME_PRINT + r'''
static void request(uint8_t op, uint16_t index, const char *name)
{
    pending = (sd_file_request_t){.op = op, .index = index, .name_len = (uint8_t)(name ? strlen(name) : 0)};
    if (name) strcpy(pending.name, name);
    has_pending = true; sd_file_service_turn();
}
static unsigned hash(const char *name)
{
    const unsigned before = answered; unsigned turns = 0;
    request(SD_FILE_OP_HASH, 0, name);
    while (answered == before && turns < 100) { sd_file_service_turn(); ++turns; }
    return turns;
}
int main(void)
{
    sd_file_service_turn(); assert(answered == 0);    /* nothing claimed */
    /* Enumeration: regular irec*.csv only, FileX order, END after the last. */
    for (uint16_t i = 0; i < 5; ++i) { request(SD_FILE_OP_LIST, i, NULL); hex("LIST", last, last_len); }
    request(SD_FILE_OP_LIST, 1, NULL); assert(last[1] == 0 && !memcmp(last + 12, "IREC_LAUNCH", 11)); /* backwards restarts */
    /* The writer's open file is flagged, in LIST and OPEN. */
    file.fx_file_name = "irec-1795392200-000.csv"; sd_file_open = 1;
    request(SD_FILE_OP_LIST, 2, NULL); assert(last[1] == 0 && last[9] == SD_FILE_ENTRY_OPEN);
    request(SD_FILE_OP_OPEN, 0, "irec-1795392200-000.csv"); assert(last[0] == 0xa5 && opened_growing && opened_size == 0);
    xfer_active = false; sd_file_open = 0;
    /* OPEN keeps one read handle; DATA reads whole sectors through it. */
    const uint32_t o = opens;
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(opened_size == 1300 && !opened_growing && handle_open);
    sd_file_service_turn(); assert(produce_ok); hex("DATA", produced, 1300);
    assert(opens == o + 1);
    /* The handle stays open for a transfer however long the gaps. */
    clock_ms += 10000; sd_file_service_turn(); assert(handle_open);
    /* HASH: refused while a transfer runs, then SHA-256 of the whole file in bounded steps. */
    request(SD_FILE_OP_HASH, 0, "irec-1795392000-000.csv"); assert(last[0] == 0xa7 && last[1] == SD_FILE_STATUS_BUSY && last_len == 39);
    xfer_active = false;
    printf("TURNS %u\n", hash("irec-1795392000-000.csv")); hex("HASH", last, last_len);
    hash("IREC_LAUNCH-1795392100-000.CSV"); hex("HASH", last, last_len);
    hash("irec-1795392200-000.csv"); hex("HASH", last, last_len);   /* empty file */
    /* A file the writer still appends: the hash covers the size its directory
     * entry had at the last media flush, and says so (growing flag). */
    file.fx_file_name = "IREC_LAUNCH-1795392100-000.CSV"; sd_file_open = 1;
    hash("IREC_LAUNCH-1795392100-000.CSV"); hex("GROWING", last, last_len);
    sd_file_open = 0;
    request(SD_FILE_OP_OPEN, 0, "irec-missing.csv"); assert(last[0] == 0xa5 && last[1] == SD_FILE_STATUS_NOT_FOUND);
    request(SD_FILE_OP_HASH, 0, "irec-missing.csv"); assert(last[0] == 0xa7 && last[1] == SD_FILE_STATUS_NOT_FOUND);
    /* I/O failure: the handle closes and the answer says IO_ERROR. */
    request(SD_FILE_OP_OPEN, 0, "IREC_LAUNCH-1795392100-000.CSV"); assert(handle_open);
    io_fail = 1; sd_file_service_turn(); assert(!produce_ok && !handle_open); xfer_active = false;
    request(SD_FILE_OP_LIST, 0, NULL); assert(last[1] == SD_FILE_STATUS_IO_ERROR);
    request(SD_FILE_OP_HASH, 0, "irec-1795392000-000.csv"); assert(last[1] == SD_FILE_STATUS_IO_ERROR);
    io_fail = 0;
    prepare_fail = 1; request(SD_FILE_OP_LIST, 0, NULL); assert(last[1] == SD_FILE_STATUS_IO_ERROR); prepare_fail = 0;
    /* A cancelled hash stops quietly. */
    request(SD_FILE_OP_HASH, 0, "irec-1795392000-000.csv"); const unsigned a = answered;
    ++ticket_now; sd_file_service_turn(); sd_file_service_turn(); assert(answered == a);
    /* Gate re-checked in the SD thread; the handle closes when not allowed or idle. */
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(handle_open); xfer_active = false;
    allowed = false; request(SD_FILE_OP_LIST, 0, NULL); assert(!handle_open); allowed = true;
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(handle_open);
    allowed = false; request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv");
    allowed = true; xfer_active = false;
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(handle_open); xfer_active = false;
    clock_ms += 3000; sd_file_service_turn(); assert(!handle_open);
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); session = false; sd_file_service_turn(); assert(!handle_open);
    session = true; xfer_active = false;
    have_area = false; request(SD_FILE_OP_LIST, 0, NULL); assert(last[1] == SD_FILE_STATUS_BAD_STATE);
    /* A remount mid-transfer and mid-hash: the dead handle is dropped (not
     * closed on the aborted mount), the hash stops, the stream layer is told. */
    have_area = true;
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(handle_open && xfer_active);
    const uint32_t closed = closes;
    sd_file_open = 1; sd_retry_pending = true;
    sd_pipeline_media_remounted();
    assert(!handle_open && !xfer_active && remounts_seen == 1 && closes == closed && !sd_file_open && !sd_retry_pending);
    sd_file_open = 0;
    request(SD_FILE_OP_HASH, 0, "IREC_LAUNCH-1795392100-000.CSV");
    const unsigned before_remount = answered;
    sd_pipeline_media_remounted(); sd_file_service_turn(); sd_file_service_turn();
    assert(answered == before_remount && !handle_open);
    /* The next OPEN on the new mount opens a fresh handle. */
    const uint32_t reopened = opens;
    request(SD_FILE_OP_OPEN, 0, "irec-1795392000-000.csv"); assert(handle_open && opens == reopened + 1);
    xfer_active = false;
    sd_pipeline_wake(); assert(line.full == 0); sd_prepared = true; sd_pipeline_wake(); assert(line.full == 1);
    puts("done");
}
'''


class FilexSideTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        start = src.index('#ifdef FC_SD_SERVICE\n/* Network SD transfer service')
        end = src.index('#else\n#define sd_file_service_turn()')
        block = src[start + len('#ifdef FC_SD_SERVICE\n'):end]
        # The handle flags sit above the writer, which clears them on a remount.
        flags = 'static bool dl_open, dl_list_valid;\nstatic struct { bool active, growing; uint32_t ticket, pos, size; } dl_hash;\n'
        assert flags in src[:start]
        remount = src[src.index('void sd_pipeline_media_remounted(void)'):]
        remount = remount[:remount.index('\n}\n') + 3]
        block = flags + block + remount
        cls.block = block
        lc = configured_dependency_source(ROOT, 'sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')
        with tempfile.TemporaryDirectory() as tmp:
            exe = pathlib.Path(tmp) / 'check'
            subprocess.run(CFLAGS + ['-I', str(lc / 'bootloader/include'), '-x', 'c', '-', '-x', 'none',
                                     str(lc / 'bootloader/src/sha256.c'), '-o', str(exe)],
                           input=FILEX_STUBS + block + FILEX_MAIN, text=True, check=True)
            cls.out = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout

    @staticmethod
    def content(entry, size):
        return bytes((i * 31 + entry) & 0xff for i in range(size))

    def test_enumeration_filter(self):
        self.assertIn('done', self.out)
        lists = frames(self.out, 'LIST')
        self.assertEqual(lists[0], bytes([0xa1, 0, 0, 0, 0]) + (1300).to_bytes(4, 'little') + bytes([0, 23, 0]) + b'irec-1795392000-000.csv')
        self.assertEqual(lists[1][12:], b'IREC_LAUNCH-1795392100-000.CSV')
        self.assertEqual(lists[2][12:], b'irec-1795392200-000.csv')
        self.assertEqual(lists[3], bytes([0xa1, 6, 0, 3, 0] + [0] * 7))  # END
        self.assertEqual(lists[4][:2], bytes([0xa1, 6]))

    def test_data_reads_and_hashes_match_the_card(self):
        self.assertEqual(frames(self.out, 'DATA')[0], self.content(2, 1300))
        hashes = frames(self.out, 'HASH')
        for answer, (entry, size) in zip(hashes, ((2, 1300), (5, 512), (8, 0))):
            self.assertEqual(answer[:3], bytes([0xa7, 0, 0]))
            self.assertEqual(int.from_bytes(answer[3:7], 'little'), size)
            self.assertEqual(answer[7:], hashlib.sha256(self.content(entry, size)).digest())
        growing = frames(self.out, 'GROWING')[0]
        self.assertEqual(growing[:3], bytes([0xa7, 0, 0x04]))
        self.assertEqual(int.from_bytes(growing[3:7], 'little'), 512)
        self.assertEqual(growing[7:], hashlib.sha256(self.content(5, 512)).digest())
        turns = int(next(l.split()[1] for l in self.out.splitlines() if l.startswith('TURNS ')))
        self.assertEqual(turns, 0, '1300 bytes hash within the first turn (8 sectors per turn)')

    def test_read_only_and_wired_into_the_sd_thread(self):
        self.assertNotIn('fx_file_write', self.block)
        self.assertNotIn('fx_file_create', self.block)
        self.assertNotIn('FX_OPEN_FOR_WRITE', self.block)
        self.assertNotIn('fx_media_close', self.block)
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        task = src[src.index('void sd_pipeline_task()'):]
        self.assertIn('sd_file_service_turn();\n    }\n    sd_pipeline_shutdown(surprise);\n    sd_file_service_forever();', task)
        self.assertIn('sd_file_service_forever();\n        return;', task)
        shutdown = src[src.index('static inline void sd_pipeline_shutdown'):src.index('static inline int sd_pipeline_init(')]
        self.assertRegex(shutdown, r'#ifndef FC_SD_SERVICE\n\s+st = fx_media_close')
        # With the service the volume stays mounted: the directory entry that
        # fx_file_close() caches must be flushed again before a power-off.
        self.assertRegex(shutdown, r'fx_file_close\(&file\)[\s\S]*#else[\s\S]*fx_media_flush\(&sdio_disk\)[\s\S]*#endif')
        tel = (ROOT / 'Core/Src/telemetry.c').read_text()
        self.assertIn('sd_file_stream_init(r)', tel)
        self.assertIn('sd_file_stream_poll();', tel)
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('option(FC_SD_SERVICE', cmake)
        # Both images: the ground-test image is no longer excluded.
        self.assertNotIn('FC_SD_SERVICE AND INTEGRATED_GROUND_TEST', cmake)


if __name__ == '__main__':
    unittest.main()

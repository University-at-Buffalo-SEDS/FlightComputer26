"""FC SD block-read service (P2P port 4511): wire, gating, session and FileX side.

Runs the production sources on the host: Core/Inc/sd_file_wire.h directly,
Core/Src/sd_file_stream.c with SDK/RTOS stubs, and the FC_SD_SERVICE block of
Core/Src/sdpipeline.c against a small FileX model."""
import pathlib
import re
import subprocess
import tempfile
import unittest
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
CFLAGS = ['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
          '-Wno-unused-parameter', '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc')]


def run_c(code):
    with tempfile.TemporaryDirectory() as tmp:
        exe = pathlib.Path(tmp) / 'check'
        subprocess.run(CFLAGS + ['-x', 'c', '-', '-o', str(exe)], input=code, text=True, check=True)
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
static uint8_t read_req(uint32_t off, uint8_t dir, const char *name, int len_adjust)
{
    uint8_t p[200]; size_t n = strlen(name);
    p[0] = 0x22; sd_file_put_u32(p + 1, off); p[5] = dir; p[6] = (uint8_t)n; memcpy(p + 7, name, n);
    return parse(p, 7 + n + len_adjust);
}
int main(void)
{
    const uint8_t cancel[] = {0x23}, cancel_long[] = {0x23, 0};
    assert(parse(cancel, 1) == SD_FILE_STATUS_OK && parse(cancel_long, 2) == SD_FILE_STATUS_BAD_MESSAGE);
    const uint8_t list[] = {0x21, 0, 0x34, 0x12}, list_dir1[] = {0x21, 1, 0, 0}, list_short[] = {0x21, 0, 0};
    sd_file_request_t r;
    assert(sd_file_parse_request(list, 4, &r) == 0 && r.op == 0x21 && r.index == 0x1234 && r.dir == 0);
    assert(parse(list_dir1, 4) == 1 && parse(list_short, 3) == 1);
    const uint8_t unknown[] = {0x24}, empty[] = {0};
    assert(parse(unknown, 1) == 1 && parse(empty, 0) == 1 && parse(NULL, 1) == 1);
    assert(parse(list, 118) == 1);                       /* over the frame bound */
    assert(read_req(0x01020304u, 0, "irec-1795392000-000.csv", 0) == 0);
    assert(read_req(0, 0, "IREC_LAUNCH-5-001.CSV", 0) == 0);  /* case-insensitive */
    assert(read_req(0, 1, "irec-1-000.csv", 0) == 1);    /* only dir 0 */
    assert(read_req(0, 2, "irec-1-000.csv", 0) == 1);
    assert(read_req(0, 0, "irec-1-000.csv", 1) == 1);    /* length mismatch */
    assert(read_req(0, 0, "irec-1-000.csv", -1) == 1);
    assert(read_req(0, 0, "IGT/IMU0.CSV", 0) == 1);      /* separator */
    assert(read_req(0, 0, "../irec.csv", 0) == 1);
    assert(read_req(0, 0, "IMU0.CSV", 0) == 1);          /* not a log */
    assert(read_req(0, 0, "irec-1-000.txt", 0) == 1);
    assert(read_req(0, 0, "irec.csv", 0) == 0);          /* shortest log name */
    assert(read_req(0, 0, "irec csv.csv", 0) == 1);
    char longname[65]; memset(longname, 'a', 64); memcpy(longname, "irec", 4); memcpy(longname + 60, ".csv", 4); longname[64] = 0;
    assert(read_req(0, 0, longname, 0) == 1);            /* 64 > SD_FILE_NAME_MAX */
    longname[59] = 0; memcpy(longname + 55, ".csv", 5);
    assert(read_req(0, 0, longname, 0) == 0);
    uint8_t p[8] = {0x22, 0, 0, 0, 0, 0, 0, 'x'};
    assert(parse(p, 7) == 1 && parse(p, 8) == 1);        /* zero-length name */
    assert(sd_file_parse_request(cancel, 1, &r) == 0 && sd_file_parse_request(p, 7, &r) == 1 && r.op == 0x23);
    puts("ok");
}
''')
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

    def test_frame_encoding_with_crc(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
''' + FRAME_PRINT + r'''
int main(void)
{
    uint8_t f[SD_FILE_FRAME_MAX], data[100];
    for (unsigned i = 0; i < 100; ++i) data[i] = (uint8_t)(i * 7);
    hex("D", f, sd_file_encode_data(f, 0, SD_FILE_DATA_LAST | SD_FILE_DATA_CRC, 0x01020304u, 0x0a0b0c0du, data, 100, 0xdeadbeefu));
    hex("D", f, sd_file_encode_data(f, 0, 0, 7, 9, data, 5, 0xdeadbeefu));
    hex("D", f, sd_file_encode_data(f, 6, SD_FILE_DATA_LAST, 0, 0, NULL, 0, 0));
    printf("Z %zu\n", sd_file_encode_data(f, 0, 0, 0, 0, data, 101, 0));
    hex("L", f, sd_file_encode_list(f, 0, 0x0102, 0x11223344u, SD_FILE_ENTRY_OPEN, "irec-1-000.csv", 14));
    hex("L", f, sd_file_encode_list(f, 6, 3, 99, 1, "ignored", 7));
}
''')
        d = frames(out, 'D')
        self.assertEqual(len(d[0]), 115)
        self.assertEqual(d[0][:11], bytes([0xa2, 0, 3, 4, 3, 2, 1, 0x0d, 0x0c, 0x0b, 0x0a]))
        self.assertEqual(d[0][11:111], bytes((i * 7) & 0xff for i in range(100)))
        self.assertEqual(d[0][111:], (0xdeadbeef).to_bytes(4, 'little'))
        self.assertEqual(len(d[1]), 16)  # no CRC without the flag
        self.assertEqual(d[2], bytes([0xa2, 6, 1] + [0] * 8))
        self.assertIn('Z 0', out)
        l = frames(out, 'L')
        self.assertEqual(l[0], bytes([0xa1, 0, 0, 2, 1, 0x44, 0x33, 0x22, 0x11, 1, 14, 0]) + b'irec-1-000.csv')
        self.assertEqual(l[1], bytes([0xa1, 6, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0]))

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
typedef struct { char opaque[400]; } FX_FILE;
static uint32_t primask, tick, wakes, resets, queued, allocs, frees, send_fail;
static uint8_t local_state; static bool net_known; static uint8_t net_state;
static uint8_t sent[64][SD_FILE_FRAME_MAX]; static size_t sent_len[64]; static unsigned nsent;
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
{ (void)r; assert(id == 7 && n <= SD_FILE_FRAME_MAX && !primask); if (send_fail) return SEDS_ERR;
  memcpy(sent[nsent], p, n); sent_len[nsent++] = n; return SEDS_OK; }
SedsResult seds_router_reset_p2p_stream(SedsRouter *r, uint32_t id) { (void)r; assert(id == 7); ++resets; return SEDS_OK; }
int32_t seds_router_has_queued_work(SedsRouter *r) { (void)r; return (int32_t)queued; }
SedsResult seds_router_bind_p2p_stream_port(SedsRouter *r, uint16_t port, SedsP2pStreamHandlerFn cb, void *u)
{ (void)r; (void)u; assert(port == 4511); handler = cb; return SEDS_OK; }
'''

STREAM_MAIN = FRAME_PRINT + r'''
static SedsResult ev(uint8_t kind, const char *peer, const uint8_t *p, size_t n)
{
    SedsP2pStreamEventView e = {.kind = kind, .stream_id = 7, .peer_stream_id = 9, .peer_hostname = peer,
        .peer_hostname_len = strlen(peer), .peer_address = 0x1234, .local_port = 4511, .peer_port = 4511,
        .payload = p, .payload_len = n};
    return handler(&e, NULL);
}
static const uint8_t READ[] = {0x22, 0x10, 0, 0, 0, 0, 14, 'i','r','e','c','-','1','-','0','0','0','.','c','s','v'};
static const uint8_t LIST[] = {0x21, 0, 0, 0}, CANCEL[] = {0x23};
static void pump(void) { for (unsigned i = 0; i < 400; ++i) { tick += 1; sd_file_stream_poll(); } }
int main(void)
{
    assert(sd_file_stream_init(&router) == SEDS_OK && handler);
    local_state = 9;                                   /* Landed */
    /* Peer check: GS and RRLS only. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "VB", NULL, 0) == SEDS_BAD_ARG && allocs == 0);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "RRLS", NULL, 0) == SEDS_OK && allocs == 1);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 1); /* replaces it */
    assert(sd_file_stream_session_active());
    /* READ: the router callback only queues and wakes the SD thread. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ, sizeof READ) == SEDS_OK && wakes == 1 && nsent == 0);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && nsent == 1);
    hex("BUSY", sent[0], sent_len[0]); nsent = 0;
    sd_file_request_t rq; uint32_t ticket;
    assert(sd_file_stream_claim(&rq, &ticket) && rq.op == 0x22 && rq.offset == 16 && !strcmp(rq.name, "irec-1-000.csv"));
    assert(!sd_file_stream_claim(&rq, &ticket));
    uint8_t *block = sd_file_stream_block();
    assert(block && sd_file_stream_handle() == block + 512);
    for (unsigned i = 0; i < 512; ++i) block[i] = (uint8_t)(i ^ 0x5a);
    sd_file_result_t res = {.status = 0, .data_start = 16, .data_len = 250, .file_size = 4000, .offset = 16, .crc = 0x11223344u};
    sd_file_stream_finish(ticket, &res);
    /* Pacing: >= 2 ms between frames, >= 25 ms while the router is busy. */
    tick = 1000; sd_file_stream_poll(); assert(nsent == 1);
    sd_file_stream_poll(); tick += 1; sd_file_stream_poll(); assert(nsent == 1);
    tick += 1; queued = 1; sd_file_stream_poll(); assert(nsent == 1);
    tick += 22; sd_file_stream_poll(); assert(nsent == 1);
    tick += 1; sd_file_stream_poll(); assert(nsent == 2);
    queued = 0; send_fail = 1; tick += 2; sd_file_stream_poll(); assert(nsent == 2);  /* retried */
    send_fail = 0; sd_file_stream_poll(); assert(nsent == 3);
    pump(); assert(nsent == 3);
    for (unsigned i = 0; i < nsent; ++i) hex("DATA", sent[i], sent_len[i]);
    nsent = 0;
    /* CANCEL stales a busy answer. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ, sizeof READ) == SEDS_OK);
    assert(sd_file_stream_claim(&rq, &ticket));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", CANCEL, 1) == SEDS_OK);
    sd_file_stream_finish(ticket, &res); pump();
    assert(nsent == 1 && sent_len[0] == 2 && sent[0][0] == 0xa3 && sent[0][1] == 0); nsent = 0;
    /* Bad requests answer at once with BAD_MESSAGE. */
    const uint8_t bad[] = {0x21, 1, 0, 0};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", bad, 4) == SEDS_OK && nsent == 1);
    hex("BAD", sent[0], sent_len[0]); nsent = 0;
    /* Error result: one LAST frame, no CRC. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ, sizeof READ) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_result_t eof = {.status = SD_FILE_STATUS_END, .file_size = 16, .offset = 16};
    sd_file_stream_finish(ticket, &eof); pump();
    hex("EOF", sent[0], sent_len[0]); nsent = 0;
    /* LIST answer carries the name from the block. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    memcpy(sd_file_stream_block(), "irec-1-000.csv", 14);
    sd_file_result_t entry = {.status = 0, .flags = 1, .name_len = 14, .file_size = 300};
    sd_file_stream_finish(ticket, &entry); pump();
    hex("LIST", sent[0], sent_len[0]); nsent = 0;
    /* Leaving the allowed states: a request is refused, the session is reset. */
    local_state = 2;                                   /* Armed */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && nsent == 1);
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
    /* A busy SD turn finishing after its session was reset publishes nothing. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ, sizeof READ) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    net_state = 6;                                     /* network Armed */
    sd_file_stream_poll(); assert(resets == 2);
    sd_file_stream_finish(ticket, &res);
    net_state = 1; assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK);
    pump(); assert(nsent == 0);
    assert(ev(SEDS_P2P_STREAM_CLOSED, "GS", NULL, 0) == SEDS_OK && !sd_file_stream_session_active());
    sd_file_stream_poll(); assert(frees == 2);
    puts("done");
}
'''


class StreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = strip_includes((ROOT / 'Core/Src/sd_file_stream.c').read_text())
        cls.out = run_c(STREAM_STUBS + src + STREAM_MAIN)

    def test_session_and_pacing(self):
        self.assertIn('done', self.out)
        self.assertEqual(frames(self.out, 'BUSY')[0], bytes([0xa1, 3] + [0] * 10))
        self.assertEqual(frames(self.out, 'BAD')[0], bytes([0xa1, 1] + [0] * 10))
        self.assertEqual(frames(self.out, 'STATE')[0], bytes([0xa1, 2] + [0] * 10))
        self.assertEqual(frames(self.out, 'EOF')[0], bytes([0xa2, 6, 1, 16, 0, 0, 0, 16, 0, 0, 0]))
        self.assertEqual(frames(self.out, 'LIST')[0],
                         bytes([0xa1, 0, 0, 0, 0, 44, 1, 0, 0, 1, 14, 0]) + b'irec-1-000.csv')

    def test_read_answer_is_split_with_crc_on_the_last_frame(self):
        data = frames(self.out, 'DATA')
        self.assertEqual([len(f) for f in data], [111, 111, 11 + 50 + 4])
        self.assertEqual([f[2] for f in data], [0, 0, 0x03])
        offsets = [int.from_bytes(f[3:7], 'little') for f in data]
        self.assertEqual(offsets, [16, 116, 216])
        self.assertTrue(all(int.from_bytes(f[7:11], 'little') == 4000 for f in data))
        payload = data[0][11:] + data[1][11:] + data[2][11:-4]
        self.assertEqual(payload, bytes((i ^ 0x5a) & 0xff for i in range(16, 266)))
        self.assertEqual(data[2][-4:], (0x11223344).to_bytes(4, 'little'))


FILEX_STUBS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "sd_file_stream.h"
typedef unsigned UINT; typedef unsigned long ULONG; typedef unsigned long long ULONG64; typedef char CHAR;
#define FX_SUCCESS 0U
#define FX_NOT_FOUND 4U
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
static uint8_t block[512 + sizeof(FX_FILE)];
static bool have_block = true, allowed = true, session = true, handle_open;
static sd_file_request_t pending; static bool has_pending; static sd_file_result_t last; static unsigned finished;
bool sd_file_stream_claim(sd_file_request_t *r, uint32_t *t) { if (!has_pending) return false; *r = pending; *t = 1; has_pending = false; return true; }
uint8_t *sd_file_stream_block(void) { return have_block ? block : NULL; }
void *sd_file_stream_handle(void) { return have_block ? block + 512 : NULL; }
void sd_file_stream_handle_open(bool open) { handle_open = open; }
void sd_file_stream_finish(uint32_t t, const sd_file_result_t *r) { (void)t; last = *r; ++finished; }
bool sd_file_stream_session_active(void) { return session; }
bool sd_file_stream_state_allowed(void) { return allowed; }
'''

FILEX_MAIN = FRAME_PRINT + r'''
static sd_file_result_t list(uint16_t index)
{ pending = (sd_file_request_t){.op = SD_FILE_OP_LIST, .index = index}; has_pending = true; sd_file_service_turn(); return last; }
static sd_file_result_t rd(const char *name, uint32_t offset)
{
    pending = (sd_file_request_t){.op = SD_FILE_OP_READ, .offset = offset, .name_len = (uint8_t)strlen(name)};
    strcpy(pending.name, name); has_pending = true; sd_file_service_turn(); return last;
}
int main(void)
{
    sd_file_service_turn(); assert(finished == 0);    /* nothing claimed */
    /* Enumeration: regular irec*.csv only, FileX order, END after the last. */
    for (uint16_t i = 0; i < 5; ++i) {
        sd_file_result_t r = list(i);
        printf("LIST %u %u %lu %u %.*s\n", i, r.status, (unsigned long)r.file_size, r.flags, r.name_len, (char *)block);
    }
    sd_file_result_t r = list(1); assert(r.status == 0 && !memcmp(block, "IREC_LAUNCH", 11)); /* backwards restarts */
    /* The writer's open file is flagged. */
    file.fx_file_name = "irec-1795392200-000.csv"; sd_file_open = 1;
    r = list(2); assert(r.status == 0 && r.flags == SD_FILE_ENTRY_OPEN);
    r = rd("irec-1795392200-000.csv", 0); assert(r.status == SD_FILE_STATUS_END && r.flags == SD_FILE_ENTRY_OPEN);
    sd_file_open = 0;
    /* Reads: block-aligned source, answer from the requested offset. */
    const uint32_t o = opens;
    r = rd("irec-1795392000-000.csv", 0); assert(r.status == 0 && r.data_start == 0 && r.data_len == 512 && r.file_size == 1300);
    hex("BLK", block + r.data_start, r.data_len); printf("CRC %08x\n", r.crc);
    r = rd("irec-1795392000-000.csv", 1100); assert(r.status == 0 && r.offset == 1100 && r.data_start == 76 && r.data_len == 200);
    hex("BLK", block + r.data_start, r.data_len); printf("CRC %08x\n", r.crc);
    assert(opens == o + 1);                            /* handle reused */
    r = rd("irec-1795392000-000.csv", 1299); assert(r.status == 0 && r.data_len == 1);
    r = rd("irec-1795392000-000.csv", 1300); assert(r.status == SD_FILE_STATUS_END && r.data_len == 0);
    r = rd("irec-1795392000-000.csv", 1301); assert(r.status == SD_FILE_STATUS_BAD_OFFSET && r.file_size == 1300);
    r = rd("irec-1795392000-000.csv", 0xffffffffu); assert(r.status == SD_FILE_STATUS_BAD_OFFSET);
    r = rd("irec-missing.csv", 0); assert(r.status == SD_FILE_STATUS_NOT_FOUND);
    r = rd("IREC_LAUNCH-1795392100-000.CSV", 0); assert(r.status == 0 && r.data_len == 512 && opens == o + 2);
    /* I/O failure closes the handle and reports IO_ERROR. */
    io_fail = 1; r = rd("IREC_LAUNCH-1795392100-000.CSV", 0); assert(r.status == SD_FILE_STATUS_IO_ERROR && !handle_open);
    r = list(0); assert(r.status == SD_FILE_STATUS_IO_ERROR); io_fail = 0;
    prepare_fail = 1; r = list(0); assert(r.status == SD_FILE_STATUS_IO_ERROR); prepare_fail = 0;
    /* Gate re-checked in the SD thread; the handle closes when not allowed or idle. */
    r = rd("irec-1795392000-000.csv", 0); assert(handle_open);
    allowed = false; r = list(0); assert(r.status == SD_FILE_STATUS_BAD_STATE && !handle_open); allowed = true;
    r = rd("irec-1795392000-000.csv", 0); assert(handle_open);
    clock_ms += 3000; sd_file_service_turn(); assert(!handle_open);
    r = rd("irec-1795392000-000.csv", 0); session = false; sd_file_service_turn(); assert(!handle_open); session = true;
    have_block = false; r = list(0); assert(r.status == SD_FILE_STATUS_BAD_STATE);
    sd_pipeline_wake(); assert(line.full == 0); sd_prepared = true; sd_pipeline_wake(); assert(line.full == 1);
    puts("done");
}
'''


class FilexSideTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        start = src.index('#ifdef FC_SD_SERVICE\n/* Network SD read service')
        end = src.index('#else\n#define sd_file_service_turn()')
        block = src[start + len('#ifdef FC_SD_SERVICE\n'):end]
        # The handle flags sit above the writer, which clears them on a remount.
        flags = 'static bool dl_open, dl_list_valid;\n'
        assert flags in src[:start]
        block = flags + block
        cls.block = block
        cls.out = run_c(FILEX_STUBS + crc_source() + block + FILEX_MAIN)

    def test_enumeration_filter(self):
        self.assertIn('done', self.out)
        lines = [l.split() for l in self.out.splitlines() if l.startswith('LIST ')]
        self.assertEqual(lines[0], ['LIST', '0', '0', '1300', '0', 'irec-1795392000-000.csv'])
        self.assertEqual(lines[1], ['LIST', '1', '0', '512', '0', 'IREC_LAUNCH-1795392100-000.CSV'])
        self.assertEqual(lines[2], ['LIST', '2', '0', '0', '0', 'irec-1795392200-000.csv'])
        self.assertEqual(lines[3][:3], ['LIST', '3', '6'])  # END
        self.assertEqual(lines[4][:3], ['LIST', '4', '6'])

    def test_crc_matches_zlib(self):
        blocks = frames(self.out, 'BLK')
        crcs = [int(l.split()[1], 16) for l in self.out.splitlines() if l.startswith('CRC ')]
        self.assertEqual(blocks[0], bytes((i * 31 + 2) & 0xff for i in range(512)))
        self.assertEqual(blocks[1], bytes((i * 31 + 2) & 0xff for i in range(1100, 1300)))
        for data, crc in zip(blocks, crcs):
            self.assertEqual(crc, zlib.crc32(data))

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
        self.assertIn('FC_SD_SERVICE AND INTEGRATED_GROUND_TEST', cmake)


if __name__ == '__main__':
    unittest.main()

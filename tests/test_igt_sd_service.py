"""SD read service and IGT CSV replay together (ground-test image).

The ground-test image (-DINTEGRATED_GROUND_TEST=ON) carries the SD transfer
service v2 (P2P port 4511) too. Both use FileX from the SD thread only:
igt_sd_service() and sd_file_service_turn() are called from the same loop in
sd_pipeline_task(). This compiles, in one program, the production
ground_test.c (its FileX port included), the FC_SD_SERVICE block and
sd_pipeline_media_remounted() of sdpipeline.c, and a FileX model with two
cards. The model checks every call: only the SD thread enters FileX, the
replay only uses its own handle and the service only its own, no handle
outlives an igt_sd_service() call, and no handle of an aborted mount is read
or closed. Scenarios: validation and listing/reading/hashing /IGT and the
logs on the pad; the replay in flight states with the service refused; a
remount at Ready, during validation and mid-replay; reads after landing.
"""
import hashlib
import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
from native_sdk import configured_dependency_source  # noqa: E402
from test_sd_file_service import crc_source  # noqa: E402

CC = ['cc', '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
      '-Wno-unused-parameter', '-fsanitize=address,undefined', '-fno-sanitize-recover=all']

ENGINE = [str(ROOT / 'Core/Src/sd_xfer.c'), str(ROOT / 'Core/Src/sd_xfer_codec.c')]

# ground_test.c includes this as its host port (IGT_HOST_TEST); with
# IGT_HOST_FILEX its own FileX port is compiled against the model below.
PORT = r'''
#include <assert.h>
#include <ctype.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "fcconfig.h"
#define IGT_PUB_PERIOD_MS 200U
#define IGT_SD_PERIOD_MS 50U
float g_launch_site[3] = {LAUNCH_SITE_LAT_DEFAULT, LAUNCH_SITE_LON_DEFAULT, LAUNCH_SITE_SEA_DEFAULT};
char igt_log[64][96];
unsigned igt_logs;
static bool igt_port_sd_ready(void) { return true; }
static void igt_port_message(const char *text) { if (igt_logs < 64) snprintf(igt_log[igt_logs++], 96, "%s", text); }
static void igt_port_sample(const float v[8], bool radio, bool sd) { (void)v; (void)radio; (void)sd; }

/* ---- FileX model ---- */
typedef unsigned UINT; typedef unsigned long ULONG; typedef unsigned long long ULONG64; typedef char CHAR;
#define FX_SUCCESS 0U
#define FX_NOT_FOUND 4U
#define FX_END_OF_FILE 0x09U
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
#define TX_SEMAPHORE_ERROR 0x0CU
typedef struct { CHAR *fx_file_name; CHAR fx_file_name_buffer[FX_MAX_LONG_NAME_LEN]; ULONG64 fx_file_current_file_size;
                 int idx, gen, open; ULONG pos; } FX_FILE;
typedef struct { struct { ULONG fx_path_current_entry; } fx_media_default_path; } FX_MEDIA;
static FX_MEDIA sdio_disk;

struct mfile { char name[256]; int igt; unsigned char *data; ULONG size; };
struct card { struct mfile f[64]; unsigned n; };
static struct card cards[2];
static int cur_card, media_gen, swap_pending, swap_countdown, remounts;
/* Who is calling: the SD thread's replay step or service turn, or another thread. */
enum { CTX_OTHER, CTX_IGT, CTX_SERVICE, CTX_WRITER };
static int ctx = CTX_OTHER, in_fx;
static unsigned calls[4], open_handles;
static FX_FILE *igt_handle_seen;

static void load_card(struct card *c, const char *dir)
{
    for (int igt = 0; igt < 2; ++igt) {
        char path[600]; snprintf(path, sizeof path, "%s%s", dir, igt ? "/IGT" : "");
        DIR *d = opendir(path); if (!d) continue;
        struct dirent *e;
        while ((e = readdir(d))) {
            if (e->d_name[0] == '.' || (!igt && !strcmp(e->d_name, "IGT"))) continue;
            char full[900]; snprintf(full, sizeof full, "%s/%s", path, e->d_name);
            FILE *fp = fopen(full, "rb"); assert(fp);
            struct mfile *m = &c->f[c->n++];
            snprintf(m->name, sizeof m->name, "%s", e->d_name); m->igt = igt;
            fseek(fp, 0, SEEK_END); m->size = (ULONG)ftell(fp); fseek(fp, 0, SEEK_SET);
            m->data = malloc(m->size + 1); assert(fread(m->data, 1, m->size, fp) == m->size); fclose(fp);
        }
        closedir(d);
    }
    /* FileX lists in directory order; make it deterministic. */
    for (unsigned i = 0; i < c->n; ++i) for (unsigned j = i + 1; j < c->n; ++j)
        if (strcmp(c->f[j].name, c->f[i].name) < 0) { struct mfile t = c->f[i]; c->f[i] = c->f[j]; c->f[j] = t; }
}
static void fx_enter(void) { assert(ctx != CTX_OTHER && "FileX entered outside the SD thread"); assert(!in_fx); in_fx = 1; ++calls[ctx]; }
static void fx_leave(void) { in_fx = 0; }
static int lookup(const char *path, int *igt_dir_present)
{
    int igt = !strncasecmp(path, "/IGT/", 5);
    const char *leaf = igt ? path + 5 : path;
    struct card *c = &cards[cur_card];
    if (igt_dir_present) { *igt_dir_present = 0; for (unsigned i = 0; i < c->n; ++i) if (c->f[i].igt) *igt_dir_present = 1; }
    for (unsigned i = 0; i < c->n; ++i) if (c->f[i].igt == igt && !strcasecmp(c->f[i].name, leaf)) return (int)i;
    return -1;
}
static void check_handle(FX_FILE *f)
{
    assert(f->open && "handle not open");
    assert(f->gen == media_gen && "handle of an aborted mount");
    if (ctx == CTX_IGT) { assert(f == igt_handle_seen && "the replay used another handle"); }
    if (ctx == CTX_SERVICE) { assert(f != igt_handle_seen && "the service touched the replay's handle"); }
}
static UINT fx_file_open(FX_MEDIA *m, FX_FILE *f, CHAR *name, UINT mode)
{
    (void)m; fx_enter(); assert(mode == FX_OPEN_FOR_READ);
    if (ctx == CTX_IGT) { if (!igt_handle_seen) igt_handle_seen = f; assert(f == igt_handle_seen); }
    if (f->open && f->gen == media_gen) assert(!"opened an open handle");
    int present = 0; const int i = lookup(name, &present);
    if (i < 0) { fx_leave(); return !strncasecmp(name, "/IGT/", 5) && !present ? FX_INVALID_PATH : FX_NOT_FOUND; }
    struct mfile *mf = &cards[cur_card].f[i];
    strcpy(f->fx_file_name_buffer, mf->name); f->fx_file_name = f->fx_file_name_buffer;
    f->fx_file_current_file_size = mf->size; f->idx = i; f->gen = media_gen; f->open = 1; f->pos = 0; ++open_handles;
    fx_leave(); return FX_SUCCESS;
}
static UINT fx_file_close(FX_FILE *f) { fx_enter(); check_handle(f); f->open = 0; --open_handles; fx_leave(); return FX_SUCCESS; }
static UINT fx_file_seek(FX_FILE *f, ULONG at) { fx_enter(); check_handle(f); f->pos = at; fx_leave(); return FX_SUCCESS; }
static UINT fx_file_read(FX_FILE *f, void *buf, ULONG want, ULONG *actual)
{
    fx_enter(); check_handle(f);
    struct mfile *mf = &cards[cur_card].f[f->idx];
    ULONG n = f->pos >= mf->size ? 0 : (f->pos + want > mf->size ? mf->size - f->pos : want);
    memcpy(buf, mf->data + f->pos, n); f->pos += n; *actual = n;
    fx_leave(); return n == 0 && want ? FX_END_OF_FILE : FX_SUCCESS;
}
static UINT fx_directory_next_full_entry_find(FX_MEDIA *m, CHAR *name, UINT *attr, ULONG *size,
    UINT *y, UINT *mo, UINT *d, UINT *h, UINT *mi, UINT *s)
{
    (void)y; (void)mo; (void)d; (void)h; (void)mi; (void)s;
    fx_enter();
    /* Root: the IGT directory entry first, then the root files. */
    struct card *c = &cards[cur_card];
    ULONG i = m->fx_media_default_path.fx_path_current_entry, k = 0;
    int has_igt = 0; for (unsigned j = 0; j < c->n; ++j) if (c->f[j].igt) has_igt = 1;
    if (has_igt) { if (i == 0) { strcpy(name, "IGT"); *attr = FX_DIRECTORY; *size = 0; m->fx_media_default_path.fx_path_current_entry = 1; fx_leave(); return FX_SUCCESS; } k = 1; }
    for (unsigned j = 0; j < c->n; ++j) {
        if (c->f[j].igt) continue;
        if (k++ == i) { strcpy(name, c->f[j].name); *attr = 0; *size = c->f[j].size; m->fx_media_default_path.fx_path_current_entry = i + 1; fx_leave(); return FX_SUCCESS; }
    }
    fx_leave(); return FX_NO_MORE_ENTRIES;
}
static unsigned info_calls;
static UINT fx_directory_information_get(FX_MEDIA *m, CHAR *path, UINT *attr, ULONG *size,
    UINT *y, UINT *mo, UINT *d, UINT *h, UINT *mi, UINT *s)
{
    (void)m; (void)y; (void)mo; (void)d; (void)h; (void)mi; (void)s;
    fx_enter(); ++info_calls;
    assert(path[0] == '/' && "absolute paths only: the default path never moves");
    int present = 0; const int i = lookup(path, &present);
    fx_leave();
    if (i < 0) return present ? FX_NOT_FOUND : FX_INVALID_PATH;
    *attr = 0; *size = cards[cur_card].f[i].size; return FX_SUCCESS;
}
static bool flight_sd_io_verified(void) { return true; }
void sd_pipeline_media_remounted(void);
/* resilient_storage.c: a card that left its slot is mounted fresh. */
static UINT flight_sd_prepare_io(void)
{
    assert(ctx != CTX_OTHER);
    /* Counted on the replay's own opens: the swap lands inside one. */
    if (swap_countdown && ctx == CTX_IGT && --swap_countdown == 0) swap_pending = 1;
    if (swap_pending) {
        swap_pending = 0; cur_card = !cur_card; ++media_gen; ++remounts;
        /* fx_media_abort: every handle of the old mount is dead. */
        open_handles = 0;
        sd_pipeline_media_remounted();
    }
    return FX_SUCCESS;
}
'''

DRIVER = r'''
#define SD_AVAILABLE 1
#include "sd_file_stream.h"
#include "launchcore/image.h"
#include "ground_test.c"

/* ---- writer and RTOS stubs used by the service block ---- */
static FX_FILE file;
static uint8_t sd_file_open;
static bool sd_prepared, sd_retry_pending;
static struct { unsigned full; } line;
static uint32_t clock_ms;
static uint32_t now_ms(void) { return clock_ms; }
static UINT tx_semaphore_put(unsigned *s) { ++*s; return 0; }
static UINT tx_semaphore_get(unsigned *s, ULONG t) { (void)s; (void)t; return 0; }
static void tx_thread_sleep(ULONG t) { (void)t; }

/* ---- sd_file_stream.c side ---- */
static uint8_t area[512 + sizeof(FX_FILE)]; static launchcore_sha256_ctx_t sha_ctx;
static bool allowed = true, session = true, handle_open, xfer_active;
static sd_file_request_t pending; static bool has_pending; static uint32_t ticket_now = 1;
static uint8_t last[SD_FILE_ANSWER_MAX]; static size_t last_len; static unsigned answered;
static uint32_t opened_size, xfer_pos; static uint8_t produced[65536];
bool sd_file_stream_claim(sd_file_request_t *r, uint32_t *t) { if (!has_pending) return false; *r = pending; *t = ticket_now; has_pending = false; return true; }
bool sd_file_stream_ticket_current(uint32_t t) { return t == ticket_now; }
uint8_t *sd_file_stream_block(void) { return area; }
void *sd_file_stream_handle(void) { return area + 512; }
void *sd_file_stream_scratch(void) { return &sha_ctx; }
void sd_file_stream_handle_open(bool open) { handle_open = open; }
void sd_file_stream_answer(uint32_t t, const uint8_t *f, size_t n) { if (t != ticket_now) return; memcpy(last, f, n); last_len = n; ++answered; }
void sd_file_stream_open(uint32_t t, const sd_xfer_open_t *o, uint32_t size, bool growing)
{ (void)o; (void)growing; opened_size = size; xfer_pos = 0; xfer_active = true; uint8_t a[SD_XFER_OPEN_ANSWER] = {0xa5}; sd_file_stream_answer(t, a, sizeof a); }
bool sd_file_stream_transfer_active(void) { return xfer_active; }
/* One sector per DATA message, like the engine's bounded step. */
bool sd_file_stream_produce(sd_xfer_read_fn read, void *c, uint32_t now)
{
    (void)now;
    const uint32_t n = opened_size - xfer_pos > 512U ? 512U : opened_size - xfer_pos;
    if (!read(c, xfer_pos, produced + xfer_pos, n)) { xfer_active = false; return false; }
    xfer_pos += n; if (xfer_pos == opened_size) xfer_active = false;
    return true;
}
bool sd_file_stream_session_active(void) { return session; }
bool sd_file_stream_state_allowed(void) { return allowed; }
void sd_file_stream_media_remounted(void) { xfer_active = false; ++ticket_now; }

/* ---- the service block of sdpipeline.c ---- */
@SERVICE@

/* ---- driver ---- */
static void hex(const char *tag, const uint8_t *p, size_t n)
{ printf("%s ", tag); for (size_t i = 0; i < n; ++i) printf("%02x", p[i]); printf("\n"); }
static void igt_turn(void)
{
    ctx = CTX_IGT; igt_sd_service(); ctx = CTX_OTHER;
    assert(!igt_handle_seen || !igt_handle_seen->open || igt_handle_seen->gen != media_gen);
    if (igt_handle_seen) assert(!igt_handle_seen->open && "a replay handle outlived its call");
}
static void service_turn(void) { ctx = CTX_SERVICE; sd_file_service_turn(); ctx = CTX_OTHER; }
/* One pass of sd_pipeline_task's loop: prepare, replay, service. */
static void sd_loop(void)
{
    ctx = CTX_WRITER; (void)flight_sd_prepare_io(); ctx = CTX_OTHER;
    igt_turn(); service_turn(); ++clock_ms;
}
static void request(uint8_t op, uint8_t dir, uint16_t index, const char *name)
{
    pending = (sd_file_request_t){.op = op, .dir = dir, .index = index, .name_len = (uint8_t)(name ? strlen(name) : 0)};
    if (name) strcpy(pending.name, name);
    has_pending = true;
    const unsigned before = answered;
    for (unsigned i = 0; i < 200 && answered == before; ++i) sd_loop();
    assert(answered > before);
}
static bool logged(const char *needle)
{
    for (unsigned i = 0; i < igt_logs; ++i) if (strstr(igt_log[i], needle)) return true;
    return false;
}
static unsigned mode_now(void) { return (igt_health_bits() >> 4) & 7U; }
static bool reason_now_ok(void) { return ((igt_health_bits() >> 12) & 15U) == IGT_OK; }
static void list_dir(uint8_t dir, const char *tag)
{
    for (uint16_t i = 0;; ++i) {
        request(SD_FILE_OP_LIST, dir, i, NULL);
        hex(tag, last, last_len);
        if (last[1] != SD_FILE_STATUS_OK) break;
    }
}
static void fetch(uint8_t dir, const char *name, const char *tag)
{
    request(SD_FILE_OP_OPEN, dir, 0, name);
    assert(last[0] == 0xa5 && last[1] == 0 && handle_open);
    while (xfer_active) sd_loop();       /* the replay steps between DATA messages */
    hex(tag, produced, opened_size);
}
static void hash(uint8_t dir, const char *name, const char *tag)
{
    request(SD_FILE_OP_HASH, dir, 0, name);
    hex(tag, last, last_len);
}
static const float rest_a[3] = {0.0f, 0.0f, 9.80665f};
static const float rest_g[3] = {0.1f, -0.2f, 0.05f};
static void enter(void)
{
    ctx = CTX_OTHER;                  /* the recovery task: no FileX */
    assert(igt_command(IGT_CMD_ENTER, true, false));
    assert(mode_now() == IGT_VALIDATING);
}

/* Pad: validation, Ready refills and the service on /IGT and the root. */
static void scenario_pad(void)
{
    enter();
    /* The first SD loop pass validates and primes, then answers LIST. */
    request(SD_FILE_OP_LIST, 1, 0, NULL);
    assert(last[1] == 0 && !memcmp(last + 12, "IMU0.CSV", 8));
    printf("mode=%u\n", mode_now());
    assert(mode_now() == IGT_READY && logged("IGT CSV ok"));
    list_dir(1, "IGTLIST");
    list_dir(0, "ROOTLIST");
    fetch(1, "BARO0.CSV", "BARO");
    fetch(0, "irec-1795392000-000.csv", "LOG");
    hash(1, "IMU0.CSV", "HASHIMU");
    hash(1, "gps0.csv", "HASHGPS");
    request(SD_FILE_OP_OPEN, 1, 0, "BARO7.CSV"); assert(last[1] == SD_FILE_STATUS_NOT_FOUND);
    assert(mode_now() == IGT_READY && calls[CTX_IGT] > 0 && calls[CTX_SERVICE] > 0);
    puts("pad ok");
}

/* Flight: the service is refused; the replay keeps feeding the estimator.
 * Landed: the replay is done; the service reads the card again. */
static void scenario_flight(void)
{
    enter(); sd_loop(); assert(mode_now() == IGT_READY);
    request(SD_FILE_OP_OPEN, 1, 0, "IMU0.CSV"); assert(handle_open);   /* left open on the pad */
    allowed = false;                  /* Postinit .. Descent */
    float seed[3];
    assert(igt_launch(1000, rest_a, rest_g, 0.2f, seed) && mode_now() == IGT_SIM);
    sd_loop(); assert(!handle_open);   /* the service closes its handle at once */
    xfer_active = false;              /* sd_file_stream_poll ends the session (telemetry thread) */
    const unsigned service_calls = calls[CTX_SERVICE];
    has_pending = true; pending = (sd_file_request_t){.op = SD_FILE_OP_LIST, .dir = 1};
    float a[3], g[3], b[3];
    for (uint32_t t = 0; t <= 4500; t += 10) {
        memcpy(a, rest_a, sizeof a); memcpy(g, rest_g, sizeof g);
        b[0] = 0.2f; b[1] = 101000.0f; b[2] = 20.0f;
        igt_override_accl(a, 1000 + t); igt_override_gyro(g, 1000 + t); igt_override_baro(b, 1000 + t);
        if (t > 20) {
            assert(fabsf(a[2] - (9.80665f + (float)t / 1000.0f)) < 1e-3f);
            assert(fabsf(b[0] - (float)t / 100.0f) < 1e-3f);
        }
        if (t % 50U == 0U) sd_loop();
    }
    assert(calls[CTX_SERVICE] == service_calls && "no FileX use by the service in flight states");
    assert(has_pending && "the request waits; the stream layer refuses it");
    has_pending = false;
    assert(reason_now_ok());
    igt_landed(); assert(mode_now() == IGT_DONE);
    allowed = true;                   /* Landed / Recovery */
    const unsigned igt_calls = calls[CTX_IGT];
    hash(1, "IMU0.CSV", "LANDEDIMU");
    fetch(0, "irec-1795392000-000.csv", "LANDEDLOG");
    assert(calls[CTX_IGT] == igt_calls && "Done: the replay reads nothing");
    puts("flight ok");
}
/* A remount at Ready (card B in): the transfer and the replay both reset;
 * card B validates; the next OPEN reads card B. */
static void scenario_remount_ready(void)
{
    enter(); sd_loop(); assert(mode_now() == IGT_READY && logged("IMU 501"));
    request(SD_FILE_OP_OPEN, 1, 0, "IMU0.CSV"); assert(handle_open && xfer_active);
    swap_pending = 1;
    sd_loop();
    assert(remounts == 1 && !handle_open && !xfer_active);
    assert(logged("IGT card remounted: validating again"));
    for (int i = 0; i < 3 && mode_now() != IGT_READY; ++i) sd_loop();
    assert(mode_now() == IGT_READY && logged("IMU 601"));
    list_dir(1, "BLIST");
    puts("remount ready ok");
}

/* A remount during validation, inside the replay's own open well into
 * IMU0.CSV: nothing is read at card A's offsets from card B; validation
 * starts over on card B. */
static void scenario_remount_validating(void)
{
    enter();
    swap_countdown = 6;
    sd_loop();
    assert(remounts == 1);
    for (int i = 0; i < 3 && mode_now() != IGT_READY; ++i) sd_loop();
    assert(mode_now() == IGT_READY && logged("IMU 601") && !logged("IMU 501"));
    puts("remount validating ok");
}

/* A remount mid-replay: the streams end (last rows held), no read of the
 * old mount's offsets on the new card, then the replay ends as on a read error. */
/* One 10 ms cycle; never a card B row (gx 2, pressure +1, lat +0.001). */
static void replay_step(uint32_t t, float a[3], float b[3])
{
    float g[3];
    memcpy(a, rest_a, 3 * sizeof(float)); igt_override_accl(a, 1000 + t);
    memcpy(g, rest_g, sizeof g); igt_override_gyro(g, 1000 + t);
    b[0] = 0.2f; b[1] = 99000.0f; b[2] = 20.0f; igt_override_baro(b, 1000 + t);
    if (mode_now() != IGT_SIM) return;
    assert(g[0] < 1.5f && "card B IMU row");
    assert(b[1] < 101000.5f && b[1] > 100999.5f && "card B barometer row");
    float fix[3];
    if (igt_gps_fix(fix, 1000 + t)) assert(fix[2] < 31.7585f && "card B GPS row");
}
static void scenario_remount_flight(void)
{
    enter(); sd_loop(); assert(mode_now() == IGT_READY);
    allowed = false;
    float seed[3], a[3], b[3];
    assert(igt_launch(1000, rest_a, rest_g, 0.2f, seed));
    uint32_t t = 0;
    for (; t <= 1000; t += 10) { replay_step(t, a, b); if (t % 50U == 0U) sd_loop(); }
    swap_countdown = 1;                 /* inside the next refill's open */
    for (; !remounts && t <= 3000; t += 10) { replay_step(t, a, b); sd_loop(); }
    sd_loop();
    assert(remounts == 1 && logged("IGT card remounted mid-replay"));
    for (unsigned i = 0; i < S_COUNT; ++i) assert(streams[i].eof);
    const unsigned igt_calls = calls[CTX_IGT];
    sd_loop(); assert(calls[CTX_IGT] == igt_calls && "ended streams read nothing");
    const uint32_t swapped = t;
    for (; t <= 45000 && mode_now() == IGT_SIM; t += 10) {
        replay_step(t, a, b);
        if (t < swapped + 200U) assert(fabsf(b[0] - 0.2f) > 1e-3f);   /* the held replay, not the real value */
    }
    assert(mode_now() == IGT_DONE);
    puts("remount flight ok");
}

int main(int argc, char **argv)
{
    load_card(&cards[0], argv[2]); load_card(&cards[1], argv[3]);
    if (!strcmp(argv[1], "pad")) scenario_pad();
    else if (!strcmp(argv[1], "flight")) scenario_flight();
    else if (!strcmp(argv[1], "remount_ready")) scenario_remount_ready();
    else if (!strcmp(argv[1], "remount_validating")) scenario_remount_validating();
    else if (!strcmp(argv[1], "remount_flight")) scenario_remount_flight();
    else return 2;
    assert(!in_fx);
    return 0;
}
'''

IMU_HEADER = 't_ms,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps'
BARO_HEADER = 't_ms,alt_m,pressure_pa,temp_c'
GPS_HEADER = 't_ms,lat_deg,lon_deg,alt_msl_m'


def dataset(end, b=0):
    """Card B (b=1) differs in every stream with the same row lengths, so
    card A's offsets land on card B's row starts: IMU gx 2 deg/s instead of
    1, pressure +1 Pa, GPS latitude +0.001 deg."""
    gx, pa, lat = 1 + b, 101000 + b, 31.758 + 0.001 * b
    return {
        'IMU0.CSV': '\n'.join([IMU_HEADER] + [f'{t},0,0,{9.80665 + t / 1000:.5f},{gx},0,0'
                                              for t in range(0, end + 1, 10)]) + '\n',
        'BARO0.CSV': '\n'.join([BARO_HEADER] + [f'{t},{t / 100:.2f},{pa},20'
                                                for t in range(0, end + 1, 20)]) + '\n',
        'GPS0.CSV': '\n'.join([GPS_HEADER] + [f'{t},{lat + t * 1e-7:.7f},-102.046,{900 + t / 100:.2f}'
                                              for t in range(0, end + 1, 100)]) + '\n',
    }


LOG = bytes((i * 7 + 3) & 0xff for i in range(1500))


def service_block():
    src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
    start = src.index('#ifdef FC_SD_SERVICE\n/* Network SD transfer service')
    end = src.index('#else\n#define sd_file_service_turn()')
    flags = 'static bool dl_open, dl_list_valid;\nstatic struct { bool active, growing; uint32_t ticket, pos, size; } dl_hash;\n'
    assert flags in src[:start]
    remount = src[src.index('void sd_pipeline_media_remounted(void)'):]
    remount = remount[:remount.index('\n}\n') + 3]
    return flags + src[start + len('#ifdef FC_SD_SERVICE\n'):end] + remount


def frames(out, tag):
    return [bytes.fromhex(l.split(' ', 1)[1]) for l in out.splitlines() if l.startswith(tag + ' ')]


class ServiceAndReplayCoexist(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        work = pathlib.Path(cls.tmp.name)
        (work / 'ground_test_host_port.h').write_text(PORT)
        (work / 'driver.c').write_text('#include <math.h>\n' + DRIVER.replace('@SERVICE@', service_block()))
        cls.cards = []
        for name, end, b in (('a', 5000, 0), ('b', 6000, 1)):
            card = work / name
            (card / 'IGT').mkdir(parents=True)
            for f, text in dataset(end, b).items():
                (card / 'IGT' / f).write_text(text)
            (card / 'irec-1795392000-000.csv').write_bytes(LOG)
            (card / 'notes.txt').write_bytes(b'x')
            cls.cards.append(card)
        lc = configured_dependency_source(ROOT, 'sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')
        cls.exe = work / 'coexist'
        subprocess.run(CC + ['-DIGT_HOST_TEST', '-DIGT_HOST_FILEX', '-DFC_GROUND_TEST=1', '-DFC_SD_SERVICE=1',
                             '-I', str(work), '-I', str(ROOT / 'Core/Inc'), '-I', str(ROOT / 'Core/Src'),
                             '-I', str(lc / 'bootloader/include'),
                             str(work / 'driver.c'), str(lc / 'bootloader/src/sha256.c'),
                             '-lm', '-o', str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_driver(self, scenario):
        r = subprocess.run([str(self.exe), scenario, str(self.cards[0]), str(self.cards[1])],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return r.stdout

    def igt(self, card, name):
        return (self.cards[card] / 'IGT' / name).read_bytes()

    def test_pad_validation_listing_reads_and_hashes(self):
        out = self.run_driver('pad')
        self.assertIn('pad ok', out)
        listing = frames(out, 'IGTLIST')
        names = [f[12:12 + f[10]].decode() for f in listing if f[1] == 0]
        self.assertEqual(names, ['IMU0.CSV', 'BARO0.CSV', 'GPS0.CSV'])
        for f, name in zip(listing, names):
            self.assertEqual(f[2], 1, 'directory 1 in the answer')
            self.assertEqual(int.from_bytes(f[5:9], 'little'), len(self.igt(0, name)))
            self.assertEqual(f[9], 0, 'never flagged open')
        self.assertEqual(listing[-1][:3], bytes([0xa1, 6, 1]))   # END
        roots = [f[12:12 + f[10]] for f in frames(out, 'ROOTLIST') if f[1] == 0]
        self.assertEqual(roots, [b'irec-1795392000-000.csv'], 'the root lists only logs')
        self.assertEqual(frames(out, 'BARO')[0], self.igt(0, 'BARO0.CSV'))
        self.assertEqual(frames(out, 'LOG')[0], LOG)
        for tag, name in (('HASHIMU', 'IMU0.CSV'), ('HASHGPS', 'GPS0.CSV')):
            answer = frames(out, tag)[0]
            self.assertEqual(answer[:3], bytes([0xa7, 0, 0]))
            self.assertEqual(int.from_bytes(answer[3:7], 'little'), len(self.igt(0, name)))
            self.assertEqual(answer[7:], hashlib.sha256(self.igt(0, name)).digest())

    def test_flight_refuses_the_service_and_landed_reads_again(self):
        out = self.run_driver('flight')
        self.assertIn('flight ok', out)
        self.assertEqual(frames(out, 'LANDEDIMU')[0][7:], hashlib.sha256(self.igt(0, 'IMU0.CSV')).digest())
        self.assertEqual(frames(out, 'LANDEDLOG')[0], LOG)

    def test_remount_at_ready_resets_both(self):
        out = self.run_driver('remount_ready')
        self.assertIn('remount ready ok', out)
        sizes = [int.from_bytes(f[5:9], 'little') for f in frames(out, 'BLIST') if f[1] == 0]
        self.assertEqual(sizes, [len(self.igt(1, n)) for n in ('IMU0.CSV', 'BARO0.CSV', 'GPS0.CSV')])

    def test_remount_during_validation_starts_over(self):
        self.assertIn('remount validating ok', self.run_driver('remount_validating'))

    def test_remount_mid_replay_ends_the_streams(self):
        self.assertIn('remount flight ok', self.run_driver('remount_flight'))


class IgtWire(unittest.TestCase):
    def test_directory_one_only_in_the_ground_test_image(self):
        code = r'''#include "sd_file_wire.h"
#include <assert.h>
#include <stdio.h>
static uint8_t parse(const uint8_t *p, size_t n) { sd_file_request_t r; return sd_file_parse_request(p, n, &r); }
static uint8_t hash_req(uint8_t dir, const char *name)
{ uint8_t p[100]; size_t n = strlen(name); p[0] = 0x27; p[1] = dir; p[2] = (uint8_t)n; memcpy(p + 3, name, n); return parse(p, 3 + n); }
int main(void)
{
    const uint8_t list1[] = {0x21, 1, 2, 0}, list2[] = {0x21, 2, 0, 0};
    sd_file_request_t r;
    const uint8_t s = sd_file_parse_request(list1, 4, &r);
#if FC_GROUND_TEST
    assert(s == 0 && r.dir == 1 && r.index == 2);
    assert(hash_req(1, "IMU0.CSV") == 0 && hash_req(1, "baro9.csv") == 0 && hash_req(1, "GPS3.CSV") == 0);
    assert(hash_req(1, "irec-1-000.csv") == 1 && hash_req(0, "IMU0.CSV") == 1);
    assert(hash_req(1, "IMU10.CSV") == 1 && hash_req(1, "IMU.CSV") == 1 && hash_req(1, "IMUA.CSV") == 1);
    assert(hash_req(1, "../IMU0.CSV") == 1 && hash_req(1, "IMU0.TXT") == 1 && hash_req(1, "XIMU0.CSV") == 1);
    uint8_t f[SD_FILE_ANSWER_MAX];
    assert(sd_file_encode_list(f, 0, 1, 0, 9, 0, "IMU0.CSV", 8) == 20 && f[2] == 1);
#else
    assert(s == 1 && hash_req(1, "IMU0.CSV") == 1);
#endif
    assert(parse(list2, 4) == 1 && hash_req(0, "irec-1-000.csv") == 0);
    puts("ok");
}
'''
        for flag in ('-DFC_GROUND_TEST=1', '-DFC_GROUND_TEST=0'):
            with self.subTest(flag=flag), tempfile.TemporaryDirectory() as tmp:
                exe = pathlib.Path(tmp) / 'wire'
                subprocess.run(CC + [flag, '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-x', 'none', *ENGINE,
                                     '-o', str(exe)],
                               input='#include <stddef.h>\n#include <stdint.h>\n' + crc_source() + code,
                               text=True, check=True)
                self.assertEqual(subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout, 'ok\n')


class GroundTestImageWiring(unittest.TestCase):
    def test_build_matrix_includes_the_service_in_the_ground_test_image(self):
        """Runs the CMakeLists.txt FC_SD_SERVICE block under cmake -P for every
        combination of its inputs and the ground-test option."""
        import itertools
        import shutil
        if not shutil.which('cmake'):
            self.skipTest('cmake not installed')
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        start = cmake.index('option(FC_SD_SERVICE')
        block = cmake[start:cmake.index('endif()', start) + len('endif()')]
        self.assertNotIn('INTEGRATED_GROUND_TEST', block, 'no ground-test exclusion')
        stub = (
            'set(CMAKE_PROJECT_NAME fc)\n'
            'function(target_compile_definitions t scope)\n'
            '  set_property(GLOBAL APPEND PROPERTY DEFS ${ARGN})\nendfunction()\n'
            'function(target_sources t scope)\n'
            '  set_property(GLOBAL APPEND PROPERTY SRCS ${ARGN})\nendfunction()\n')
        report = ('get_property(d GLOBAL PROPERTY DEFS)\nget_property(s GLOBAL PROPERTY SRCS)\n'
                  'message(NOTICE "ACTIVE=${FC_SD_SERVICE_ACTIVE};DEFS=${d};SRCS=${s}")\n')
        with tempfile.TemporaryDirectory() as tmp:
            script = pathlib.Path(tmp) / 'matrix.cmake'
            script.write_text(stub + block + '\n' + report)
            keys = ('ENABLE_TELEMETRY', 'ONBOARD_SD', 'TELEMETRY_USE_TLSF', 'INTEGRATED_GROUND_TEST', 'FC_SD_SERVICE')
            for values in itertools.product((True, False), repeat=len(keys)):
                config = dict(zip(keys, values))
                with self.subTest(**config):
                    args = ['cmake'] + [f'-D{k}={"ON" if v else "OFF"}' for k, v in config.items()] + ['-P', str(script)]
                    out = subprocess.run(args, capture_output=True, text=True, check=True).stderr
                    want = config['FC_SD_SERVICE'] and config['ENABLE_TELEMETRY'] and config['ONBOARD_SD'] \
                        and config['TELEMETRY_USE_TLSF']
                    self.assertIn('ACTIVE=ON' if want else 'ACTIVE=OFF', out)
                    self.assertEqual('FC_SD_SERVICE=1' in out, want)
                    self.assertEqual('sd_file_stream.c' in out, want)

    def test_a_configured_ground_test_build_has_the_service(self):
        """Any configured --ground-test build tree defines FC_SD_SERVICE."""
        import re
        trees = [c.parent for c in ROOT.glob('build/*/CMakeCache.txt')
                 if re.search(r'(?m)^INTEGRATED_GROUND_TEST:BOOL=ON$', c.read_text())]
        if not trees:
            self.skipTest('no configured --ground-test build')
        for tree in trees:
            with self.subTest(tree=tree.name):
                ninja = (tree / 'build.ninja').read_text()
                self.assertIn('-DFC_GROUND_TEST=1', ninja)
                self.assertIn('-DFC_SD_SERVICE=1', ninja)
                self.assertIn('sd_file_stream.c', ninja)

    def test_one_thread_owns_filex(self):
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        task = src[src.index('void sd_pipeline_task()'):]
        # The replay and the service run from the same SD-task loop...
        self.assertLess(task.index('igt_sd_service();'), task.index('sd_file_service_turn();\n    }'))
        # ...and from nowhere else.
        for path in sorted((ROOT / 'Core').rglob('*.c')):
            text = path.read_text()
            if path.name in ('ground_test.c', 'sdpipeline.c'):
                continue
            with self.subTest(path=path.name):
                self.assertNotIn('igt_sd_service(', text)
                self.assertNotIn('sd_file_service_turn(', text)
        self.assertEqual(src.count('igt_sd_service();'), 1)
        remount = src[src.index('void sd_pipeline_media_remounted(void)'):]
        remount = remount[:remount.index('\n}\n')]
        self.assertIn('sd_file_stream_media_remounted();', remount)
        self.assertIn('igt_media_remounted();', remount)

    def test_health_reports_the_service(self):
        health = (ROOT / 'Core/Src/board_health.c').read_text()
        self.assertRegex(health, r'#ifdef FC_SD_SERVICE\n\s+words\[6\] \|= 1U << 29U;')


if __name__ == '__main__':
    unittest.main()

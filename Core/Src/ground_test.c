/* Core/Src/ground_test.c
 *
 * Integrated ground test: SD-card CSV replay into the real estimator after a
 * real Launch on the stand, and the dry-run/LIVE PYRO gate. See
 * ground_test.h and GroundStation26 docs/integrated-ground-test.md.
 *
 * Threads: the SD task alone opens, reads and parses the CSVs and fills one
 * single-producer ring of parsed rows per sensor. The distribution task only
 * pops rows and interpolates; no text is parsed on the sensor path. The
 * recovery task changes the mode. No allocation; ~2.5 KB of RAM. */

#include "ground_test.h"

#if FC_GROUND_TEST

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#ifdef IGT_HOST_TEST
#include "ground_test_host_port.h"
#else
#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#ifdef SD_AVAILABLE
#include "resilient_storage.h"
#endif

#define IGT_PUB_PERIOD_MS FC_TELEMETRY_PERIOD_MS
#define IGT_SD_PERIOD_MS  LOG_RATE_SD

#ifdef SD_AVAILABLE
static FX_FILE igt_file;

/* 0 open, -1 no such file, -2 card or transport failure */
static int igt_port_open(const char *path)
{
  if (flight_sd_prepare_io() != FX_SUCCESS) return -2;
  const UINT st = fx_file_open(&sdio_disk, &igt_file, (CHAR *)path, FX_OPEN_FOR_READ);
  return st == FX_SUCCESS ? 0 : (st == FX_NOT_FOUND || st == FX_INVALID_PATH) ? -1 : -2;
}

static long igt_port_read(uint32_t offset, char *buf, uint32_t cap)
{
  ULONG got = 0U;
  if (fx_file_seek(&igt_file, offset) != FX_SUCCESS) return -1;
  const UINT st = fx_file_read(&igt_file, buf, cap, &got);
  return (st == FX_SUCCESS || st == FX_END_OF_FILE) ? (long)got : -1;
}

static void igt_port_close(void) { (void)fx_file_close(&igt_file); }
#else
static int igt_port_open(const char *path) { (void)path; return -2; }
static long igt_port_read(uint32_t o, char *b, uint32_t c) { (void)o; (void)b; (void)c; return -1; }
static void igt_port_close(void) {}
#endif

static void igt_port_message(const char *text) { message(text, true); }

static void igt_port_sample(const float v[8], bool radio, bool sd)
{
  if (radio) log_periodic_f32(SEDS_DT_SIM_SAMPLE, 8, v);
#ifdef SD_AVAILABLE
  if (sd) sd_append_f32(SEDS_DT_SIM_SAMPLE, v, 8);
#else
  (void)sd;
#endif
}
#endif /* IGT_HOST_TEST */


#define IGT_CHUNK          256U  /* < one sector: FileX reads via its cache */
#define IGT_LINE_MAX       120U
#define IGT_SEGMENTS       10U
#define IGT_MIN_COVER_MS   5000U
#define IGT_GPS_PERIOD_MS  100U
#define IGT_END_HOLD_MS    30000U
#define IGT_REST_ACC       1.5f
#define IGT_REST_GYRO      15.0f
#define IGT_REST_ALT       20.0f
#define IGT_MOTION_ALT     50.0f
#define IGT_MOTION_SAMPLES 10U
#define IGT_REFILL_STEPS   12U

enum { S_IMU, S_BARO, S_GPS, S_COUNT };
enum { R_MORE, R_FULL, R_EOF, R_ERR };

typedef struct { uint32_t t; float v[6]; } igt_row;

typedef struct {
  const char *name, *header;
  uint8_t fields, cap;
  igt_row *ring;
  _Atomic uint8_t head, tail;        /* free-running; cap divides 256 */
  /* SD task */
  uint32_t off, line, rows, last_t;
  uint8_t seg;
  bool eof, header_seen;
  /* distribution task */
  igt_row prev, next;
  uint8_t have;
} igt_stream;

static igt_row ring_imu[32], ring_baro[32], ring_gps[16];
static igt_stream streams[S_COUNT] = {
  { .name = "IMU", .header = "t_ms,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps",
    .fields = 6U, .cap = 32U, .ring = ring_imu },
  { .name = "BARO", .header = "t_ms,alt_m,pressure_pa,temp_c",
    .fields = 3U, .cap = 32U, .ring = ring_baro },
  { .name = "GPS", .header = "t_ms,lat_deg,lon_deg,alt_msl_m",
    .fields = 3U, .cap = 16U, .ring = ring_gps },
};

static _Atomic uint8_t mode;
static volatile uint8_t reason, live, dry_bits, gps_present, gps_sim;
static uint32_t t0, gps_due, motion_count;
static float real_imu[6], real_baro[3];
static uint32_t pub_last[6], sd_last[6];
static char chunk[IGT_CHUNK + 1U];

static const char *const reason_text[] = {
  "ok", "SD unavailable", "file missing", "bad header", "parse error",
  "time not increasing", "value out of range", "too short",
  "not stationary at Launch", "real motion: replay abandoned",
  "ring underrun", "refused", "data ended before landing",
};

static void say(const char *text) { igt_port_message(text); }



/* Ring (single producer: SD task; single consumer: one task at a time) */

static uint8_t ring_count(const igt_stream *s)
{
  return (uint8_t)(atomic_load_explicit(&s->head, memory_order_acquire) -
                   atomic_load_explicit(&s->tail, memory_order_acquire));
}

static void ring_push(igt_stream *s, const igt_row *row)
{
  const uint8_t head = atomic_load_explicit(&s->head, memory_order_relaxed);
  s->ring[head % s->cap] = *row;
  atomic_store_explicit(&s->head, (uint8_t)(head + 1U), memory_order_release);
}

static bool ring_pop(igt_stream *s, igt_row *row)
{
  const uint8_t tail = atomic_load_explicit(&s->tail, memory_order_relaxed);
  if ((uint8_t)(atomic_load_explicit(&s->head, memory_order_acquire) - tail) == 0U)
    return false;
  *row = s->ring[tail % s->cap];
  atomic_store_explicit(&s->tail, (uint8_t)(tail + 1U), memory_order_release);
  return true;
}


/* CSV numbers: [+-]digits[.digits], at most 9 significant digits (further
 * fraction digits are truncated). No exponent, no locale, no strtof. */

static const float pow10_neg[10] = {
  1.0f, 1e-1f, 1e-2f, 1e-3f, 1e-4f, 1e-5f, 1e-6f, 1e-7f, 1e-8f, 1e-9f
};

static const char *parse_number(const char *p, float *out, bool integer)
{
  bool negative = false;
  if (!integer && (*p == '-' || *p == '+')) negative = *p++ == '-';
  uint32_t mantissa = 0U;
  int digits = 0, fraction = -1;
  for (;; ++p) {
    if (*p >= '0' && *p <= '9') {
      if (digits < 9) {
        mantissa = mantissa * 10U + (uint32_t)(*p - '0');
        ++digits;
        if (fraction >= 0) ++fraction;
      } else if (fraction < 0) {
        return NULL; /* integer part too long */
      }
    } else if (*p == '.' && !integer && fraction < 0) {
      fraction = 0;
    } else {
      break;
    }
  }
  if (digits == 0) return NULL;
  float value = (float)mantissa;
  if (fraction > 0) value *= pow10_neg[fraction];
  *out = negative ? -value : value;
  return p;
}

static bool in_range(float v, float lo, float hi) { return v >= lo && v <= hi; }

/* Returns 0 or an igt_reason. */
static int parse_row(const igt_stream *s, const char *p, igt_row *row)
{
  float t;
  if (!(p = parse_number(p, &t, true))) return IGT_E_PARSE;
  row->t = (uint32_t)t;
  for (unsigned i = 0U; i < s->fields; ++i) {
    if (*p++ != ',' || !(p = parse_number(p, &row->v[i], false))) return IGT_E_PARSE;
  }
  if (*p != '\0') return IGT_E_PARSE;
  for (unsigned i = s->fields; i < 6U; ++i) row->v[i] = 0.0f;
  const float *v = row->v;
  if (s == &streams[S_IMU]) {
    for (unsigned i = 0U; i < 3U; ++i)
      if (!in_range(v[i], MIN_ACC, MAX_ACC) || !in_range(v[i + 3U], MIN_DPS, MAX_DPS))
        return IGT_E_RANGE;
  } else if (s == &streams[S_BARO]) {
    if (!in_range(v[0], MIN_ALT, MAX_ALT) || !in_range(v[1], MIN_PRS, MAX_PRS))
      return IGT_E_RANGE;
  } else if (!in_range(v[0], MIN_LAT, MAX_LAT) || !in_range(v[1], MIN_LON, MAX_LON) ||
             !in_range(v[2], MIN_SEA, MAX_SEA)) {
    return IGT_E_RANGE;
  }
  return 0;
}


/* SD task: one bounded chunk of one stream */

static void stream_rewind(igt_stream *s)
{
  s->off = s->line = s->rows = s->last_t = 0U;
  s->seg = 0U;
  s->eof = s->header_seen = false;
}

static int stream_fail(igt_stream *s, int why, bool validating)
{
  if (!validating) { s->eof = true; return R_ERR; } /* replay holds its last row */
  reason = (uint8_t)why;
  char buf[96];
  snprintf(buf, sizeof buf, "IGT %s%u.CSV:%u %s", s->name, (unsigned)s->seg,
           (unsigned)s->line, reason_text[why]);
  say(buf);
  uint8_t expected = IGT_VALIDATING;
  atomic_compare_exchange_strong(&mode, &expected, (uint8_t)IGT_FAULT);
  return R_ERR;
}

static int stream_step(igt_stream *s, bool validating, bool optional)
{
  char path[20];
  snprintf(path, sizeof path, "/IGT/%s%u.CSV", s->name, (unsigned)s->seg);
  const int opened = igt_port_open(path);
  if (opened == -1) {
    if (s->seg == 0U && !optional) return stream_fail(s, IGT_E_MISSING, validating);
    s->eof = true;
    return R_EOF;
  }
  if (opened < 0) return stream_fail(s, IGT_E_SD, validating);
  const long n = igt_port_read(s->off, chunk, IGT_CHUNK);
  igt_port_close();
  if (n < 0) return stream_fail(s, IGT_E_SD, validating);
  if (n == 0) {
    if (!s->header_seen) return stream_fail(s, IGT_E_HEADER, validating);
    s->off = s->line = 0U;
    s->header_seen = false;
    if (++s->seg >= IGT_SEGMENTS) { s->eof = true; return R_EOF; }
    return R_MORE;
  }
  chunk[n] = '\0';
  const bool last = (uint32_t)n < IGT_CHUNK;
  char *p = chunk, *const end = chunk + n;
  while (p < end) {
    if (!validating && ring_count(s) >= s->cap) return R_FULL;
    char *nl = memchr(p, '\n', (size_t)(end - p));
    if (nl == NULL) {
      if (!last) break;
      nl = end;
    }
    const uint32_t consumed = (uint32_t)(nl - p) + (nl < end ? 1U : 0U);
    size_t len = (size_t)(nl - p);
    if (len && p[len - 1U] == '\r') --len;
    ++s->line;
    if (len > IGT_LINE_MAX) return stream_fail(s, IGT_E_PARSE, validating);
    p[len] = '\0';
    if (len == 0U || *p == '#') {
      /* blank or comment */
    } else if (!s->header_seen) {
      if (strcmp(p, s->header) != 0) return stream_fail(s, IGT_E_HEADER, validating);
      s->header_seen = true;
    } else {
      igt_row row;
      const int bad = parse_row(s, p, &row);
      if (bad) return stream_fail(s, bad, validating);
      if (s->rows && row.t <= s->last_t) return stream_fail(s, IGT_E_TIME, validating);
      s->last_t = row.t;
      ++s->rows;
      if (!validating) ring_push(s, &row);
    }
    s->off += consumed;
    p += consumed;
  }
  if (p == chunk) { ++s->line; return stream_fail(s, IGT_E_PARSE, validating); }
  return R_MORE;
}

static bool still_validating(void)
{
  return atomic_load_explicit(&mode, memory_order_acquire) == IGT_VALIDATING;
}

static void validate_and_prime(void)
{
  for (unsigned i = 0U; i < S_COUNT; ++i) {
    igt_stream *s = &streams[i];
    stream_rewind(s);
    while (!s->eof) {
      if (!still_validating() || stream_step(s, true, i == S_GPS) == R_ERR) return;
    }
    if (i == S_GPS && s->rows == 0U && s->seg == 0U) continue; /* GPS optional */
    if (s->rows < 2U || (i != S_GPS && s->last_t < IGT_MIN_COVER_MS)) {
      stream_fail(s, IGT_E_SHORT, true);
      return;
    }
  }
  gps_present = streams[S_GPS].rows >= 2U;
  {
    char buf[96];
    snprintf(buf, sizeof buf, "IGT CSV ok: IMU %u rows %us, BARO %u rows %us, GPS %u rows",
             (unsigned)streams[S_IMU].rows, (unsigned)(streams[S_IMU].last_t / 1000U),
             (unsigned)streams[S_BARO].rows, (unsigned)(streams[S_BARO].last_t / 1000U),
             (unsigned)streams[S_GPS].rows);
    say(buf);
  }
  /* Prime the rings from the start for the replay. */
  for (unsigned i = 0U; i < S_COUNT; ++i) {
    igt_stream *s = &streams[i];
    stream_rewind(s);
    atomic_store_explicit(&s->head, 0U, memory_order_relaxed);
    atomic_store_explicit(&s->tail, 0U, memory_order_relaxed);
    s->have = 0U;
    if (i == S_GPS && !gps_present) { s->eof = true; continue; }
    while (!s->eof && ring_count(s) < s->cap) {
      if (!still_validating() || stream_step(s, false, false) != R_MORE) break;
    }
  }
  uint8_t expected = IGT_VALIDATING;
  if (atomic_compare_exchange_strong(&mode, &expected, (uint8_t)IGT_READY))
    say("IGT ready: dry-run");
}

void igt_sd_service(void)
{
  const uint8_t m = atomic_load_explicit(&mode, memory_order_acquire);
  if (m == IGT_VALIDATING) { validate_and_prime(); return; }
  if (m != IGT_READY && m != IGT_SIM) return;
  for (unsigned i = 0U; i < S_COUNT; ++i) {
    igt_stream *s = &streams[i];
    for (unsigned k = 0U; k < IGT_REFILL_STEPS && !s->eof && ring_count(s) < s->cap; ++k)
      if (stream_step(s, false, false) != R_MORE) break;
  }
}


/* Distribution task: linear interpolation at replay time t (ms) */

static bool stream_exhausted(const igt_stream *s, uint32_t t)
{
  return s->eof && ring_count(s) == 0U && s->have == 2U && t >= s->next.t + IGT_END_HOLD_MS;
}

/* out == NULL only advances the stream to t: a sensor the flight phase is
 * not reading (GPS before apogee, IMU after it) stays current. */
static void sample(igt_stream *s, uint32_t t, float *out)
{
  igt_row row;
  while (s->have < 2U || s->next.t <= t) {
    if (!ring_pop(s, &row)) break;
    if (s->have == 0U) { s->prev = row; s->have = 1U; continue; }
    if (s->have == 2U) s->prev = s->next;
    s->next = row;
    s->have = 2U;
  }
  const igt_row *a = &s->prev, *b = &s->next;
  if (s->have == 0U || out == NULL) return; /* never primed: keep the caller's value */
  if (s->have == 1U || t <= a->t) { memcpy(out, a->v, s->fields * sizeof(float)); return; }
  if (t >= b->t) {
    if (!s->eof && reason == IGT_OK) reason = IGT_I_UNDERRUN;
    memcpy(out, b->v, s->fields * sizeof(float));
    return;
  }
  const float k = (float)(t - a->t) / (float)(b->t - a->t);
  for (unsigned i = 0U; i < s->fields; ++i) out[i] = a->v[i] + (b->v[i] - a->v[i]) * k;
}

static bool running(void)
{
  return atomic_load_explicit(&mode, memory_order_acquire) == IGT_SIM;
}

bool igt_sim_running(void) { return running(); }
bool igt_engaged(void) { return atomic_load_explicit(&mode, memory_order_acquire) != IGT_OFF; }

static void finish(uint8_t to, uint8_t why, const char *text)
{
  uint8_t expected = IGT_SIM;
  if (!atomic_compare_exchange_strong(&mode, &expected, to)) return;
  gps_sim = 0U;
  if (why != IGT_OK) reason = why;
  say(text);
}

void igt_override_accl(float *v, uint32_t now)
{
  if (!running()) return;
  float out[6];
  memcpy(real_imu, v, 3U * sizeof(float));
  memcpy(out, v, 3U * sizeof(float));
  memcpy(out + 3, real_imu + 3, 3U * sizeof(float));
  sample(&streams[S_IMU], now - t0, out);
  memcpy(v, out, 3U * sizeof(float));
}

void igt_override_gyro(float *v, uint32_t now)
{
  if (!running()) return;
  float out[6];
  memcpy(real_imu + 3, v, 3U * sizeof(float));
  memcpy(out, real_imu, 3U * sizeof(float));
  memcpy(out + 3, v, 3U * sizeof(float));
  sample(&streams[S_IMU], now - t0, out);
  memcpy(v, out + 3, 3U * sizeof(float));
}

void igt_override_baro(float *v, uint32_t now)
{
  if (!running()) return;
  memcpy(real_baro, v, sizeof real_baro);
  /* The real barometer is the vehicle's own: a real climb ends the replay
   * and returns the pyros to normal flight behaviour. */
  motion_count = v[0] > IGT_MOTION_ALT ? motion_count + 1U : 0U;
  if (motion_count >= IGT_MOTION_SAMPLES) {
    live = 1U;
    finish(IGT_FAULT, IGT_E_REAL_MOTION, "IGT real motion: replay abandoned, real sensors, pyros live");
    return;
  }
  const uint32_t t = now - t0;
  sample(&streams[S_BARO], t, v);
  sample(&streams[S_IMU], t, NULL);
  sample(&streams[S_GPS], t, NULL);
  if (stream_exhausted(&streams[S_BARO], t) && stream_exhausted(&streams[S_IMU], t)) {
    finish(IGT_DONE, IGT_I_DATA_ENDED, "IGT data ended before landing: real sensors; GPS: real");
  }
}

bool igt_gps_simulated(void) { return running() && gps_sim; }

bool igt_gps_fix(float *kf_gps, uint32_t now)
{
  if (!igt_gps_simulated() || (int32_t)(now - gps_due) < 0) return false;
  gps_due = now + IGT_GPS_PERIOD_MS;
  float fix[3];
  sample(&streams[S_GPS], now - t0, fix);
  kf_gps[0] = fix[2]; /* sea */
  kf_gps[1] = fix[1]; /* lon */
  kf_gps[2] = fix[0]; /* lat */
  igt_publish(IGT_KIND_GPS, fix, 3U, now);
  return true;
}

const float *igt_real_imu(void) { return real_imu; }
const float *igt_real_baro(void) { return real_baro; }

void igt_publish(unsigned kind, const float *v, unsigned count, uint32_t now)
{
  if (kind == 0U || kind > IGT_KIND_DESCENT || count > 6U) return;
  float out[8] = { (float)kind, (float)(now - t0) * 0.001f };
  memcpy(out + 2, v, count * sizeof(float));
  const bool radio = now - pub_last[kind] >= IGT_PUB_PERIOD_MS;
  const bool sd = now - sd_last[kind] >= IGT_SD_PERIOD_MS;
  if (radio) pub_last[kind] = now;
  if (sd) sd_last[kind] = now;
  if (radio || sd) igt_port_sample(out, radio, sd);
}


/* Recovery task: commands and Launch */

static bool refuse(const char *what)
{
  /* While Off the status has no other reason: show the refusal there so the
   * station can say why nothing changed. An active test keeps its own. */
  const uint8_t m = atomic_load_explicit(&mode, memory_order_acquire);
  if (m == IGT_OFF) reason = IGT_E_REFUSED;
  char buf[64];
  snprintf(buf, sizeof buf, "IGT %s refused (mode %u)", what, (unsigned)m);
  say(buf);
  return false;
}

bool igt_command(unsigned cmd, bool pad, bool launch_requested)
{
  const uint8_t m = atomic_load_explicit(&mode, memory_order_acquire);
  switch (cmd) {
    case IGT_CMD_ENTER:
      if (!pad || launch_requested)
        return refuse("enter: needs Startup, no Postinit/Launch, station heard");
      if (m == IGT_VALIDATING || m == IGT_READY || m == IGT_SIM)
        return refuse("enter");
      live = dry_bits = gps_sim = gps_present = 0U;
      reason = IGT_OK;
      motion_count = 0U;
      atomic_store_explicit(&mode, (uint8_t)IGT_VALIDATING, memory_order_release);
      say("IGT entered: validating CSV; pyros dry-run");
      return true;
    case IGT_CMD_LIVE_PYRO:
      if (m != IGT_READY || launch_requested) return refuse("LIVE PYRO");
      live = 1U;
      say("IGT LIVE PYRO armed");
      return true;
    case IGT_CMD_DRY_RUN:
      live = 0U;
      say("IGT pyros dry-run");
      return true;
    case IGT_CMD_EXIT:
      if (m == IGT_SIM) return refuse("exit");
      live = gps_sim = 0U;
      atomic_store_explicit(&mode, (uint8_t)IGT_OFF, memory_order_release);
      say("IGT off");
      return true;
    default:
      return false;
  }
}

void igt_stop(void)
{
  /* No I/O: callers may hold the pyro IRQ guard. */
  live = gps_sim = 0U;
  atomic_store_explicit(&mode, (uint8_t)IGT_OFF, memory_order_release);
}

bool igt_launch(uint32_t now, const float *accl, const float *gyro,
                float alt, float *seed)
{
  uint8_t m = atomic_load_explicit(&mode, memory_order_acquire);
  if (m == IGT_OFF) return false;
  if (m != IGT_READY) {
    if (m == IGT_VALIDATING) {
      reason = IGT_E_REFUSED;
      atomic_store_explicit(&mode, (uint8_t)IGT_FAULT, memory_order_release);
      say("IGT Launch before CSV ready: no replay, real sensors");
    }
    return false;
  }
  const float n2 = accl[0] * accl[0] + accl[1] * accl[1] + accl[2] * accl[2];
  const float lo = GRAVITY_SI - IGT_REST_ACC, hi = GRAVITY_SI + IGT_REST_ACC;
  bool rest = n2 >= lo * lo && n2 <= hi * hi && alt < IGT_REST_ALT && alt > -IGT_REST_ALT;
  for (unsigned i = 0U; i < 3U; ++i)
    rest = rest && gyro[i] < IGT_REST_GYRO && gyro[i] > -IGT_REST_GYRO;
  if (!rest) {
    reason = IGT_E_MOVING;
    atomic_store_explicit(&mode, (uint8_t)IGT_FAULT, memory_order_release);
    say("IGT not stationary at Launch: no replay, real sensors");
    return false;
  }
  memcpy(real_imu, accl, 3U * sizeof(float));
  memcpy(real_imu + 3, gyro, 3U * sizeof(float));
  real_baro[0] = alt;
  t0 = now;
  gps_due = now;
  motion_count = 0U;
  for (unsigned i = 0U; i < 6U; ++i) pub_last[i] = sd_last[i] = now - 1000000U;
  float first[6] = { accl[0], accl[1], accl[2], gyro[0], gyro[1], gyro[2] };
  sample(&streams[S_IMU], 0U, first);
  memcpy(seed, first, 3U * sizeof(float));
  gps_sim = gps_present;
  atomic_store_explicit(&mode, (uint8_t)IGT_SIM, memory_order_release);
  say(gps_present ? "IGT replay started; GPS: simulated" : "IGT replay started; GPS: real");
  return true;
}

void igt_landed(void)
{
  finish(IGT_DONE, IGT_OK, "IGT replay ended at landing: real sensors; GPS: real");
}


/* Pyro gate and status */

bool igt_pyro_dry(void)
{
  return atomic_load_explicit(&mode, memory_order_acquire) != IGT_OFF && !live;
}

void igt_note_dry(bool reef) { dry_bits |= reef ? 2U : 1U; }

uint32_t igt_health_bits(void)
{
  const uint32_t m = atomic_load_explicit(&mode, memory_order_acquire);
  return IGT_HEALTH_SUPPORTED | (m << 4U) |
         (live ? IGT_HEALTH_LIVE : 0U) |
         (m == IGT_SIM && gps_sim ? IGT_HEALTH_GPS_SIM : 0U) |
         ((uint32_t)dry_bits << 10U) |
         ((uint32_t)(reason & 0xFU) << 12U);
}

#endif /* FC_GROUND_TEST */

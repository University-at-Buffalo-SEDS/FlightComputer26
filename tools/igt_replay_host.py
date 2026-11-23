#!/usr/bin/env python3
"""Replay an integrated-ground-test CSV set through the FC's real estimator on
the host, and report what the flight computer would decide.

Compiles, for a 32-bit host (gcc -m32, like the native SITL):
  * Core/Src/ground_test.c   the real SD-card CSV validation, ring replay and
                             linear interpolation (IGT_HOST_TEST port: the
                             "card" is a directory);
  * Core/Src/kalman.c        the ascent EKF and descent KF;
  * Core/Src/evaluation.c    the flight state machine and deployment decisions;
  * CMSIS-DSP                the matrix/trig functions they call;
against the board's real headers, with ThreadX/HAL calls stubbed. A small
driver plays the distribution and evaluation tasks' schedule: the replay is
sampled exactly as on the board (baro conversions every 40 ms, the board's
Baro_ODR_25 in recovery.c, drive the evaluator; ascent predict on the IMU
just after each baro update; descent
predict/update on every baro sample; replayed GPS at 10 Hz, its first descent
fix becoming the rail reference), and tx_thread_sleep() pauses the evaluator
for its real duration. Deployments go to a dry-run stub.

    python3 tools/igt_replay_host.py <dir with IGT/*.CSV> [--json out.json] [--trace trace.csv]

Prints the FC state transitions, deployment times and the estimator's apogee
against the replayed baro. Only what this tool stubs differs from the board:
CPU timing (the real cycle is not instantaneous), sensor validation and
fault/recovery paths, and telemetry. Build outputs go to a temporary
directory that is removed afterwards.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

PRE = r'''
#include <stdint.h>
#include <stddef.h>
#define __CMSIS_GCC_H
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __RESTRICT __restrict
#define __COMPILER_BARRIER() __asm volatile("" ::: "memory")
#define __UNALIGNED_UINT32(x) (*(uint32_t *)(x))
extern uint32_t host_primask;
static inline uint32_t __get_PRIMASK(void) { return host_primask; }
static inline void __set_PRIMASK(uint32_t v) { host_primask = v; }
static inline void __disable_irq(void) { host_primask = 1; }
static inline void __enable_irq(void) { host_primask = 0; }
static inline void __DMB(void) {}
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __NOP(void) {}
static inline void __WFI(void) {}
static inline void __WFE(void) {}
static inline void __SEV(void) {}
#define __BKPT(x) ((void)0)
#define HOST_REG(name) static inline uint32_t __get_##name(void) { return 0; } \
                       static inline void __set_##name(uint32_t v) { (void)v; }
HOST_REG(CONTROL) HOST_REG(MSP) HOST_REG(PSP) HOST_REG(BASEPRI) HOST_REG(FAULTMASK)
HOST_REG(MSPLIM) HOST_REG(PSPLIM) HOST_REG(FPSCR)
static inline uint32_t __get_IPSR(void) { return 0; }
static inline void __set_BASEPRI_MAX(uint32_t v) { (void)v; }
static inline uint32_t __REV(uint32_t v) { return __builtin_bswap32(v); }
static inline uint32_t __REV16(uint32_t v) { return ((v & 0xff00ff00u) >> 8) | ((v & 0x00ff00ffu) << 8); }
static inline int16_t __REVSH(int16_t v) { return (int16_t)__builtin_bswap16((uint16_t)v); }
static inline uint32_t __ROR(uint32_t a, uint32_t b) { b %= 32; return b ? (a >> b) | (a << (32 - b)) : a; }
static inline uint32_t __RBIT(uint32_t v) { uint32_t r = 0; for (int i = 0; i < 32; ++i) { r = (r << 1) | (v & 1); v >>= 1; } return r; }
static inline uint8_t __CLZ(uint32_t v) { return v ? (uint8_t)__builtin_clz(v) : 32; }
static inline int32_t __SSAT(int32_t v, uint32_t s) { int32_t mx = (1 << (s - 1)) - 1, mn = -mx - 1; return v > mx ? mx : v < mn ? mn : v; }
static inline uint32_t __USAT(int32_t v, uint32_t s) { int32_t mx = (1 << s) - 1; return v > mx ? (uint32_t)mx : v < 0 ? 0 : (uint32_t)v; }
static inline uint8_t __LDREXB(volatile uint8_t *a) { return *a; }
static inline uint16_t __LDREXH(volatile uint16_t *a) { return *a; }
static inline uint32_t __LDREXW(volatile uint32_t *a) { return *a; }
static inline uint32_t __STREXB(uint8_t v, volatile uint8_t *a) { *a = v; return 0; }
static inline uint32_t __STREXH(uint16_t v, volatile uint16_t *a) { *a = v; return 0; }
static inline uint32_t __STREXW(uint32_t v, volatile uint32_t *a) { *a = v; return 0; }
static inline void __CLREX(void) {}
'''

PORT = r'''
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "fcconfig.h"
#define IGT_PUB_PERIOD_MS 200U
#define IGT_SD_PERIOD_MS 50U
char igt_root[512];
float g_launch_site[3] = {LAUNCH_SITE_LAT_DEFAULT, LAUNCH_SITE_LON_DEFAULT, LAUNCH_SITE_SEA_DEFAULT};
static FILE *igt_fp;
void host_igt_message(const char *text);
static bool igt_port_sd_ready(void) { return true; }
static int igt_port_open(const char *path)
{
  char full[640];
  snprintf(full, sizeof full, "%s%s", igt_root, path);
  igt_fp = fopen(full, "rb");
  return igt_fp ? 0 : -1;
}
static long igt_port_read(uint32_t off, char *buf, uint32_t cap)
{
  if (fseek(igt_fp, (long)off, SEEK_SET) != 0) return -1;
  return (long)fread(buf, 1, cap, igt_fp);
}
static void igt_port_close(void) { fclose(igt_fp); igt_fp = NULL; }
static void igt_port_message(const char *text) { host_igt_message(text); }
static void igt_port_sample(const float v[8], bool radio, bool sd) { (void)v; (void)radio; (void)sd; }
'''

DRIVER = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"

uint32_t host_primask;
static uint32_t host_now;
static uint32_t host_wake;
static uint32_t host_t0;
static FILE *trace;
extern char igt_root[512];
extern float g_launch_site[3];

/* Globals the evaluator and filters share with tasks this host does not run */
measm meas;
spinlock meas_locks[MEMS_Devices];
volatile log_rates rates = { LOG_RATE_SD, LOG_RATE_GND };
volatile fu32 local_time[Time_Users];
atomic_uint_fast32_t g_conf = FC_DEFAULTS | USER_OPTIONS;
TX_QUEUE seds_syscall;
static float drogue_t = -1, main_t = -1, drogue_alt, main_alt;

uint32_t HAL_GetTick(void) { return host_now; }
static double replay_s(void) { return (host_now - host_t0) / 1000.0; }
void host_igt_message(const char *text) { printf("[%9.3f] %s\n", replay_s(), text); }
void *_sbrk(ptrdiff_t n) { return aligned_alloc(32, (size_t)((n + 31) & ~31)); }
UINT _tx_thread_sleep(ULONG ticks) { host_wake = host_now + (uint32_t)ticks; return TX_SUCCESS; }
VOID _txe_thread_relinquish(VOID) {}
UINT _txe_queue_send(TX_QUEUE *q, VOID *s, ULONG w) { (void)q; (void)s; (void)w; return TX_SUCCESS; }
UINT _txe_byte_allocate(TX_BYTE_POOL *p, VOID **m, ULONG s, ULONG w) { (void)p; (void)w; *m = malloc(s); return TX_SUCCESS; }
UINT _txe_event_flags_create(TX_EVENT_FLAGS_GROUP *g, CHAR *n, UINT s) { (void)g; (void)n; (void)s; return TX_SUCCESS; }
UINT _txe_event_flags_get(TX_EVENT_FLAGS_GROUP *g, ULONG r, UINT o, ULONG *a, ULONG w) { (void)g; (void)r; (void)o; (void)a; (void)w; return TX_SUCCESS; }
UINT _txe_thread_create(TX_THREAD *t, CHAR *n, VOID (*e)(ULONG), ULONG i, VOID *s, ULONG z, UINT p, UINT pt, ULONG ts, UINT a, UINT c)
{ (void)t; (void)n; (void)e; (void)i; (void)s; (void)z; (void)p; (void)pt; (void)ts; (void)a; (void)c; return TX_SUCCESS; }
void board_watchdog_progress(uint32_t m) { (void)m; }
void thread_comm_estimator_enter(void) {}
void thread_comm_estimator_leave(void) {}
void thread_comm_estimator_sleep(ULONG ticks) { _tx_thread_sleep(ticks); }
void distribution_wake(void) {}
bool fc_recovery_eval_pause_requested(void) { return false; }
void fc_recovery_evaluation_quiescent(void) {}
void fc_recovery_evaluation_running(void) {}
void fc_recovery_ota_reset_retire_locked(void) {}
bool fc_recovery_postinit_allowed(uint32_t e) { (void)e; return true; }
void fc_sd_capture_event(unsigned e, bool r) { (void)e; (void)r; }
void sd_append_f32(SedsDataType t, const float *v, fu8 n) { (void)t; (void)v; (void)n; }
void sd_append_string(SedsDataType t, const char *s) { (void)t; printf("[%9.3f] %s\n", replay_s(), s); }
void sd_conclude(void) {}
void sd_launch_landed(uint32_t t) { (void)t; }
void deployment_pulse_start(bool reef, bool manual)
{
  (void)manual;
  g_conf |= reef ? option(Parachute_Expanded) : option(Parachute_Deployed);
  if (reef) { main_t = replay_s(); main_alt = svec(0).alt; }
  else { drogue_t = replay_s(); drogue_alt = svec(0).alt; }
  printf("[%9.3f] PYRO %s (dry-run stub) at estimated %.1f m\n", replay_s(), reef ? "main" : "drogue", svec(0).alt);
}

void kalman_seed_attitude(const f_xyz *accl);

static const char *names[] = {"Startup", "Postinit", "Armed", "Launch", "Ascent", "Coast", "Apogee",
                              "Descent", "Reefing", "Landed", "Recovery"};

/* recovery.c baro_conf.odr = Baro_ODR_25: one conversion every 40 ms. The FSM
 * counts samples (MIN_SAMP_*, 8-entry history), so the period sets its timing. */
#define BARO_PERIOD_MS 40U
int main(int argc, char **argv)
{
  snprintf(igt_root, sizeof igt_root, "%s", argv[1]);
  if (argc > 4) {
    g_launch_site[0] = strtof(argv[3], NULL); g_launch_site[1] = strtof(argv[4], NULL);
    g_launch_site[2] = strtof(argv[5], NULL);
  }
  trace = argc > 2 && argv[2][0] ? fopen(argv[2], "w") : NULL;
  if (trace) fprintf(trace, "t_s,state,alt,vel,baro_alt,q0,q1,q2,q3\n");
  kfpool_buf = _sbrk(16 + KF_POOL_SIZE);
  host_now = 1000;
  if (!igt_command(IGT_CMD_ENTER, true, false)) return 2;
  igt_sd_service();
  if (((igt_health_bits() >> 4) & 7U) != IGT_READY) { printf("not ready\n"); return 3; }

  /* Postinit on the stand left the FC Armed; Launch at rest seeds the replay's attitude. */
  sm.flight = Armed;
  host_now = 2000;
  host_t0 = host_now;
  const float rest_a[3] = {0, 0, GRAVITY_SI}, rest_g[3] = {0, 0, 0};
  float seed[3];
  if (!igt_launch(host_now, rest_a, rest_g, 0.0f, seed)) { printf("launch refused\n"); return 4; }
  kalman_seed_attitude((const f_xyz *)seed);
  fu32 conf = load(&g_conf, Acq);
  ascent_initialize(conf);
  g_conf |= option(Launch_Requested);

  state last = current();
  float dt = 0, max_alt = -1e9f, max_alt_t = 0, max_vel = -1e9f, max_vel_t = 0;
  bool predicted = false, rail_set = false;
  kf_gps rail = {0};
  float times[Flight_States];
  for (unsigned i = 0; i < Flight_States; ++i) times[i] = -1;
  uint32_t end_after = 0;
  printf("[%9.3f] Launch accepted; replay t=0\n", 0.0);
  for (;;) {
    ++host_now;
    if (host_now % 50U == 0) igt_sd_service();  /* IGT_SD_PERIOD_MS = LOG_RATE_SD */
    float a[3] = {0, 0, GRAVITY_SI}, g[3] = {0, 0, 0}, b[3] = {0, 101325, 20};
    /* The DMA callbacks replace every real sample with the replay. */
    const bool baro_due = host_now % BARO_PERIOD_MS == 0U;
    if (baro_due) igt_override_baro(b, host_now);
    if (!igt_sim_running() && end_after == 0) end_after = host_now + 6000;
    if (end_after && host_now >= end_after) break;
    if (replay_s() > 1500) break;
    if (host_now < host_wake) continue;
    conf = load(&g_conf, Acq);
    if (conf & option(Using_Ascent_KF)) {
      if (!predicted && host_now % BARO_PERIOD_MS == 1U) {
        igt_override_accl(a, host_now); igt_override_gyro(g, host_now);
        memcpy(&meas.accl, a, sizeof a); memcpy(&meas.gyro, g, sizeof g);
        dt = fsec(timer_exchange(AscentKF));
        ascent_predict(dt, conf);
        predicted = true;
      } else if (predicted && baro_due) {
        memcpy(&meas.baro, b, sizeof b);
        ascent_update();
        predicted = false;
        evaluate_rocket_state(conf, dt);
      } else continue;
    } else if (baro_due) {
      memcpy(&meas.baro, b, sizeof b);
      kf_gps fix;
      if (igt_gps_fix(&fix.sea, host_now)) {
        if (!rail_set) { rail = fix; rail_set = true; }
        meas.gps.sea = fix.sea;
        meas.gps.lon = fabsf(fix.lon - rail.lon);
        meas.gps.lat = fabsf(fix.lat - rail.lat);
      }
      dt = fsec(timer_exchange(DescentKF));
      descent_predict(dt);
      descent_update();
      evaluate_rocket_state(conf, dt);
    } else continue;
    /* evaluate_rocket_state advanced sm.idx: the newest estimate is svec(1). */
    const kf_svec *s = &svec(1);
    if (s->alt > max_alt) { max_alt = s->alt; max_alt_t = replay_s(); }
    if (s->vel > max_vel) { max_vel = s->vel; max_vel_t = replay_s(); }
    if (trace) fprintf(trace, "%.3f,%u,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f\n", replay_s(), (unsigned)current(),
                       s->alt, s->vel, meas.baro.alt, qv.q0, qv.rho1, qv.rho2, qv.rho3);
    state now = current();
    while (last < now) { ++last; times[last] = replay_s(); }
  }
  printf("RESULT {");
  for (unsigned i = Launch; i < Flight_States; ++i) printf("\"%s_s\": %.3f, ", names[i], times[i]);
  printf("\"drogue_s\": %.3f, \"drogue_est_alt_m\": %.2f, \"main_s\": %.3f, \"main_est_alt_m\": %.2f, "
         "\"est_apogee_m\": %.2f, \"est_apogee_s\": %.3f, \"est_max_vel_mps\": %.2f, \"est_max_vel_s\": %.3f, "
         "\"kf_deviations\": %d, \"igt_mode\": %u, \"igt_reason\": %u}\n",
         drogue_t, drogue_alt, main_t, main_alt, max_alt, max_alt_t, max_vel, max_vel_t,
         (int)sm.kf_deviations, (unsigned)((igt_health_bits() >> 4) & 7U), (unsigned)((igt_health_bits() >> 12) & 15U));
  if (trace) fclose(trace);
  return 0;
}
'''

DSP = ['MatrixFunctions/arm_mat_add_f32.c', 'MatrixFunctions/arm_mat_sub_f32.c', 'MatrixFunctions/arm_mat_mult_f32.c',
       'MatrixFunctions/arm_mat_trans_f32.c', 'MatrixFunctions/arm_mat_inverse_f32.c',
       'MatrixFunctions/arm_mat_scale_f32.c', 'MatrixFunctions/arm_mat_vec_mult_f32.c',
       'FastMathFunctions/arm_sin_f32.c', 'FastMathFunctions/arm_cos_f32.c', 'FastMathFunctions/arm_atan2_f32.c',
       'CommonTables/arm_common_tables.c']


def build(work: Path) -> Path:
    inc = ['Core/Inc', 'Core/Test', 'Drivers/STM32H5xx_HAL_Driver/Inc', 'Drivers/CMSIS/Device/ST/STM32H5xx/Include',
           'Drivers/CMSIS/Include', 'Drivers/CMSIS/DSP/Include', 'Middlewares/ST/threadx/common/inc',
           'Middlewares/ST/threadx/ports/cortex_m33/gnu/inc', 'Middlewares/ST/filex/common/inc',
           'Middlewares/ST/filex/ports/generic/inc', 'FileX/App', 'FileX/Target']
    defs = ['-DSTM32H523xx', '-DUSE_HAL_DRIVER', '-DTX_INCLUDE_USER_DEFINE_FILE', '-DFX_INCLUDE_USER_DEFINE_FILE',
            '-DNDEBUG', '-DFC_GROUND_TEST=1', '-DBOARD_WATCHDOG_ENABLE=0', '-DSD_AVAILABLE', '-DGPS_AVAILABLE']
    base = ['gcc', '-m32', '-msse2', '-mfpmath=sse', '-std=gnu11', '-O2', '-ffast-math', '-w']
    (work / 'pre.h').write_text(PRE)
    (work / 'ground_test_host_port.h').write_text(PORT)
    (work / 'driver.c').write_text(DRIVER)
    incs = [f'-I{ROOT / d}' for d in inc]
    objs = []
    fw = base + ['-include', str(work / 'pre.h')] + defs + incs
    jobs = [(fw, ROOT / 'Core/Src/kalman.c'), (fw, ROOT / 'Core/Src/evaluation.c'), (fw, work / 'driver.c')]
    jobs += [(fw, ROOT / 'Drivers/CMSIS/DSP/Source' / s) for s in DSP]
    jobs.append((base + ['-DIGT_HOST_TEST', '-DFC_GROUND_TEST=1', f'-I{work}', f'-I{ROOT / "Core/Inc"}'],
                 ROOT / 'Core/Src/ground_test.c'))
    for flags, src in jobs:
        obj = work / (src.stem + '.o')
        subprocess.run(flags + ['-c', str(src), '-o', str(obj)], check=True)
        objs.append(str(obj))
    exe = work / 'igt_replay'
    subprocess.run(['gcc', '-m32', '-o', str(exe), *objs, '-lm'], check=True)
    return exe


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', type=Path, help='folder holding IGT/*.CSV (or the IGT folder itself)')
    ap.add_argument('--site', help="the FC's launch site lat,lon,msl (default: compiled)")
    ap.add_argument('--json', type=Path, help='write the result here')
    ap.add_argument('--trace', type=Path, help='per-cycle estimator trace CSV')
    ap.add_argument('--quiet', action='store_true', help='only the result line')
    a = ap.parse_args(argv)
    folder = a.folder.resolve()
    root = folder.parent if folder.name.upper() == 'IGT' else folder
    work = Path(tempfile.mkdtemp(prefix='igt-replay-'))
    try:
        exe = build(work)
        cmd = [str(exe), str(root), str(a.trace.resolve()) if a.trace else '']
        if a.site:
            cmd += a.site.split(',')
        out = subprocess.run(cmd, capture_output=True, text=True)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if out.returncode != 0:
        print(out.stdout + out.stderr, file=sys.stderr)
        return out.returncode
    result = None
    for line in out.stdout.splitlines():
        if line.startswith('RESULT '):
            result = json.loads(line[7:])
        elif not a.quiet:
            print(line)
    print(json.dumps(result, indent=2))
    if a.json:
        a.json.write_text(json.dumps(result, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

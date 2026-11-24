#!/usr/bin/env python3
"""Sweep the break-up (RUD) abort detector over flight data.

Builds the IGT host replay once (tools/igt_replay_host.py: the real
estimator, state machine and rud_detect.c) and replays

  * every IGT dataset as given,
  * each of them with injected faults that an intact flight can show (baro
    outliers and bursts, a frozen or missing barometer, IMU saturation
    transients and a stuck saturated IMU, deployment and landing shocks, GPS
    loss, heavy noise): the detector must never fire, and the drogue and
    main must still come;
  * synthetic break-ups (tumbling with lateral loads and a wild static
    pressure, at max-Q and in coast): the detector must fire before Apogee;
  * break-up signatures on one sensor only (IMU or barometer): it must not
    fire.

    python3 tools/rud_sweep.py <GroundStation26>/test-data/igt [--json out.json]

Prints one line per case and the margins: the largest vote counts and raw
maxima the detector saw while armed, against its thresholds (rud_detect.h).
Exits non-zero if any case has the wrong outcome.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import igt_replay_host  # noqa: E402

ROOT = HERE.parent
G = 9.80665
ACC_MAX = 24.0 * G - 0.01   # just inside the IGT row limits (fcconfig.h MAX_ACC / MAX_DPS / ALT)
DPS_MAX = 1999.9
ALT_MIN, ALT_MAX = -29.0, 4571.0   # one unit inside the row limits
PRS_MIN, PRS_MAX = 57201.0, 109999.0


def thresholds() -> dict:
    text = (ROOT / 'Core/Inc/rud_detect.h').read_text()
    return {k: float(re.search(rf'#define {k}\s+([0-9.]+)', text).group(1))
            for k in ('RUD_TUMBLE_DPS', 'RUD_LATERAL_MPS2', 'RUD_PRESSURE_RATE_MPS',
                      'RUD_WINDOW', 'RUD_MOTION_MIN', 'RUD_PRESSURE_MIN')}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Data:
    def __init__(self, folder: Path):
        self.summary = json.loads((folder / 'summary.json').read_text())
        self.rows = {}
        for name in ('IMU', 'BARO', 'GPS'):
            path = folder / 'IGT' / f'{name}0.CSV'
            with path.open() as f:
                r = list(csv.reader(f))
            self.rows[name] = (r[0], [[float(x) for x in row] for row in r[1:]])

    def event(self, key: str, default: float) -> float:
        def find(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    if k == key and isinstance(v, (int, float)):
                        return float(v)
                    r = find(v)
                    if r is not None:
                        return r
            return None
        v = find(self.summary)
        return default if v is None else v

    def write(self, folder: Path):
        (folder / 'IGT').mkdir(parents=True)
        for name, (head, rows) in self.rows.items():
            with (folder / 'IGT' / f'{name}0.CSV').open('w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(head)
                for row in rows:
                    t = int(round(row[0]))
                    w.writerow([t] + [f'{v:.4f}' for v in row[1:]])

    def each(self, name, t0, t1, fn):
        for row in self.rows[name][1]:
            t = row[0] / 1000.0
            if t0 <= t < t1:
                fn(row, t)
        self.clamp()

    def drop(self, name, t0, t1):
        head, rows = self.rows[name]
        self.rows[name] = (head, [r for r in rows if not t0 <= r[0] / 1000.0 < t1])

    def clamp(self):
        for row in self.rows['IMU'][1]:
            for i in (1, 2, 3):
                row[i] = clamp(row[i], -ACC_MAX, ACC_MAX)
            for i in (4, 5, 6):
                row[i] = clamp(row[i], -DPS_MAX, DPS_MAX)
        for row in self.rows['BARO'][1]:
            row[1] = clamp(row[1], ALT_MIN, ALT_MAX)
            row[2] = clamp(row[2], PRS_MIN, PRS_MAX)


def baro_shift(row, dh):
    """Move one barometer row by dh metres (pressure consistently, ~12 Pa/m)."""
    row[1] += dh
    row[2] -= dh * 11.0


# Faults an intact flight can show: must never fire.
def f_outliers(d, rng):
    def f(row, t):
        if rng.random() < 0.10:
            baro_shift(row, rng.choice((-1, 1)) * rng.uniform(15, 40))
    d.each('BARO', 10.0, 1e9, f)


def f_bursts(d, rng):
    for c in (d.event('max_speed_s', 15.8), 25.0, 33.0):
        d.each('BARO', c, c + 0.25, lambda row, t: baro_shift(row, 60.0 * (1 if int(t * 50) % 2 else -1)))


def f_baro_freeze(d, rng):
    held = {}
    def f(row, t):
        held.setdefault('v', row[1:3])
        row[1:3] = held['v']
    d.each('BARO', 22.0, 25.0, f)


def f_baro_gap(d, rng):
    d.drop('BARO', 20.0, 22.0)


def f_imu_transient(d, rng):
    for c in (12.0, 18.0, 28.0):
        d.each('IMU', c, c + 0.3, lambda row, t: row.__setitem__(slice(1, 7), [ACC_MAX, -ACC_MAX, ACC_MAX, DPS_MAX, -DPS_MAX, DPS_MAX]))


def f_imu_stuck(d, rng):
    d.each('IMU', 24.0, 27.0, lambda row, t: row.__setitem__(slice(1, 7), [ACC_MAX, ACC_MAX, ACC_MAX, DPS_MAX, DPS_MAX, DPS_MAX]))


def f_imu_transient_and_outliers(d, rng):
    f_imu_transient(d, rng)
    for c in (12.0, 18.0, 28.0):
        d.each('BARO', c, c + 0.3, lambda row, t: baro_shift(row, 40.0 * (1 if int(t * 50) % 2 else -1)))


def f_shocks(d, rng):
    for key, dflt in (('apogee_s', 36.6), ('main_s', 141.0), ('landing_s', 204.0)):
        c = d.event(key, dflt) + (1.2 if key == 'apogee_s' else 0.3 if key == 'main_s' else 0.0)
        d.each('IMU', c, c + 0.2, lambda row, t: row.__setitem__(slice(1, 7), [ACC_MAX, -ACC_MAX, -ACC_MAX, DPS_MAX, DPS_MAX, -DPS_MAX]))
        d.each('BARO', c, c + 0.2, lambda row, t: baro_shift(row, -110.0))


def f_gps_loss(d, rng):
    d.drop('GPS', 10.0, 1e9)


def f_noise(d, rng):
    d.each('BARO', 0.0, 1e9, lambda row, t: baro_shift(row, rng.gauss(0, 3.0)))
    def f(row, t):
        for i in (1, 2, 3):
            row[i] += rng.gauss(0, 2.0)
        for i in (4, 5, 6):
            row[i] += rng.gauss(0, 15.0)
    d.each('IMU', 0.0, 1e9, f)


FAULTS = {
    'baro_outliers_10pct': f_outliers, 'baro_bursts': f_bursts, 'baro_freeze_3s': f_baro_freeze,
    'baro_gap_2s': f_baro_gap, 'imu_saturation_0.3s': f_imu_transient, 'imu_stuck_saturated_3s': f_imu_stuck,
    'imu_saturation_and_baro_bursts': f_imu_transient_and_outliers, 'deploy_and_landing_shocks': f_shocks,
    'gps_loss': f_gps_loss, 'heavy_noise': f_noise,
}


# Break-ups.
def tumble(d, t0, imu=True, baro=True, rate=900.0, lateral=120.0, port=600.0):
    if imu:
        def f(row, t):
            ph = 2 * math.pi * 2.5 * (t - t0)
            row[1:7] = [lateral * math.cos(ph), lateral * math.sin(ph), row[3] * 0.3 + 30 * math.sin(3 * ph),
                        rate * math.cos(ph), rate * math.sin(ph), row[6] + 200.0]
        d.each('IMU', t0, 1e9, f)
    if baro:
        d.each('BARO', t0, 1e9, lambda row, t: baro_shift(row, port * math.sin(2 * math.pi * 3.0 * (t - t0))))


def r_maxq(d, rng):
    tumble(d, d.event('max_speed_s', 15.8))


def r_coast(d, rng):
    tumble(d, 25.0)


def r_pressure_walk(d, rng):
    tumble(d, 22.0, baro=False, rate=500.0, lateral=20.0)
    walk = {'h': 0.0}
    def f(row, t):
        walk['h'] += rng.choice((-1, 1)) * rng.uniform(25, 60)
        baro_shift(row, walk['h'])
    d.each('BARO', 22.0, 1e9, f)


def s_imu_only(d, rng):
    tumble(d, 25.0, baro=False)


def s_baro_only(d, rng):
    tumble(d, 25.0, imu=False)


def s_imu_stuck(d, rng):
    """The IMU freezes at rest-like readings from coast to the end."""
    d.each('IMU', 22.0, 1e9, lambda row, t: row.__setitem__(slice(1, 7), [0.1, -0.1, G, 0.5, -0.5, 0.2]))


def s_imu_bias(d, rng):
    """The accelerometer's long axis gains a 3 g bias from coast on."""
    d.each('IMU', 22.0, 1e9, lambda row, t: row.__setitem__(3, row[3] + 3 * G))


def s_baro_stuck_descent(d, rng):
    """The barometer freezes 10 s after apogee for the rest of the flight."""
    held = {}
    def f(row, t):
        held.setdefault('v', row[1:3])
        row[1:3] = held['v']
    d.each('BARO', d.event('apogee_s', 36.6) + 10.0, 1e9, f)


def s_baro_noise_descent(d, rng):
    """A failing barometer: 200 m white noise from apogee on."""
    d.each('BARO', d.event('apogee_s', 36.6), 1e9, lambda row, t: baro_shift(row, rng.gauss(0, 200.0)))


BREAKUPS = {'breakup_at_max_q': r_maxq, 'breakup_in_coast': r_coast, 'tumble_with_pressure_jumps': r_pressure_walk}
# A break-up seen by one sensor, or one sensor failing outright: no abort,
# and the other sensors must still bring both chutes at sane times.
ONE_SENSOR = {'imu_only_tumble': s_imu_only, 'baro_only_chaos': s_baro_only, 'imu_stuck_rest': s_imu_stuck,
              'imu_accel_bias_3g': s_imu_bias, 'baro_stuck_in_descent': s_baro_stuck_descent,
              'baro_noise_200m_in_descent': s_baro_noise_descent}


# Barometer failures in descent, with GPS absent or stale (docs/rud-abort.md
# section 6). Each takes the onset time t0; an absent GPS delivers nothing
# (IGT_SILENT, tools/igt_replay_host.py); a stale GPS keeps sending its last
# fix (the replay holds the last row: the rows after t0 are dropped).
def x_baro_frozen(d, rng, t0):
    held = {}
    def f(row, t):
        held.setdefault('v', row[1:3])
        row[1:3] = held['v']
    d.each('BARO', t0, 1e9, f)


def x_baro_wild(d, rng, t0):
    d.each('BARO', t0, 1e9, lambda row, t: baro_shift(row, rng.gauss(0, 200.0)))


def x_baro_offset(dh):
    def f(d, rng, t0):
        d.each('BARO', t0, 1e9, lambda row, t: baro_shift(row, dh))
    return f


def x_gps_stale(d, rng, t0):
    d.drop('GPS', t0, 1e9)


# name -> (data faults [(fn, onset key)], silent sensors [(sensor, onset key)],
#          what is expected: drogue 'sensor' (within [-1, +4] s of the true
#          apogee); main 'measured' (barometer or GPS: true altitude 457 +/-
#          100 m), 'dr' (dead reckoning from the true altitude at 'ref':
#          457 - 100 m up to 457 + 25 % of the descent from there + 100 m) or
#          'offset' (the barometer's offset is unobservable: 457 - offset +/-
#          100 m).
# Onsets: 'apogee', 'apogee+10', 'apogee+20', 'main-15' (about 830 m),
# 'apogee-5' (GPS).
def baro_descent_cases():
    faults = {
        'baro_frozen_at_apogee': ([(x_baro_frozen, 'apogee')], [], 'dr', 'apogee'),
        'baro_frozen_apogee+10': ([(x_baro_frozen, 'apogee+10')], [], 'dr', 'apogee+10'),
        'baro_frozen_at_830m': ([(x_baro_frozen, 'main-15')], [], 'dr', 'main-15'),
        'baro_wild_at_apogee': ([(x_baro_wild, 'apogee')], [], 'dr', 'apogee'),
        'baro_wild_apogee+10': ([(x_baro_wild, 'apogee+10')], [], 'dr', 'apogee+10'),
        'baro_offset+100m': ([(x_baro_offset(100.0), 'apogee+20')], [], ('offset', 100.0), None),
        'baro_offset-100m': ([(x_baro_offset(-100.0), 'apogee+20')], [], ('offset', -100.0), None),
    }
    gps = {'gps_absent': ([], [('gps', 'apogee-5')]), 'gps_stale': ([(x_gps_stale, 'apogee-5')], [])}
    out = {}
    for fname, (data, silent, main, ref) in faults.items():
        for gname, (gdata, gsilent) in gps.items():
            out[f'{fname}+{gname}'] = (data + gdata, silent + gsilent, 'sensor', main, ref)
    return out


def main_crossing(src: Path, apogee_s: float) -> float:
    """When the dataset's own barometer (0.5 s median) first falls below
    457.2 m after apogee: the FC's main trigger, whatever the scripted main
    of the variant (off_nominal opens its chute late, at 300 m)."""
    with (src / 'IGT' / 'BARO0.CSV').open() as f:
        rows = [(int(r['t_ms']) / 1000.0, float(r['alt_m'])) for r in csv.DictReader(f)]
    for i, (t, _) in enumerate(rows):
        if t <= apogee_s:
            continue
        window = sorted(a for tt, a in rows[max(0, i - 12):i + 13])
        if window[len(window) // 2] < 457.2:
            return t
    return float('inf')


def true_alt(src: Path):
    """The dataset's own altitude (0.5 s median of its barometer) at t."""
    with (src / 'IGT' / 'BARO0.CSV').open() as f:
        rows = [(int(r['t_ms']) / 1000.0, float(r['alt_m'])) for r in csv.DictReader(f)]
    times = [t for t, _ in rows]
    import bisect
    def at(t):
        i = bisect.bisect_left(times, t)
        window = sorted(a for _, a in rows[max(0, i - 12):i + 13])
        return window[len(window) // 2] if window else float('nan')
    return at


def onset(key: str, truth: dict) -> float:
    base = {'coast': 22.0, 'apogee': truth['apogee_s'], 'main': truth['main_trigger_s']}
    for k, v in base.items():
        if key.startswith(k):
            return v + float(key[len(k):] or 0.0)
    raise KeyError(key)


def run(exe: Path, folder: Path, silent: str = '') -> dict:
    out = subprocess.run([str(exe), str(folder), ''], capture_output=True, text=True,
                         env=dict(os.environ, IGT_RUD_MARGINS='1', IGT_SILENT=silent), timeout=600)
    for line in out.stdout.splitlines():
        if line.startswith('RESULT '):
            return json.loads(line[7:])
    raise RuntimeError(f'{folder}: no result\n{out.stdout[-2000:]}{out.stderr[-2000:]}')


def judge(r: dict, truth: dict, alt_at, want: str, drogue: str, main: str, ref) -> tuple[bool, str]:
    """The outcome against the dataset's truth (it is replayed open loop).

    Drogue 'sensor': within [-1, +4] s of the true apogee (1 s early is under
    10 m/s; the sensor path itself lags 0.8-2.3 s), and the Apogee state not
    more than 2 s before the true apogee (a false Apogee switches to the
    descent filter: reported as a warning, see early_apogee_state).
    Main: the true altitude when it fires (the dataset's own barometer).
    'measured' 457 +/- 100 m (4 s under the drogue); 'dr' from 357 m up to
    457 m plus 25 % of the descent from the reference altitude (the fast
    bias, 15 % on a rate that may be 15 % slow) plus 100 m; 'offset' the
    unobservable offset moves it: 457 - offset +/- 100 m. 'abort': the
    break-up detector fires before Apogee."""
    fired = 'rud_abort_s' in r
    ap = truth['apogee_s']
    if want == 'fire' or main == 'abort':
        return fired and (r.get('Apogee_s', -1) < 0 or r['rud_abort_s'] < r['Apogee_s']), 'abort before Apogee'
    if fired:
        return False, 'aborted'
    d, m = r.get('drogue_s', -1), r.get('main_s', -1)
    if d < 0 or m < 0:
        return False, 'no drogue' if d < 0 else 'no main'
    ok_d = ap - 1.0 <= d <= ap + 4.0
    h = alt_at(m)
    if main == 'measured':
        ok_m = abs(h - 457.2) <= 100.0
    elif main == 'dr':
        h_ref = alt_at(ref)
        ok_m = 357.2 <= h <= 457.2 + 0.25 * max(0.0, h_ref - 457.2) + 100.0
    else:
        ok_m = abs(h - (457.2 - main[1])) <= 100.0
    note = '' if ok_d else 'drogue '
    note += '' if ok_m else 'main'
    return ok_d and ok_m, note.strip()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('igt', type=Path, help='GroundStation26 test-data/igt')
    ap.add_argument('--json', type=Path)
    a = ap.parse_args(argv)
    th = thresholds()
    sets = sorted(p for p in a.igt.iterdir() if (p / 'IGT').is_dir())
    work = Path(tempfile.mkdtemp(prefix='rud-sweep-'))
    results, bad = [], 0
    try:
        (work / 'build').mkdir()
        exe = igt_replay_host.build(work / 'build')
        # (set, case, data faults, silent sensors, want, drogue, main, ref)
        cases = []
        for base in sets:
            cases.append((base.name, 'as given', None, [], 'quiet', 'sensor', 'measured', None))
            for name, fn in FAULTS.items():
                cases.append((base.name, name, fn, [], 'quiet', 'sensor', 'measured', None))
        nominal = [p for p in sets if p.name in ('nominal_10k', 'measured_motor', 'high_wind', 'off_nominal')]
        for base in nominal:
            for name, fn in BREAKUPS.items():
                cases.append((base.name, name, fn, [], 'fire', None, 'abort', None))
            for name, fn in ONE_SENSOR.items():
                cases.append((base.name, name, fn, [], 'nofire', 'sensor', 'measured', None))
        for base in sets:
            for name, (data, silent, drogue, main_kind, ref) in baro_descent_cases().items():
                cases.append((base.name, name, data, silent, 'baro_descent', drogue, main_kind, ref))
        for i, (set_name, case, fn, silent, want, drogue, main_kind, ref) in enumerate(cases):
            folder = work / f'case{i}'
            src = a.igt / set_name
            truth = json.loads((src / 'summary.json').read_text())
            truth['main_trigger_s'] = main_crossing(src, truth['apogee_s'])
            rng = random.Random(f'{set_name}/{case}')
            if fn is None:
                shutil.copytree(src / 'IGT', folder / 'IGT')
            else:
                d = Data(src)
                if callable(fn):
                    fn(d, rng)
                else:
                    for f, key in fn:
                        f(d, rng, onset(key, truth))
                d.write(folder)
            spec = ','.join(f'{s}:{onset(key, truth):.3f}:1e9' for s, key in silent)
            r = run(exe, folder, spec)
            fired = 'rud_abort_s' in r
            ok, why = judge(r, truth, true_alt(src), want, drogue, main_kind,
                            onset(ref, truth) if ref else None)
            bad += not ok
            early = r.get('Apogee_s', -1) > 0 and r['Apogee_s'] < truth['apogee_s'] - 2.0 and not fired
            h_main = true_alt(src)(r['main_s']) if r.get('main_s', -1) > 0 else float('nan')
            row = {'set': set_name, 'case': case, 'want': want, 'fired': fired, 'ok': ok, 'why': why,
                   'expect_drogue': drogue, 'expect_main': main_kind if isinstance(main_kind, str) else 'offset',
                   'silent': spec, 'true_apogee_s': truth['apogee_s'], 'true_main_trigger_s': truth['main_trigger_s'],
                   'main_true_alt_m': h_main, 'early_apogee_state': early, **r}
            results.append(row)
            if want == 'baro_descent':
                print(f"{'ok ' if ok else 'BAD'} {set_name:18} {case:44} "
                      f"{'FIRED at %.2f s' % r['rud_abort_s'] if fired else 'no abort':16} "
                      f"drogue {r.get('drogue_s', -1) - truth['apogee_s']:+6.2f} s after apogee, "
                      f"main {r.get('main_s', -1e9) - truth['main_trigger_s']:+7.2f} s from 457 m, at a true {h_main:6.0f} m"
                      f"{'  [' + why + ']' if why and not ok else ''}", flush=True)
                continue
            print(f"{'ok ' if ok else 'BAD'} {set_name:18} {case:32} want={want:6} "
                  f"{'FIRED at %.2f s' % r['rud_abort_s'] if fired else 'no abort':16} "
                  f"motion {r['rud_max_motion']:2}/{int(th['RUD_MOTION_MIN'])} pressure {r['rud_max_pressure']:2}/{int(th['RUD_PRESSURE_MIN'])} "
                  f"|w_xy| {r['rud_max_gyro_xy_dps']:6.0f} |a_xy| {r['rud_max_accel_xy_mps2']:5.1f} "
                  f"baro {r['rud_max_baro_rate_mps']:6.0f} joint {r['rud_max_joint_pct']:3}% "
                  f"drogue {r.get('drogue_s', -1) - truth['apogee_s']:+6.2f} s after apogee, "
                  f"main {r.get('main_s', -1e9) - truth['main_trigger_s']:+6.2f} s from 457 m",
                  flush=True)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print('raw maxima on the datasets as given (armed, Launch..Coast; a single step gives one vote):')
    for r in results:
        if r['case'] == 'as given':
            print(f"  {r['set']:18} |gyro x,y| {r['rud_max_gyro_xy_dps']:6.0f} deg/s (vs {th['RUD_TUMBLE_DPS']:.0f}), "
                  f"|accel x,y| {r['rud_max_accel_xy_mps2']:5.1f} m/s^2 (vs {th['RUD_LATERAL_MPS2']:.0f}), "
                  f"baro {r['rud_max_baro_rate_mps']:5.0f} m/s (vs {th['RUD_PRESSURE_RATE_MPS']:.0f}); "
                  f"votes motion {r['rud_max_motion']}/{int(th['RUD_MOTION_MIN'])}, pressure {r['rud_max_pressure']}/{int(th['RUD_PRESSURE_MIN'])}")
    faults = [r for r in results if r['want'] != 'fire']
    worst = max(faults, key=lambda r: r['rud_max_joint_pct'])
    print(f"  over all {len(faults)} must-not-fire cases: the closest both votes came at once is "
          f"{worst['rud_max_joint_pct']}% of firing ({worst['set']}, {worst['case']}); "
          f"motion needs {int(th['RUD_MOTION_MIN'])} and pressure {int(th['RUD_PRESSURE_MIN'])} of {int(th['RUD_WINDOW'])} steps")
    print('matrix (failure -> drogue after the true apogee, main at a true altitude; min..max over the datasets):')
    by_case = {}
    for r in results:
        if r['want'] == 'baro_descent':
            by_case.setdefault(r['case'], []).append(r)
    for case, rows in by_case.items():
        dd = [r['drogue_s'] - r['true_apogee_s'] for r in rows]
        hm = [r['main_true_alt_m'] for r in rows]
        print(f"  {case:44} drogue {min(dd):+6.2f}..{max(dd):+6.2f} s, main at {min(hm):5.0f}..{max(hm):5.0f} m"
              f"  {sum(r['ok'] for r in rows)}/{len(rows)} ok")
    early = [r for r in results if r['early_apogee_state']]
    print(f"warning: the Apogee state (the switch to the descent filter) more than 2 s before the true apogee in "
          f"{len(early)} cases; the drogue still waits for the descent filter to see the fall:")
    for r in early:
        print(f"  {r['set']:18} {r['case']:44} Apogee state {r['Apogee_s'] - r['true_apogee_s']:+6.2f} s, "
              f"drogue {r['drogue_s'] - r['true_apogee_s']:+5.2f} s")
    print(f"{len(results) - bad}/{len(results)} cases as expected")
    if a.json:
        a.json.write_text(json.dumps(results, indent=1) + '\n')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

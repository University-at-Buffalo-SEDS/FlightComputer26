#!/usr/bin/env python3
"""Read-only FC barometer plots over ST-Link; no telemetry or Python packages needed."""
from __future__ import annotations
import argparse
from collections import deque
import csv
from dataclasses import dataclass
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import webbrowser

FLASH = (0x08000000, 0x08080000)
RAM = (0x20000000, 0x20044000)
SOURCE_ROOT = Path(__file__).resolve().parent.parent
DHCSR = 0xE000EDF0  # Read only: S_HALT bit; never written.

def validate_layout(root):
    """Reject same-size field reorders; -g0 ELF has no member-level DWARF."""
    source = (Path(root)/'Core/Inc/fcstructs.h').read_text()
    expected = {'kf_gps':'float sea, lon, lat;', 'baro':'float alt, prs, tmp;',
                'f_xyz':'float x, y, z;',
                'measm':'kf_gps gps; baro baro; f_xyz accl; f_xyz gyro;',
                'ekf_bias':'float az, gx, gy, gz;',
                'kf_svec':'kf_gps gps; float alt, vel; ekf_bias bias;'}
    for alias,fields in expected.items():
        match=re.search(r'typedef\s+struct\s+serial\s+\w+\s*\{([^}]+)\}\s*'+alias+r'\s*;',source,re.S)
        if not match or re.sub(r'\s+','',match[1]) != re.sub(r'\s+','',fields):
            raise ValueError(f'{alias}: unsupported source layout/reordered fields; update viewer for this firmware ABI')

@dataclass(frozen=True)
class Symbol:
    address: int
    size: int

class Firmware:
    """ELF32 ARM symbols and initialized loadable sections, including RAM LMAs."""
    def __init__(self, path):
        self.path = Path(path).resolve()
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if len(raw) < 52 or raw[:7] != b'\x7fELF\x01\x01\x01':
            raise ValueError('Expected a little-endian ELF32 ARM firmware image')
        h = struct.unpack_from('<HHIIIIIHHHHHH', raw, 16)
        if h[1] != 40 or h[0] != 2:
            raise ValueError('Expected a linked ARM executable, not an object/host binary')
        _, _, _, _, phoff, shoff, _, _, phsize, phnum, shsize, shnum, _ = h
        if phsize != 32 or shsize != 40 or not shnum:
            raise ValueError('ELF lacks supported section/program tables')
        def region(offset, size):
            if offset < 0 or size < 0 or offset + size > len(raw):
                raise ValueError('Truncated ELF table or section')
            return raw[offset:offset+size]
        sections = [struct.unpack('<10I', region(shoff+i*shsize, shsize)) for i in range(shnum)]
        segments = [struct.unpack('<8I', region(phoff+i*phsize, phsize)) for i in range(phnum)]
        self.symbols = {}
        for section in sections:
            _, kind, _, _, offset, size, link, _, _, entsize = section
            if kind != 2:
                continue
            if entsize != 16 or size % 16 or link >= len(sections):
                raise ValueError('Unsupported ELF symbol table')
            strings = sections[link]
            names = region(strings[4], strings[5])
            for i in range(size//16):
                name, address, length, info, _, index = struct.unpack('<IIIBBH', region(offset+i*16,16))
                if info & 15 != 1 or not index or name >= len(names):
                    continue
                end = names.find(b'\0', name)
                if end < 0:
                    raise ValueError('Unterminated ELF symbol name')
                label = names[name:end].decode('utf-8', errors='strict')
                self.symbols.setdefault(label, []).append(Symbol(address, length))
        self.flash = []
        for section in sections:
            _, kind, flags, address, offset, size, *_ = section
            if not flags & 2 or kind == 8 or not size:
                continue
            matching = [p for p in segments if p[0] == 1 and p[2] <= address
                        and address+size <= p[2]+p[4]]
            if len(matching) != 1:
                raise ValueError('Cannot resolve a loadable ELF section to its flash address')
            p = matching[0]
            lma = p[3] + address - p[2]
            if FLASH[0] <= lma and lma+size <= FLASH[1]:
                self.flash.append((lma, region(offset, size)))
            else:
                raise ValueError('Initialized section outside supported FC flash; refusing incomplete verification')
        self.flash.sort()
        if not self.flash:
            raise ValueError('No initialized FC flash sections found')
        # Merge only adjacent sections. ELF/PT_LOAD header padding is not firmware.
        merged = []
        for address, data in self.flash:
            if merged and address < merged[-1][0]+len(merged[-1][1]):
                raise ValueError('Overlapping initialized ELF sections')
            if merged and address == merged[-1][0]+len(merged[-1][1]):
                merged[-1] = (merged[-1][0], merged[-1][1]+data)
            else:
                merged.append((address,data))
        self.flash = merged
        self.required = {'meas':48, 'uwTick':4, 'last_tick':4}
        self.optional = {'gnd_lvl_pressure':4,'alt_bias':4,'prev_rel_alt':4,'imedsv':36}
        self.resolved = {}
        for name, size in (self.required | self.optional).items():
            matches = self.symbols.get(name, []) or [v for n, vals in self.symbols.items()
                      if n.startswith(name+'.lto_priv.') for v in vals]
            matches = list(set(matches))
            if not matches and name in self.optional:
                continue
            if len(matches) != 1 or matches[0].size != size:
                raise ValueError(f'{name}: expected one {size}-byte symbol; incompatible/stripped ELF')
            sym = matches[0]
            if sym.address % 4 or not RAM[0] <= sym.address < sym.address+size <= RAM[1]:
                raise ValueError(f'{name}: symbol is not aligned FC RAM')
            self.resolved[name] = sym

    def verify(self, probe):
        for address, expected in self.flash:
            actual = probe.read_blocks([(address,len(expected))])[0]
            if actual != expected:
                raise ValueError(f'Firmware mismatch at flash region 0x{address:08x}; select the exact flashed ELF')
        return sum(len(data) for _,data in self.flash)

class CubeProbe:
    def __init__(self, executable, serial, frequency=1000, timeout=15, runner=subprocess.run):
        self.executable, self.serial = executable, serial
        self.frequency, self.timeout, self.runner = frequency, timeout, runner

    def read_blocks(self, blocks):
        with tempfile.TemporaryDirectory(prefix='fc-baroscope-read-') as tmp:
            command = [self.executable,'-c','port=SWD','mode=HOTPLUG',f'sn={self.serial}',f'freq={self.frequency}']
            paths = []
            for i,(address,size) in enumerate(blocks):
                if size <= 0 or size > 2*1024*1024:
                    raise ValueError('Invalid bounded memory read size')
                path = Path(tmp)/f'{i}.bin'
                paths.append(path)
                command += ['-u',hex(address),str(size),str(path)]
            try:
                result = self.runner(command, capture_output=True, text=True, timeout=self.timeout)
            except subprocess.TimeoutExpired as e:
                raise RuntimeError('ST-Link read timed out; probe may be busy/disconnected. No takeover/reset attempted.') from e
            if result.returncode or any(not p.exists() for p in paths):
                # Avoid forwarding arbitrary third-party diagnostic output into logs.
                raise RuntimeError('ST-Link read failed: check power, SWD wiring, selected probe, or another debugger. No reset/takeover attempted.')
            values = [p.read_bytes() for p in paths]
            if any(len(value) != size for value,(_,size) in zip(values,blocks)):
                raise RuntimeError('Short ST-Link read; sample discarded')
            return values

    def snapshot(self, firmware):
        names = list(firmware.resolved)
        ranges = sorted([(s.address,s.size) for s in firmware.resolved.values()]+[(DHCSR,4)])
        groups = []
        for address,size in ranges:
            if groups and address <= groups[-1][0]+groups[-1][1]+64 and address+size-groups[-1][0] <= 4096:
                start,length=groups[-1];groups[-1]=(start,max(start+length,address+size)-start)
            else:groups.append((address,size))
        def unpack(blocks,address,size):
            for (start,length),data in zip(groups,blocks):
                if start<=address and address+size<=start+length:return data[address-start:address-start+size]
            raise RuntimeError('Internal read range mismatch')
        values = self.read_blocks(groups+groups)
        first,second=values[:len(groups)],values[len(groups):]
        if any(struct.unpack('<I',unpack(blocks,DHCSR,4))[0] & (1<<17) for blocks in (first,second)):
            raise RuntimeError('CPU halted; not resumed by this viewer')
        a={n:unpack(first,firmware.resolved[n].address,firmware.resolved[n].size) for n in names}
        b={n:unpack(second,firmware.resolved[n].address,firmware.resolved[n].size) for n in names}
        # GPS/IMU members of meas update independently and do not invalidate a
        # stable barometer read. Advancing HAL ticks also are expected.
        stable=[n for n in names if n not in ('uwTick','meas','imedsv')]
        if a['meas'][12:24] != b['meas'][12:24] or any(a[n]!=b[n] for n in stable):
            raise RuntimeError('Inconsistent live barometer read; sample discarded (CPU remains running)')
        if 'imedsv' in b and a['imedsv']!=b['imedsv']:
            del b['imedsv']  # Do not present a visibly changing multi-float prediction as coherent.
        return b

def decode(values):
    def f(name, offset=0):
        if name not in values:
            return None
        result = struct.unpack_from('<f',values[name],offset)[0]
        return result if math.isfinite(result) else None
    def u(name): return struct.unpack('<I',values[name])[0]
    tick, last = u('uwTick'), u('last_tick')
    age = (tick-last) & 0xffffffff
    data = {'pressure_pa':f('meas',16),'temperature_c':f('meas',20),'altitude_m':f('meas',12),
            'baseline_pa':f('gnd_lvl_pressure'),'bias_m':f('alt_bias'),
            'driver_altitude_m':f('prev_rel_alt'),'predicted_altitude_m':f('imedsv',12),
            'predicted_velocity_m_s':f('imedsv',16),'uptime_ms':tick,'sample_tick_ms':last,
            'barometer_age_ms':age if age < 0x80000000 else None}
    if data['pressure_pa'] is None or not 0 < data['pressure_pa'] <= 150000 or data['temperature_c'] is None:
        raise RuntimeError('Barometer not initialized or invalid; sample discarded')
    if data['driver_altitude_m'] is not None and data['bias_m'] is not None:
        data['before_bias_altitude_m'] = data['driver_altitude_m']+data['bias_m']
    else:
        data['before_bias_altitude_m'] = None
    return data

FIELDS = [('pressure_pa','Pressure','Pa'),('temperature_c','Temperature','°C'),
          ('altitude_m','Measured altitude','m'),('before_bias_altitude_m','Before bias (derived)','m'),
          ('bias_m','Altitude bias','m'),('predicted_altitude_m','Kalman prediction altitude','m'),
          ('predicted_velocity_m_s','Kalman prediction vertical velocity','m/s'),('baseline_pa','Pad pressure','Pa')]

PAGE = r'''<!doctype html><meta name="viewport" content="width=device-width,initial-scale=1"><title>FC Barometer</title>
<style>body{font:16px system-ui;background:#101722;color:#e2eaf5;margin:20px}header{max-width:1200px}#status{padding:10px;border:1px solid #688399;border-radius:8px}#panels{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,340px),1fr));gap:12px}article{background:#172435;border:1px solid #42566b;border-radius:10px;padding:12px}h2{font-size:16px;margin:0}strong{display:block;font:26px monospace;margin:8px 0}canvas{width:100%;height:170px}small{color:#a9b8cc}button{padding:10px}</style>
<header><h1>FC Barometer</h1><p id="status">Waiting for ST-Link read…</p><p><small>Read-only HOTPLUG · no telemetry · no reset/halt/resume. Before-bias altitude is derived from the driver altitude + bias. Kalman values are predictions, not final corrected state. Raw ADC counts are unavailable in this build. Reads are best-effort consistent, not atomic.</small></p><p id="build"></p></header><main id="panels"></main>
<script>
let fields=[], busy=false;
function graph(canvas, rows, key){const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=170;canvas.width=w*dpr;canvas.height=h*dpr;const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);const vals=rows.map(r=>r[key]).filter(Number.isFinite);if(!vals.length)return;let lo=Math.min(...vals),hi=Math.max(...vals);if(lo===hi){lo-=.5;hi+=.5}const t0=rows[0].elapsed_s,t1=Math.max(t0+1,rows.at(-1).elapsed_s);c.fillStyle='#a9b8cc';c.font='12px monospace';c.fillText(hi.toFixed(3),0,13);c.fillText(lo.toFixed(3),0,h-3);c.strokeStyle='#63d6c1';c.lineWidth=1.5;c.beginPath();let open=false;for(const row of rows){const v=row[key];if(!Number.isFinite(v)||row.stale){open=false;continue}const x=55+(w-60)*(row.elapsed_s-t0)/(t1-t0),y=18+(h-38)*(1-(v-lo)/(hi-lo));open?c.lineTo(x,y):c.moveTo(x,y);open=true}c.stroke();c.fillText('Last '+(t1-t0).toFixed(0)+' s',w/2-30,h-3)}
async function refresh(){if(busy)return;busy=true;try{const r=await fetch('/data',{cache:'no-store'});const d=await r.json();document.querySelector('#status').textContent=d.status;document.querySelector('#build').textContent=d.build;if(!fields.length){fields=d.fields;for(const [key,label,unit] of fields){const a=document.createElement('article');a.innerHTML='<h2></h2><strong>—</strong><canvas></canvas>';a.querySelector('h2').textContent=label+' ('+unit+')';a.dataset.key=key;document.querySelector('#panels').append(a)}}for(const a of document.querySelectorAll('article')){const v=d.latest?.[a.dataset.key];a.querySelector('strong').textContent=Number.isFinite(v)?v.toFixed(4):'Unavailable';graph(a.querySelector('canvas'),d.rows,a.dataset.key)}}catch(e){document.querySelector('#status').textContent='Viewer disconnected; displayed values are stale'}finally{busy=false}}setInterval(refresh,1000);refresh();
</script>'''

class Scope:
    def __init__(self, firmware, probe, rate, history, writer=None):
        self.firmware,self.probe,self.rate,self.writer = firmware,probe,rate,writer
        self.rows=deque(maxlen=min(20000,max(2,math.ceil(history*rate))))
        self.lock=threading.Lock();self.stop=threading.Event();self.start=time.monotonic()
        self.status='Waiting for first read';self.latest=None;self.verified=True
        self.last_sample=None;self.last_progress=self.start;self.last_uptime=None

    def sample(self):
        now=time.monotonic()
        try:
            row=decode(self.probe.snapshot(self.firmware))
            if self.last_uptime is not None and ((row['uptime_ms']-self.last_uptime)&0xffffffff)>0x80000000:
                self.status='MCU uptime restarted; stopping. Re-run to verify flash again.'
                with self.lock:
                    if self.latest:self.latest=dict(self.latest,stale=True)
                self.stop.set();return
            self.last_uptime=row['uptime_ms']
            if row['sample_tick_ms'] != self.last_sample:
                self.last_sample=row['sample_tick_ms'];self.last_progress=now
            stale=row['barometer_age_ms'] is None or row['barometer_age_ms']>3000 or now-self.last_progress>3
            row.update(elapsed_s=now-self.start,stale=stale)
            with self.lock:
                self.latest=row
                self.rows.append(row)
                self.status=('STALE barometer: acquisition tick is not advancing' if stale else 'Running · read-only live values')
            if self.writer:self.writer.writerow(row)
        except (RuntimeError,ValueError,OSError) as error:
            with self.lock:
                self.status=str(error)+' · last accepted values are stale'
                if self.latest:self.latest=dict(self.latest,stale=True)
                self.rows.append({'elapsed_s':now-self.start,'stale':True})

    def run(self):
        while not self.stop.is_set():
            began=time.monotonic();self.sample();self.stop.wait(max(0,1/self.rate-(time.monotonic()-began)))

    def payload(self):
        with self.lock:
            return {'status':self.status,'latest':self.latest,'rows':list(self.rows),'fields':FIELDS,
                    'build':f'{self.firmware.path} · ELF SHA256 {self.firmware.sha256}' + ('' if self.verified else ' · WARNING: firmware identity mismatch overridden')}

def find_cli(explicit):
    if explicit:return str(Path(explicit).expanduser().resolve())
    found=shutil.which('STM32_Programmer_CLI')
    if found:return found
    candidates=list(Path('/opt/ST').glob('*/STM32CubeProgrammer/bin/STM32_Programmer_CLI'))
    candidates+=list(Path('/Applications/STMicroelectronics').glob('STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI'))
    if len(candidates)==1:return str(candidates[0])
    raise ValueError('STM32_Programmer_CLI not found; install STM32CubeProgrammer and pass --programmer PATH')

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    target=parser.add_mutually_exclusive_group()
    target.add_argument('--elf',type=Path,help='Exact currently flashed application ELF (symbols retained)')
    target.add_argument('--build',type=Path,help='Current build directory containing FlightComputer26.elf')
    parser.add_argument('--inspect',action='store_true',help='Resolve ELF symbols only; no probe access')
    parser.add_argument('--list-probes',action='store_true',help='List USB ST-Link probes only, then exit')
    parser.add_argument('--programmer',help='STM32_Programmer_CLI executable')
    parser.add_argument('--serial',help='ST-Link serial number; mandatory if several probes are present')
    parser.add_argument('--frequency',type=int,default=1000,help='SWD clock kHz (default 1000)')
    parser.add_argument('--rate',type=float,default=1,help='Requested reads/second; actual rate depends on CLI (0.1–10)')
    parser.add_argument('--history',type=int,default=600,help='Plot history seconds (bounded to 20000 samples)')
    parser.add_argument('--csv',type=Path,help='Write accepted samples to a new CSV file')
    parser.add_argument('--port',type=int,default=0,help='Local browser port; default chooses a free port')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--allow-mismatched-firmware',action='store_true',help='DANGEROUS diagnostic override: selected symbols may not describe target RAM')
    args=parser.parse_args(argv)
    if not .1<=args.rate<=10 or not 1<=args.history<=86400 or not 1<=args.frequency<=24000 or not 0<=args.port<=65535:
        parser.error('Invalid rate/history/frequency/port')
    try:
        if args.list_probes:
            result=subprocess.run([find_cli(args.programmer),'--list','st-link'],capture_output=True,text=True,timeout=15)
            print(result.stdout);return result.returncode
        validate_layout(SOURCE_ROOT)
        elf=args.elf or (args.build or SOURCE_ROOT/'build'/'Release')/'FlightComputer26.elf'
        firmware=Firmware(elf)
        print('ELF:',firmware.path,'SHA256:',firmware.sha256)
        print('Initialized flash verification:',sum(len(x) for _,x in firmware.flash),'bytes across',len(firmware.flash),'regions')
        print('Symbols:',json.dumps({k:{'address':hex(v.address),'size':v.size} for k,v in firmware.resolved.items()},indent=2))
        if args.inspect:return 0
        executable=find_cli(args.programmer)
        serial=args.serial
        if not serial:
            listing=subprocess.run([executable,'--list','st-link'],capture_output=True,text=True,timeout=15)
            serials=re.findall(r'ST-?LINK\s+SN\s*:\s*(\S+)',listing.stdout,re.I)
            if listing.returncode or len(serials)!=1:raise ValueError('Select one probe with --serial; use --list-probes to inspect USB probes')
            serial=serials[0]
        probe=CubeProbe(executable,serial,args.frequency)
        matched=True
        try: verified=firmware.verify(probe);print('Matched all initialized ELF flash sections:',verified,'bytes')
        except ValueError:
            if not args.allow_mismatched_firmware:raise
            matched=False
            print('WARNING: firmware mismatch explicitly overridden; all displayed interpretations are unverified')
        csv_file=None
        try:
            writer=None
            if args.csv:
                csv_file=args.csv.open('x',newline='');writer=csv.DictWriter(csv_file,fieldnames=['elapsed_s','stale','uptime_ms','sample_tick_ms','barometer_age_ms']+[x[0] for x in FIELDS]+['driver_altitude_m']);writer.writeheader()
            scope=Scope(firmware,probe,args.rate,args.history,writer);scope.verified=matched
            class Handler(BaseHTTPRequestHandler):
                def do_GET(self):
                    if self.path=='/':data=PAGE.encode();kind='text/html; charset=utf-8'
                    elif self.path=='/data':data=json.dumps(scope.payload(),allow_nan=False).encode();kind='application/json'
                    else:self.send_error(404);return
                    self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                def log_message(self,*_):pass
            with ThreadingHTTPServer(('127.0.0.1',args.port),Handler) as server:
                worker=threading.Thread(target=scope.run,daemon=True);worker.start()
                url=f'http://127.0.0.1:{server.server_port}/';print('Viewer:',url,'(Ctrl-C to stop; target is never resumed/reset)')
                if not args.no_browser:webbrowser.open(url)
                try:server.serve_forever(poll_interval=.25)
                finally:scope.stop.set();worker.join(timeout=17)
        finally:
            if csv_file:csv_file.close()
    except KeyboardInterrupt:return 0
    except (OSError,ValueError,RuntimeError,subprocess.TimeoutExpired) as error:
        print('Barometer viewer:',error);return 1
    return 0

if __name__=='__main__':raise SystemExit(main())

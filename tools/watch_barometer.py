#!/usr/bin/env python3
"""Read-only FC sensor plots, explicit preflight bench control, and CSV replay."""
from __future__ import annotations
import argparse
from bisect import bisect_left, bisect_right
from collections import deque
import csv
from dataclasses import dataclass
import hashlib
import io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time
from urllib.parse import parse_qs, urlsplit
import webbrowser

FLASH = (0x08000000, 0x08080000)
RAM = (0x20000000, 0x20044000)
SOURCE_ROOT = Path(__file__).resolve().parent.parent
DHCSR = 0xE000EDF0  # Read only: S_HALT bit; never written.
MONITOR_MAGIC = 0x46434d32

def csv_data(rows):
    output=io.StringIO(newline='')
    columns=['elapsed_s','stale','uptime_ms','sample_tick_ms','barometer_age_ms']+[x[0] for x in FIELDS]+['driver_altitude_m']
    writer=csv.DictWriter(output,fieldnames=columns);writer.writeheader();writer.writerows(rows)
    return output.getvalue()

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
        self.optional = {'gnd_lvl_pressure':4,'alt_bias':4,'prev_rel_alt':4,'imedsv':36,'g_fc_monitor':48}
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

def local_firmware_paths(root=SOURCE_ROOT):
    paths=list((root/'build').glob('*/FlightComputer26.elf'))
    paths.sort(key=lambda path:path.stat().st_mtime_ns,reverse=True)
    if not paths:raise ValueError('No local firmware ELF found; build the firmware or pass --elf PATH')
    return paths

def matching_firmware(probe, paths):
    errors=[]
    for path in paths:
        try:
            firmware=Firmware(path)
            return firmware,firmware.verify(probe)
        except ValueError as error:errors.append(f'{path}: {error}')
    raise ValueError('No local ELF matches flashed firmware:\n'+'\n'.join(errors))

def describe_firmware(firmware):
    print('ELF:',firmware.path,'SHA256:',firmware.sha256)
    print('Initialized flash verification:',sum(len(x) for _,x in firmware.flash),'bytes across',len(firmware.flash),'regions')
    print('Symbols:',json.dumps({k:{'address':hex(v.address),'size':v.size} for k,v in firmware.resolved.items()},indent=2))

class InconsistentRead(RuntimeError):
    pass

class OpenOCDProbe:
    def __init__(self, executable, serial, frequency=1000, timeout=15):
        if serial is not None and not re.fullmatch(r'[A-Za-z0-9]+',serial):
            raise ValueError('Invalid ST-Link serial number')
        self.executable, self.serial = executable, serial
        self.frequency, self.timeout = frequency, timeout
        self.process=None;self.connection=None;self.directory=None;self.log=None

    def commands(self):
        # AP1 is the H523 core; AP0 is debug authentication. No board/flash
        # hooks, halt/reset commands, or externally accessible debug servers.
        # serial None: the only attached probe. OpenOCD 0.12 on macOS cannot
        # read STLINK-V3 serial descriptors, so a filter would never match.
        return ['source [find interface/stlink-dap.cfg]',
                        'transport select dapdirect_swd',
                        *([f'adapter serial {self.serial}'] if self.serial else []),
                        f'adapter speed {self.frequency}',
                        'swd newdap fc cpu -expected-id 0x6ba02477',
                        'dap create fc.dap -chain-position fc.cpu',
                        'target create fc.cpu cortex_m -dap fc.dap -ap-num 1',
                        'bindto 127.0.0.1','gdb_port disabled','telnet_port disabled',
                        'tcl_port 0','init','poll off']

    def connect(self):
        if self.connection is not None:return
        try:
            self.directory=tempfile.TemporaryDirectory(prefix='fc-baroscope-openocd-')
            path=Path(self.directory.name)/'openocd.log'
            self.log=path.open('w')
            command=[self.executable]
            for c in self.commands():command+=['-c',c]
            self.process=subprocess.Popen(command,stdin=subprocess.DEVNULL,
                                          stdout=self.log,stderr=self.log)
            deadline=time.monotonic()+self.timeout
            while time.monotonic()<deadline:
                if self.process.poll() is not None:break
                match=re.search(r'Listening on port (\d+) for tcl connections',path.read_text())
                if match:
                    self.connection=socket.create_connection(('127.0.0.1',int(match[1])),timeout=self.timeout)
                    self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    return
                time.sleep(.02)
            details='\n'.join(path.read_text(errors='replace').splitlines()[-16:])
            raise RuntimeError('ST-Link connection failed. OpenOCD output:\n'+(details or 'No output before startup timeout'))
        except BaseException:
            self.close();raise

    def close(self):
        if self.connection is not None:
            self.connection.close();self.connection=None
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
            self.process=None
        if self.log is not None:self.log.close();self.log=None
        if self.directory is not None:self.directory.cleanup();self.directory=None

    def request(self, command):
        self.connect()
        try:
            # Catch Tcl failures explicitly; the RPC protocol itself carries no
            # error status. One framed response serves the entire sample batch.
            script='if {[catch {'+command+'} fc_result]} {format "ERROR:%s" $fc_result} else {format "OK:%s" $fc_result}'
            self.connection.sendall(script.encode()+b'\x1a')
            response=bytearray()
            while not response.endswith(b'\x1a'):
                chunk=self.connection.recv(65536)
                if not chunk:raise RuntimeError('ST-Link session disconnected')
                response.extend(chunk)
                if len(response)>32*1024*1024:raise RuntimeError('Oversized ST-Link response')
            value=response[:-1].decode().strip()
            if value.startswith('ERROR:'):raise RuntimeError('OpenOCD read failed: '+value[6:200])
            if not value.startswith('OK:'):raise RuntimeError('ST-Link memory read failed; sample discarded')
            return value[3:]
        except RuntimeError:
            self.close();raise
        except (OSError,UnicodeError):
            self.close();raise RuntimeError('ST-Link read failed or timed out; sample discarded. No reset/takeover attempted.')

    def read_blocks(self, blocks):
        if not blocks:return []
        if any(size<=0 or size>2*1024*1024 for _,size in blocks):
            raise ValueError('Invalid bounded memory read size')
        if sum(size for _,size in blocks)>4096:
            # Startup flash verification stays on OpenOCD's binary dump path:
            # older RPC servers cannot reliably send a huge Tcl response. Live
            # RAM snapshots use the small in-memory RPC path below, no files.
            with tempfile.TemporaryDirectory(prefix='fc-baroscope-verify-') as tmp:
                commands=[];paths=[]
                for i,(address,size) in enumerate(blocks):
                    path=Path(tmp)/f'{i}.bin';paths.append(path)
                    quoted='"'+re.sub(r'([\\"$\[\]])',r'\\\1',str(path))+'"'
                    commands.append(f'dump_image {quoted} {hex(address)} {size}')
                self.request('; '.join(commands)+'; set fc_dumped 1')
                values=[path.read_bytes() for path in paths]
                if any(len(data)!=size for data,(_,size) in zip(values,blocks)):
                    raise RuntimeError('Short ST-Link flash read')
                return values
        commands=[];widths=[]
        for address,size in blocks:
            width=32 if address%4==0 and size%4==0 else 8
            widths.append(width)
            commands.append(f'lappend fc_blocks [read_memory {hex(address)} {width} {size//(width//8)}]')
        result=self.request('set fc_blocks {}; '+'; '.join(commands)+'; join $fc_blocks "|"')
        try:
            parts=result.split('|')
            if len(parts)!=len(blocks):raise ValueError('block count')
            values=[]
            for part,width,(_,size) in zip(parts,widths,blocks):
                words=[int(v,0) for v in part.split()]
                if len(words)!=size//(width//8):raise ValueError('short read')
                values.append(struct.pack('<'+'I'*len(words),*words) if width==32 else bytes(words))
            return values
        except (ValueError,struct.error):
            self.close();raise RuntimeError('Short/invalid ST-Link read; sample discarded')

    def monitor_command(self, firmware, sequence, enabled, reset=False, mode=True):
        symbol=firmware.resolved.get('g_fc_monitor')
        if symbol is None:raise ValueError('FC monitor mode requires updated firmware')
        if struct.unpack('<I',self.read_blocks([(symbol.address,4)])[0])[0]!=MONITOR_MAGIC:
            raise ValueError('Unsupported FC monitor control ABI')
        word=(sequence & 0x1fffffff) | (0x80000000 if mode else 0) | (0x20000000 if enabled else 0) | (0x40000000 if reset else 0)
        # The only supported RAM write is this versioned, firmware-owned
        # bench mailbox. Never edit g_conf, task state, outputs, or registers.
        self.request(f'mww {hex(symbol.address+4)} {hex(word)}')

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
        # Probe uploads can take longer than a sensor period. Requiring
        # equal values across both passes rejects a healthy, updating sensor
        # indefinitely. Only compare payloads when they claim the SAME sample;
        # otherwise retain the newer pass. This remains best-effort, not atomic.
        stable=[n for n in names if n not in ('uwTick','meas','imedsv','g_fc_monitor')]
        if a['last_tick'] == b['last_tick'] and (
                a['meas'][12:24] != b['meas'][12:24] or any(a[n]!=b[n] for n in stable)):
            raise InconsistentRead('Inconsistent live barometer read; sample discarded (CPU remains running)')
        if 'imedsv' in b and a['imedsv']!=b['imedsv']:
            del b['imedsv']  # Do not present a visibly changing multi-float prediction as coherent.
        if ('g_fc_monitor' in b and a['g_fc_monitor'][12:16]==b['g_fc_monitor'][12:16]
                and a['g_fc_monitor']!=b['g_fc_monitor']):
            del b['g_fc_monitor']
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
    for i,axis in enumerate('xyz'):
        data[f'accel_{axis}']=f('meas',24+4*i)
        data[f'gyro_{axis}_dps']=f('meas',36+4*i)
    for name in ('fc_kalman_altitude_m','fc_kalman_velocity_m_s','fc_quaternion_w','fc_quaternion_x','fc_quaternion_y','fc_quaternion_z'):
        data[name]=None
    if 'g_fc_monitor' in values:
        magic,status=struct.unpack_from('<I',values['g_fc_monitor'])[0],struct.unpack_from('<I',values['g_fc_monitor'],8)[0]
        if magic==MONITOR_MAGIC and status==2:
            for offset,name in enumerate(('fc_kalman_altitude_m','fc_kalman_velocity_m_s','fc_quaternion_w','fc_quaternion_x','fc_quaternion_y','fc_quaternion_z')):
                data[name]=f('g_fc_monitor',24+offset*4)
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
FIELDS += [(f'accel_{axis}',f'Accelerometer {axis.upper()}','driver units') for axis in 'xyz']
FIELDS += [(f'gyro_{axis}_dps',f'Gyroscope {axis.upper()}','°/s') for axis in 'xyz']
FIELDS += [('fc_kalman_altitude_m','FC bench Kalman altitude','m'),('fc_kalman_velocity_m_s','FC bench Kalman vertical velocity','m/s')]
FIELDS += [(f'fc_quaternion_{axis}',f'FC bench quaternion {axis}','') for axis in 'wxyz']

PAGE = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>FC Sensors</title>
<style>
#replay-controls[hidden]{display:none}#replay-controls{max-width:1400px;margin:16px auto}
.controls a{color:#8be7d7;text-underline-offset:3px}
*{box-sizing:border-box}body{font:15px system-ui;background:#101722;color:#e2eaf5;margin:24px}header{max-width:1400px;margin:auto}h1{font-size:26px;margin:0 0 16px}#status{padding:12px;border:1px solid #688399;border-radius:8px}#build{font:11px monospace;color:#94a7bb;overflow-wrap:anywhere}#history{color:#a9b8cc}#panels{max-width:1400px;margin:auto;display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,350px),1fr));gap:14px}article{background:#172435;border:1px solid #42566b;border-radius:10px;padding:14px}.card-title,.dialog-title{display:flex;align-items:center;justify-content:space-between;gap:12px}h2{font-size:15px;margin:0}strong{display:block;font:26px monospace;margin:10px 0}canvas{display:block;width:100%;height:180px}article canvas{cursor:zoom-in}button,select{font:inherit;color:inherit;background:#203349;border:1px solid #59728d;border-radius:6px;padding:7px 10px;cursor:pointer}button:hover,select:hover{background:#2c4763}button:focus-visible,select:focus-visible,input:focus-visible{outline:2px solid #63d6c1;outline-offset:3px}.expand{font-size:12px}dialog{color:inherit;background:#172435;border:1px solid #59728d;border-radius:12px;width:min(1450px,96vw);max-width:96vw;max-height:96vh;padding:20px}dialog::backdrop{background:#060c16d9}dialog h2{font-size:20px}.controls{display:flex;flex-wrap:wrap;align-items:center;gap:12px;margin:20px 0 12px}label{display:flex;align-items:center;gap:8px}.timeline{flex:1;min-width:220px}.timeline input{width:100%;accent-color:#63d6c1}#detail-canvas{height:clamp(260px,52vh,620px);touch-action:none;cursor:crosshair}#inspect{min-height:24px;font:14px monospace;color:#8be7d7}#stats{display:flex;flex-wrap:wrap;gap:12px 28px;border-top:1px solid #42566b;padding-top:14px;font:13px monospace}small{display:block;color:#a9b8cc;margin:10px 0}#view-range{font:13px monospace;color:#a9b8cc}@media(max-width:600px){body{margin:12px}dialog{padding:12px}.controls{gap:8px}.timeline{flex-basis:100%}#detail-canvas{height:42vh}}
</style>
<header><h1>FC Sensors</h1><p id="status">Waiting for ST-Link read…</p><div class="controls"><a href="/export.csv" download="fc-sensors.csv">Save retained CSV</a><button id="kf-start" disabled>Start FC Kalman</button><button id="kf-stop" disabled>Stop FC Kalman</button><button id="kf-reset" disabled>Reset FC Kalman</button><span id="kf-status"></span></div><p id="history"></p><p id="build"></p></header><main id="panels"></main>
<div id="replay-controls" class="controls" hidden><button id="replay-play">Play replay</button><button id="replay-reset">Restart replay</button><label>Speed <select id="replay-speed"><option value="0.25">0.25×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option><option value="10">10×</option></select></label><label class="timeline">Seek <input id="replay-seek" type="range" min="0" max="1" step="0.01" value="0"></label><span id="replay-time"></span></div>
<dialog id="detail" aria-labelledby="detail-title">
 <div class="dialog-title"><h2 id="detail-title"></h2><button id="close-detail">Close</button></div>
 <div class="controls">
  <label>Window <select id="span"><option value="30">30 seconds</option><option value="120">2 minutes</option><option value="600" selected>10 minutes</option><option value="1800">30 minutes</option><option value="3600">1 hour</option><option value="all">All retained data</option><option value="custom" hidden>Custom</option></select></label>
  <button id="pause">Pause view</button><button id="live">Go live</button>
  <label class="timeline">Pan <input id="pan" type="range" min="0" max="1" step="0.01" value="1" aria-label="End of visible time range"></label>
 </div>
 <div id="view-range"></div><canvas id="detail-canvas" aria-label="Expanded sensor history"></canvas>
 <div id="inspect">Hover to inspect a sample. Scroll to zoom; drag to pan.</div>
 <div id="stats"></div><small>Viewing can be paused while acquisition continues. Every accepted value is plotted directly; gaps mark stale or missing data.</small>
</dialog>
<script>
let fields=[],busy=false,rows=[],cursor=null,latest=null,frame=null,replayPlaying=false;
const detail={key:null,span:600,end:null,follow:true,drag:null,plot:null};
const $=id=>document.getElementById(id);
function clock(t){t=Math.max(0,t);const h=Math.floor(t/3600),m=Math.floor(t%3600/60),s=(t%60).toFixed(1).padStart(4,'0');return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${s}`}
function number(v){return Number.isFinite(v)?Number(v.toPrecision(9)).toString():'Unavailable'}
function lowerBound(data,time){let lo=0,hi=data.length;while(lo<hi){const mid=(lo+hi)>>>1;if(data[mid].elapsed_s<time)lo=mid+1;else hi=mid}return lo}
function bounds(span,end){const last=rows.at(-1)?.elapsed_s||0,first=rows[0]?.elapsed_s||0;end=Math.max(first,Math.min(last,end??last));return {start:span==='all'?first:Math.max(first,end-span),end}}
function seriesStats(data,key,start,end){
 let count=0,first=null,last=null,min=Infinity,max=-Infinity,meanT=0,meanV=0,cov=0,varT=0;
 for(let i=lowerBound(data,start);i<data.length&&data[i].elapsed_s<=end;i++){
  const r=data[i],v=r[key];if(r.stale||!Number.isFinite(v))continue;
  const t=r.elapsed_s-start;if(first===null)first=v;last=v;count++;min=Math.min(min,v);max=Math.max(max,v);
  const dt=t-meanT;meanT+=dt/count;const dv=v-meanV;meanV+=dv/count;cov+=dt*(v-meanV);varT+=dt*(t-meanT);
 }
 return {count,first,last,min,max,delta:count?last-first:null,perHour:varT>0?cov/varT*3600:null};
}
function graph(canvas,key,start,end,inspectTime=null){
 const dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
 if(w<1||h<1)return null;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
 const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);
 const stats=seriesStats(rows,key,start,end),left=88,right=w-16,top=18,bottom=h-32;
 c.font='11px monospace';c.fillStyle='#a9b8cc';
 if(!stats.count){c.fillText('No valid samples in this range',12,h/2);return null}
 let lo=stats.min,hi=stats.max;if(lo===hi){lo-=.5;hi+=.5}else{const pad=(hi-lo)*.05;lo-=pad;hi+=pad}
 const t1=Math.max(start+.001,end),x=t=>left+(right-left)*(t-start)/(t1-start),y=v=>bottom-(bottom-top)*(v-lo)/(hi-lo);
 c.strokeStyle='#30465d';c.lineWidth=1;c.beginPath();
 for(let i=0;i<=4;i++){const py=top+(bottom-top)*i/4;c.moveTo(left,py);c.lineTo(right,py);c.fillText(number(hi-(hi-lo)*i/4),1,py+4)}
 c.stroke();c.fillText(clock(start),left,h-8);c.textAlign='right';c.fillText(clock(end),right,h-8);c.textAlign='left';
 c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();c.strokeStyle='#63d6c1';c.lineWidth=1.2;c.beginPath();let open=false;
 // No averaging, smoothing, fitted curve, or point reduction: use every
 // retained sample in the visible range and break the trace at data gaps.
 for(let i=lowerBound(rows,start);i<rows.length&&rows[i].elapsed_s<=end;i++){
  const r=rows[i],v=r[key];if(r.stale||!Number.isFinite(v)){open=false;continue}
  open?c.lineTo(x(r.elapsed_s),y(v)):c.moveTo(x(r.elapsed_s),y(v));open=true;
 }
 c.stroke();
 if(stats.count<=200){c.fillStyle='#63d6c1';for(let i=lowerBound(rows,start);i<rows.length&&rows[i].elapsed_s<=end;i++){const r=rows[i];if(r.stale||!Number.isFinite(r[key]))continue;c.beginPath();c.arc(x(r.elapsed_s),y(r[key]),1.8,0,Math.PI*2);c.fill()}}
 if(inspectTime!==null){c.strokeStyle='#d8e6f4';c.setLineDash([4,4]);c.beginPath();c.moveTo(x(inspectTime),top);c.lineTo(x(inspectTime),bottom);c.stroke()}
 c.restore();return {start,end,left,right,stats};
}
function renderDetail(){
 if(!$('detail').open||!detail.key||!rows.length)return;
 const b=bounds(detail.span,detail.follow?null:detail.end);detail.end=b.end;
 detail.plot=graph($('detail-canvas'),detail.key,b.start,b.end);
 $('pan').min=rows[0].elapsed_s;$('pan').max=rows.at(-1).elapsed_s;$('pan').value=b.end;
 $('pause').textContent=detail.follow?'Pause view':'Resume live';
 $('view-range').textContent=`${detail.follow?'Live':'Paused view'} · ${clock(b.start)} – ${clock(b.end)} elapsed since monitor start`;
 const s=detail.plot?.stats,unit=fields.find(f=>f[0]===detail.key)?.[2]||'';
 $('stats').textContent=s?`${s.count.toLocaleString()} samples    Δ ${number(s.delta)} ${unit}    Min ${number(s.min)}    Max ${number(s.max)}    Linear drift estimate ${number(s.perHour)} ${unit}/hour`:'No valid samples in this range';
}
function render(){
 frame=null;const b=bounds(60,null);
 for(const a of document.querySelectorAll('article')){const v=latest?.[a.dataset.key];a.querySelector('strong').textContent=number(v);graph(a.querySelector('canvas'),a.dataset.key,b.start,b.end)}
 renderDetail();
}
function scheduleRender(){if(frame===null)frame=requestAnimationFrame(render)}
async function kalman(action){try{const r=await fetch('/kalman',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});const d=await r.json();if(!r.ok)throw Error(d.error);$('kf-status').textContent=d.status}catch(e){$('kf-status').textContent=e.message}}
$('kf-start').onclick=()=>kalman('start');$('kf-stop').onclick=()=>kalman('stop');$('kf-reset').onclick=()=>kalman('reset');
async function playback(action,value){try{const r=await fetch('/playback',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,value})});if(!r.ok)throw Error('Replay request failed')}catch(e){$('status').textContent=e.message}}
$('replay-play').onclick=()=>playback(replayPlaying?'pause':'play');$('replay-reset').onclick=()=>playback('restart');$('replay-speed').onchange=e=>playback('speed',Number(e.target.value));$('replay-seek').oninput=e=>playback('seek',Number(e.target.value));
function openDetail(key){detail.key=key;detail.follow=true;detail.end=null;detail.drag=null;const f=fields.find(f=>f[0]===key);$('detail-title').textContent=`${f[1]} (${f[2]})`;$('detail').showModal();$('inspect').textContent='Hover to inspect a sample. Scroll to zoom; drag to pan.';renderDetail()}
function setFollowing(follow){detail.follow=follow;if(!follow)detail.end=rows.at(-1)?.elapsed_s||0;renderDetail()}
$('close-detail').onclick=()=>$('detail').close();
$('pause').onclick=()=>setFollowing(!detail.follow);$('live').onclick=()=>setFollowing(true);
$('span').onchange=e=>{if(e.target.value!=='custom')detail.span=e.target.value==='all'?'all':Number(e.target.value);renderDetail()};
$('pan').oninput=e=>{detail.follow=false;detail.end=Number(e.target.value);renderDetail()};
function timeAt(event){const p=detail.plot;if(!p)return null;const rect=$('detail-canvas').getBoundingClientRect(),fraction=Math.max(0,Math.min(1,(event.clientX-rect.left-p.left)/(p.right-p.left)));return p.start+(p.end-p.start)*fraction}
$('detail-canvas').addEventListener('wheel',e=>{
 if(!detail.plot||rows.length<2)return;e.preventDefault();const p=detail.plot,anchor=timeAt(e),fraction=(anchor-p.start)/Math.max(.001,p.end-p.start),span=Math.max(1,Math.min(rows.at(-1).elapsed_s-rows[0].elapsed_s,(p.end-p.start)*Math.exp(e.deltaY*.002)));
 detail.follow=false;detail.span=span;detail.end=anchor+span*(1-fraction);$('span').value='custom';renderDetail();
},{passive:false});
$('detail-canvas').onpointerdown=e=>{if(!detail.plot)return;detail.follow=false;detail.drag={x:e.clientX,end:detail.plot.end,span:detail.plot.end-detail.plot.start};$('detail-canvas').setPointerCapture(e.pointerId)};
$('detail-canvas').onpointerup=e=>{detail.drag=null;if($('detail-canvas').hasPointerCapture(e.pointerId))$('detail-canvas').releasePointerCapture(e.pointerId)};
$('detail-canvas').onpointercancel=()=>{detail.drag=null};
$('detail-canvas').onpointermove=e=>{
 if(!detail.plot||!rows.length)return;
 if(detail.drag){detail.end=detail.drag.end-(e.clientX-detail.drag.x)*detail.drag.span/(detail.plot.right-detail.plot.left);renderDetail();return}
 const t=timeAt(e),i=lowerBound(rows,t),candidates=[rows[i-1],rows[i]].filter(r=>r&&r.elapsed_s>=detail.plot.start&&r.elapsed_s<=detail.plot.end);
 const r=candidates.sort((a,b)=>Math.abs(a.elapsed_s-t)-Math.abs(b.elapsed_s-t))[0];if(!r)return;
 const f=fields.find(f=>f[0]===detail.key);$('inspect').textContent=`${clock(r.elapsed_s)} · ${r.stale?'Stale / missing sample':number(r[detail.key])+' '+f[2]}`;
 graph($('detail-canvas'),detail.key,detail.plot.start,detail.plot.end,r.elapsed_s);
};
async function refresh(){
 if(busy)return;busy=true;
 try{
  const response=await fetch('/data'+(cursor===null?'':'?after='+cursor),{cache:'no-store'});if(!response.ok)throw Error('HTTP');const d=await response.json();
  if(d.reset)rows=[];for(const r of d.rows)rows.push(r);cursor=d.cursor;latest=d.latest;
  if(rows.length>d.capacity)rows=rows.slice(-d.capacity);
  if(rows.length)rows=rows.slice(lowerBound(rows,rows.at(-1).elapsed_s-d.history_s));
  $('status').textContent=d.status;$('build').textContent=d.build;
  for(const id of ['kf-start','kf-stop','kf-reset'])$(id).disabled=!d.kalman.control_available;
  $('kf-status').textContent=d.kalman.status;
  $('replay-controls').hidden=!d.replay;
  if(d.replay){replayPlaying=d.replay.playing;$('replay-play').textContent=replayPlaying?'Pause replay':'Play replay';$('replay-seek').min=d.replay.start;$('replay-seek').max=d.replay.end;$('replay-seek').value=d.replay.position;$('replay-time').textContent=clock(d.replay.position)+' / '+clock(d.replay.end)}
  $('history').textContent=rows.length?`${rows.length.toLocaleString()} retained samples · ${clock(rows[0].elapsed_s)} – ${clock(rows.at(-1).elapsed_s)} · Expand a graph to inspect history`:'Waiting for history';
  if(!fields.length){fields=d.fields;for(const [key,label,unit] of fields){const a=document.createElement('article');a.innerHTML='<div class="card-title"><h2></h2><button class="expand">Expand ↗</button></div><strong>—</strong><canvas></canvas>';a.querySelector('h2').textContent=`${label} (${unit})`;a.dataset.key=key;a.querySelector('button').setAttribute('aria-label',`Expand ${label} graph`);a.querySelector('button').onclick=()=>openDetail(key);a.querySelector('canvas').onclick=()=>openDetail(key);$('panels').append(a)}}
  scheduleRender();
 }catch(error){$('status').textContent='Viewer disconnected; displayed values are stale'}finally{busy=false}
}
window.addEventListener('resize',scheduleRender);setInterval(refresh,100);refresh();
</script></html>'''

class Scope:
    def __init__(self, firmware, probe, rate, history, writer=None, kalman_control=False):
        self.firmware,self.probe,self.rate,self.writer = firmware,probe,rate,writer
        self.rows=deque(maxlen=min(100000,max(2,math.ceil(history*rate))))
        self.lock=threading.Lock();self.stop=threading.Event();self.start=time.monotonic()
        self.status='Waiting for first read';self.latest=None;self.verified=True
        self.last_sample=None;self.last_progress=self.start;self.last_uptime=None
        self.cursor=0;self.history=history;self.accepted_at=None;self.read_duration_ms=None
        self.kalman_control=kalman_control;self.kalman_enabled=False;self.kalman_reset=False
        self.kalman_sequence=0;self.kalman_sent=0;self.kalman_dirty=kalman_control
        self.kalman_status='FC bench control unavailable in this firmware'

    def kalman_action(self, action):
        if not self.kalman_control:raise ValueError('FC estimator control requires matching updated firmware and monitor mode')
        if action not in ('start','stop','reset'):raise ValueError('Unknown Kalman action')
        with self.lock:
            if action!='reset':self.kalman_enabled=action=='start'
            self.kalman_reset=action=='reset'
            self.kalman_dirty=True
            self.kalman_status='FC estimator request pending'

    def renew_kalman(self):
        with self.lock:
            due=self.kalman_dirty or time.monotonic()-self.kalman_sent>=.5
            if not self.kalman_control or not due:return
            self.kalman_sequence=(self.kalman_sequence+1)&0x1fffffff or 1
            self.probe.monitor_command(self.firmware,self.kalman_sequence,self.kalman_enabled,self.kalman_reset)
            self.kalman_sent=time.monotonic();self.kalman_dirty=False;self.kalman_reset=False

    def sample(self):
        now=time.monotonic()
        try:
            self.renew_kalman()
            for attempt in range(3):
                try:
                    snapshot=self.probe.snapshot(self.firmware)
                    row=decode(snapshot)
                    break
                except InconsistentRead:
                    if attempt == 2:raise
            completed=time.monotonic()
            if self.last_uptime is not None and ((row['uptime_ms']-self.last_uptime)&0xffffffff)>0x80000000:
                self.status='MCU uptime restarted; stopping. Re-run to verify flash again.'
                with self.lock:
                    if self.latest:self.latest=dict(self.latest,stale=True)
                self.stop.set();return
            self.last_uptime=row['uptime_ms']
            if row['sample_tick_ms'] != self.last_sample:
                self.last_sample=row['sample_tick_ms'];self.last_progress=completed
            stale=row['barometer_age_ms'] is None or row['barometer_age_ms']>3000 or completed-self.last_progress>3
            row.update(elapsed_s=completed-self.start,stale=stale)
            with self.lock:
                self.latest=row
                self.rows.append(row)
                self.cursor+=1;self.accepted_at=completed;self.read_duration_ms=(completed-now)*1000
                self.status=('STALE barometer: acquisition tick is not advancing' if stale else 'Running · live sensor values')
                if 'g_fc_monitor' in snapshot and struct.unpack_from('<I',snapshot['g_fc_monitor'])[0]==MONITOR_MAGIC:
                    state=struct.unpack_from('<I',snapshot['g_fc_monitor'],8)[0]
                    self.kalman_status={0:('Monitor mode · FC estimator stopped' if self.kalman_control else 'FC estimator stopped'),1:'FC bench estimator waiting for fresh sensors',2:'FC bench estimator running',3:'Entering monitor mode · waiting for flight evaluator to stop',4:'Bench entry refused by flight/recovery owner'}.get(state,'Unknown FC bench status')
            if self.writer:self.writer.writerow(row)
        except (RuntimeError,ValueError,OSError) as error:
            with self.lock:
                self.status=str(error)+' · last accepted values are stale'
                if self.latest:self.latest=dict(self.latest,stale=True)
                self.rows.append({'elapsed_s':now-self.start,'stale':True})
                self.cursor+=1

    def run(self):
        next_sample=0.0
        while not self.stop.is_set():
            now=time.monotonic()
            if now>=next_sample:
                self.sample();next_sample=now+1/self.rate
            else:
                try:self.renew_kalman()
                except (OSError,RuntimeError,ValueError) as error:
                    with self.lock:self.status=str(error)
            # Renew the session even with slow sampling or a stopped estimator.
            self.stop.wait(min(.25,max(0,next_sample-time.monotonic())))

    def payload(self, after=None):
        with self.lock:
            oldest=self.cursor-len(self.rows)
            reset=after is None or after<oldest or after>self.cursor
            rows=list(self.rows)
            if not reset:rows=rows[after-oldest:]
            return {'status':self.status,'latest':self.latest,'rows':rows,'fields':FIELDS,
                    'cursor':self.cursor,'reset':reset,'capacity':self.rows.maxlen,'history_s':self.history,
                    'read_duration_ms':self.read_duration_ms,
                    'host_sample_age_ms':None if self.accepted_at is None else (time.monotonic()-self.accepted_at)*1000,
                    'kalman':{'control_available':self.kalman_control,'status':self.kalman_status},
                    'build':f'{self.firmware.path} · ELF SHA256 {self.firmware.sha256}' + ('' if self.verified else ' · WARNING: firmware identity mismatch overridden')}

    def export_csv(self):
        with self.lock:rows=list(self.rows)
        return csv_data(rows)

class ReplayScope(Scope):
    """Replay recorded values as-is, with no probe, writes, or refiltering."""
    def __init__(self, path, history):
        path=Path(path).resolve()
        super().__init__(type('Recording',(),{'path':path,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})(),None,20,history)
        self.recorded=[]
        allowed={'elapsed_s','uptime_ms','sample_tick_ms','barometer_age_ms','driver_altitude_m'} | {f[0] for f in FIELDS}
        with path.open(newline='') as source:
            reader=csv.DictReader(source)
            if not reader.fieldnames or 'elapsed_s' not in reader.fieldnames:raise ValueError('Recording must contain elapsed_s')
            for line in reader:
                row={}
                for name,value in line.items():
                    if name=='stale':row[name]=str(value or '').lower() in ('1','true','yes')
                    elif name in allowed:
                        row[name]=float(value) if value else None
                        if row[name] is not None and not math.isfinite(row[name]):row[name]=None
                if row.get('elapsed_s') is None:raise ValueError('Recording contains invalid elapsed time')
                if self.recorded and row['elapsed_s']<self.recorded[-1]['elapsed_s']:raise ValueError('Recording time must be nondecreasing')
                row.setdefault('stale',False);self.recorded.append(row)
        if not self.recorded:raise ValueError('Recording is empty')
        self.times=[r['elapsed_s'] for r in self.recorded]
        self.position=self.times[0];self.playing=False;self.speed=1.;self.anchor=time.monotonic();self.generation=0
        self.kalman_status='Recorded FC estimates; hardware control disabled during replay'

    def advance(self):
        now=time.monotonic()
        if self.playing:
            self.position=min(self.times[-1],self.position+(now-self.anchor)*self.speed)
            if self.position>=self.times[-1]:self.playing=False
        self.anchor=now

    def playback_action(self, action, value=None):
        with self.lock:
            self.advance()
            if action=='play':self.playing=True
            elif action=='pause':self.playing=False
            elif action=='restart':self.position=self.times[0];self.playing=False;self.generation+=1
            elif action in ('seek','speed'):
                value=float(value)
                if not math.isfinite(value):raise ValueError('Invalid replay value')
                if action=='seek':self.position=max(self.times[0],min(self.times[-1],value));self.generation+=1
                elif .1<=value<=20:self.speed=value
                else:raise ValueError('Replay speed must be 0.1–20')
            else:raise ValueError('Unknown replay action')

    def run(self):self.stop.wait()

    def payload(self, after=None):
        with self.lock:
            self.advance()
            end=bisect_right(self.times,self.position)
            start=max(0,end-100000,bisect_left(self.times,self.position-self.history))
            base=self.generation*(len(self.recorded)+1)
            reset=after is None or not base+start<=after<=base+end
            rows=self.recorded[start if reset else after-base:end]
            return {'status':'Replay playing' if self.playing else 'Replay paused',
                    'latest':self.recorded[end-1] if end else None,'rows':rows,'fields':FIELDS,
                    'cursor':base+end,'reset':reset,'capacity':100000,'history_s':self.history,
                    'read_duration_ms':None,'host_sample_age_ms':None,
                    'kalman':{'control_available':False,'status':self.kalman_status},
                    'replay':{'playing':self.playing,'position':self.position,'start':self.times[0],'end':self.times[-1],'speed':self.speed},
                    'build':f'Recording: {self.firmware.path} · SHA256 {self.firmware.sha256}'}

    def export_csv(self):
        # Replay export preserves the complete loaded recording, including
        # samples after the current playhead, rather than silently truncating it.
        return csv_data(self.recorded)

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
    target.add_argument('--replay',type=Path,help='Replay a saved monitor CSV without connecting to hardware')
    parser.add_argument('--inspect',action='store_true',help='Resolve ELF symbols only; no probe access')
    parser.add_argument('--list-probes',action='store_true',help='List USB ST-Link probes only, then exit')
    parser.add_argument('--programmer',help='STM32_Programmer_CLI executable')
    parser.add_argument('--openocd',help='OpenOCD executable (default: openocd on PATH)')
    parser.add_argument('--serial',help='ST-Link serial number; mandatory if several probes are present')
    parser.add_argument('--frequency',type=int,default=1000,help='SWD clock kHz (default 1000)')
    parser.add_argument('--rate',type=float,default=20,help='Requested reads/second (default 20; range 0.1–50)')
    parser.add_argument('--history',type=int,default=3600,help='Retained history seconds (default 3600; bounded to 100000 samples)')
    parser.add_argument('--csv',type=Path,help='Write accepted samples to a new CSV file')
    parser.add_argument('--bench','--kalman-control',dest='kalman_control',action='store_true',help='Explicitly request preflight bench mode; holds deployment outputs and resets FC on exit')
    parser.add_argument('--read-only',action='store_true',help='Read-only observation (the default); incompatible with --bench')
    parser.add_argument('--enable-kalman',action='store_true',help='With --bench, start the FC estimator after preflight monitor admission')
    parser.add_argument('--port',type=int,default=0,help='Local browser port; default chooses a free port')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--allow-mismatched-firmware',action='store_true',help='DANGEROUS diagnostic override: selected symbols may not describe target RAM')
    args=parser.parse_args(argv)
    if not .1<=args.rate<=50 or not 1<=args.history<=86400 or not 1<=args.frequency<=24000 or not 0<=args.port<=65535:
        parser.error('Invalid rate/history/frequency/port')
    if args.enable_kalman and not args.kalman_control:
        parser.error('--enable-kalman requires explicit --bench')
    if args.read_only and args.kalman_control:
        parser.error('--read-only cannot request --bench')
    if args.kalman_control and (args.inspect or args.list_probes):
        parser.error('--bench requires a live preflight session')
    probe=None
    try:
        if args.list_probes:
            result=subprocess.run([find_cli(args.programmer),'--list','st-link'],capture_output=True,text=True,timeout=15)
            print(result.stdout);return result.returncode
        control=args.kalman_control
        if args.replay:
            if control or args.csv or args.inspect:raise ValueError('--replay cannot be combined with hardware control, --csv, or --inspect')
            scope=ReplayScope(args.replay,args.history)
        else:
            validate_layout(SOURCE_ROOT)
            paths=[args.elf] if args.elf else [args.build/'FlightComputer26.elf'] if args.build else local_firmware_paths()
            if args.inspect:
                describe_firmware(Firmware(paths[0]));return 0
            executable=args.openocd or shutil.which('openocd')
            if not executable:raise ValueError('OpenOCD not found; install OpenOCD and pass --openocd PATH')
            serial=args.serial
            if not serial:
                listing=subprocess.run([find_cli(args.programmer),'--list','st-link'],capture_output=True,text=True,timeout=15)
                serials=re.findall(r'ST-?LINK\s+SN\s*:\s*(\S+)',listing.stdout,re.I)
                if listing.returncode or len(serials)!=1:raise ValueError('Select one probe with --serial; use --list-probes to inspect USB probes')
                serial=None
            probe=OpenOCDProbe(executable,serial,args.frequency)
            matched=True
            try:
                firmware,verified=matching_firmware(probe,paths)
                describe_firmware(firmware)
                print('Matched all initialized ELF flash sections:',verified,'bytes')
            except ValueError:
                if not args.allow_mismatched_firmware:raise
                matched=False;firmware=Firmware(paths[0]);describe_firmware(firmware)
                print('WARNING: firmware mismatch explicitly overridden; all displayed interpretations are unverified')
            if args.read_only and control:raise ValueError('--read-only cannot enable Kalman control')
            control=control and matched and not args.read_only and 'g_fc_monitor' in firmware.resolved
            if control:
                address=firmware.resolved['g_fc_monitor'].address
                control=struct.unpack('<I',probe.read_blocks([(address,4)])[0])[0]==MONITOR_MAGIC
            if (args.enable_kalman or args.kalman_control) and not control:
                raise ValueError('FC estimator control requires matching updated firmware')
        csv_file=None
        try:
            writer=None
            if args.csv:
                csv_file=args.csv.open('x',newline='',buffering=1);writer=csv.DictWriter(csv_file,fieldnames=['elapsed_s','stale','uptime_ms','sample_tick_ms','barometer_age_ms']+[x[0] for x in FIELDS]+['driver_altitude_m']);writer.writeheader()
            if not args.replay:
                scope=Scope(firmware,probe,args.rate,args.history,writer,control);scope.verified=matched
                if args.enable_kalman:scope.kalman_action('start')
            class Handler(BaseHTTPRequestHandler):
                def do_POST(self):
                    if self.path not in ('/kalman','/playback'):self.send_error(404);return
                    if self.headers.get('Origin') not in (None,f'http://127.0.0.1:{self.server.server_port}'):
                        self.send_error(403);return
                    try:
                        length=int(self.headers.get('Content-Length','0'))
                        if not 0<length<=128:raise ValueError('Invalid request size')
                        request=json.loads(self.rfile.read(length))
                        if self.path=='/playback':
                            if not isinstance(scope,ReplayScope):raise ValueError('No recording loaded')
                            scope.playback_action(request['action'],request.get('value'))
                        else:scope.kalman_action(request['action'])
                        data=json.dumps({'status':'Request accepted'}).encode();status=200
                    except (ValueError,KeyError,TypeError) as error:
                        data=json.dumps({'error':str(error)}).encode();status=400
                    self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                def do_GET(self):
                    url=urlsplit(self.path)
                    if url.path=='/':data=PAGE.encode();kind='text/html; charset=utf-8'
                    elif url.path=='/export.csv':data=scope.export_csv().encode();kind='text/csv; charset=utf-8'
                    elif url.path=='/data':
                        try:after=int(parse_qs(url.query)['after'][0]) if 'after' in parse_qs(url.query) else None
                        except ValueError:self.send_error(400);return
                        data=json.dumps(scope.payload(after),allow_nan=False).encode();kind='application/json'
                    else:self.send_error(404);return
                    self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                def log_message(self,*_):pass
            with ThreadingHTTPServer(('127.0.0.1',args.port),Handler) as server:
                worker=threading.Thread(target=scope.run,daemon=True);worker.start()
                url=f'http://127.0.0.1:{server.server_port}/';print('Viewer:',url,'(Ctrl-C to stop; an admitted preflight bench session resets on exit)' if control else '(Ctrl-C to stop)')
                if not args.no_browser:webbrowser.open(url)
                try:server.serve_forever(poll_interval=.25)
                finally:
                    scope.stop.set();worker.join(timeout=17)
                    if control:
                        try:probe.monitor_command(firmware,scope.kalman_sequence+1,False,mode=False)
                        except (OSError,RuntimeError,ValueError):pass  # Firmware lease expires within two seconds.
        finally:
            if csv_file:csv_file.close()
    except KeyboardInterrupt:return 0
    except (OSError,ValueError,RuntimeError,subprocess.TimeoutExpired) as error:
        print('Barometer viewer:',error);return 1
    finally:
        if probe is not None:probe.close()
    return 0

if __name__=='__main__':raise SystemExit(main())

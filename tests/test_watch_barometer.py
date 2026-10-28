import importlib.util
import json
import math
import os
import re
import shlex
import shutil
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

SCRIPT=Path(__file__).resolve().parents[1]/'tools'/'watch_barometer.py'
if not SCRIPT.exists():SCRIPT=Path('/tmp/fc-baroscope.py')
spec=importlib.util.spec_from_file_location('fc_baroscope',SCRIPT)
scope=importlib.util.module_from_spec(spec);sys.modules[spec.name]=scope;spec.loader.exec_module(scope)


def fixture(path, bad_size=False):
    names=b'\0meas\0uwTick\0last_tick\0alt_bias\0imedsv\0'
    symbols=[]
    for name,addr,size in [('meas',0x20000100,44 if bad_size else 48),('uwTick',0x20000200,4),('last_tick',0x20000204,4),('alt_bias',0x20000208,4),('imedsv',0x20000300,36)]:
        symbols.append(struct.pack('<IIIBBH',names.index(name.encode()),addr,size,1,0,2))
    raw=bytearray(0x500)
    raw[:16]=b'\x7fELF\x01\x01\x01'+b'\0'*9
    raw[16:52]=struct.pack('<HHIIIIIHHHHHH',2,40,1,0x08002201,52,0x400,0,52,32,2,40,5,0)
    raw[52:84]=struct.pack('<8I',1,0,0x08002000,0x08002000,0x210,0x210,5,0x1000)
    raw[84:116]=struct.pack('<8I',1,0x210,0x20000000,0x08002300,4,48,6,4)
    raw[0x200:0x210]=bytes(range(16));raw[0x210:0x214]=b'DATA'
    raw[0x280:0x280+len(names)]=names
    raw[0x300:0x300+len(symbols)*16]=b''.join(symbols)
    sections=[(0,0,0,0,0,0,0,0,0,0), (0,1,2,0x08002200,0x200,16,0,0,4,0),
              (0,1,3,0x20000000,0x210,4,0,0,4,0),
              (0,2,0,0,0x300,len(symbols)*16,4,0,4,16),
              (0,3,0,0,0x280,len(names),0,0,1,0)]
    for i,s in enumerate(sections):struct.pack_into('<10I',raw,0x400+i*40,*s)
    path.write_bytes(raw)
    return scope.Firmware(path)


def values(tick=10000,last=9990):
    m=bytearray(48);struct.pack_into('<3f',m,12,1.25,100100.,23.5)
    return {'meas':bytes(m),'uwTick':struct.pack('<I',tick),'last_tick':struct.pack('<I',last),
            'alt_bias':struct.pack('<f',.25),'prev_rel_alt':struct.pack('<f',1.25),'imedsv':struct.pack('<9f',0,0,0,1.1,.1,0,0,0,0)}

class BaroscopeTests(unittest.TestCase):
    def test_current_headers_preserve_supported_float_offsets(self):
        root=Path(os.environ.get('FC_BAROSCOPE_SOURCE',str(SCRIPT.parent.parent)))
        header=root/'Core/Inc/fcstructs.h'
        cc=shutil.which('cc')
        if not cc or not header.exists():self.skipTest('Source headers and a C compiler needed for ABI check')
        source=header.read_text()
        declarations=[]
        for alias in ('kf_gps','baro','f_xyz','measm','ekf_bias','kf_svec'):
            match=re.search(r'typedef struct serial \w+ \{[^}]*\} '+alias+r';',source,re.S)
            self.assertIsNotNone(match,alias)
            declarations.append(match[0])
        c='#include <stddef.h>\n#define serial __attribute__((packed,aligned(4)))\n'+'\n'.join(declarations)
        for expression in ('sizeof(float)==4','sizeof(measm)==48','offsetof(measm,baro)==12','offsetof(baro,prs)==4','offsetof(baro,tmp)==8','sizeof(kf_svec)==36','offsetof(kf_svec,alt)==12','offsetof(kf_svec,vel)==16'):
            c+='\n_Static_assert('+expression+',"baroscope ABI changed");'
        subprocess.run([cc,'-std=c11','-x','c','-fsyntax-only','-'],input=c,text=True,capture_output=True,check=True)
    def test_runtime_header_guard_rejects_same_size_field_reorders(self):
        root=Path(os.environ.get('FC_BAROSCOPE_SOURCE',str(SCRIPT.parent.parent)))
        scope.validate_layout(root)
        text=(root/'Core/Inc/fcstructs.h').read_text()
        with tempfile.TemporaryDirectory() as tmp:
            dest=Path(tmp)/'Core/Inc/fcstructs.h';dest.parent.mkdir(parents=True)
            dest.write_text(text.replace('float alt, prs, tmp;','float alt, tmp, prs;'))
            with self.assertRaisesRegex(ValueError,'baro: unsupported source layout'):
                scope.validate_layout(Path(tmp))
    def test_elf_sections_use_lma_and_exclude_elf_header_padding(self):
        with tempfile.TemporaryDirectory() as tmp:
            fw=fixture(Path(tmp)/'fc.elf')
            self.assertEqual(fw.flash,[(0x08002200,bytes(range(16))),(0x08002300,b'DATA')])
            self.assertEqual(fw.resolved['meas'],scope.Symbol(0x20000100,48))
    def test_rejects_changed_symbol_sizes_and_host_or_truncated_elf(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.elf'
            with self.assertRaisesRegex(ValueError,'48-byte'):fixture(p,True)
            for raw in (b'not elf',b'\x7fELF\x02\x01\x01'):
                p.write_bytes(raw)
                with self.assertRaises(ValueError):scope.Firmware(p)
    def test_offline_inspect_never_opens_a_probe(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'fc.elf';fixture(p)
            with patch.object(scope.subprocess,'run',side_effect=AssertionError('Probe must not be accessed')), patch.object(scope,'SOURCE_ROOT',Path(os.environ.get('FC_BAROSCOPE_SOURCE',str(SCRIPT.parent.parent)))), patch('builtins.print'):
                self.assertEqual(scope.main(['--elf',str(p),'--inspect']),0)
    def test_flash_identity_refuses_any_changed_loaded_byte(self):
        with tempfile.TemporaryDirectory() as tmp:
            fw=fixture(Path(tmp)/'fc.elf')
            class Probe:
                mismatch=False
                def read_blocks(self,requests):
                    addr,size=requests[0];data=dict(fw.flash)[addr]
                    return [data[:-1]+b'!' if self.mismatch else data]
            p=Probe();self.assertEqual(fw.verify(p),20);p.mismatch=True
            with self.assertRaisesRegex(ValueError,'Firmware mismatch'):fw.verify(p)
    def test_persistent_connection_reads_fragmented_replies_and_closes(self):
        process=Mock();process.poll.return_value=None
        connection=Mock()
        connection.recv.side_effect=[b'OK:0x3433',b'3231\x1a',b'OK:0x38373635\x1a']
        def start(command,**kwargs):
            kwargs['stdout'].write('Listening on port 12345 for tcl connections\n')
            kwargs['stdout'].flush()
            return process
        probe=scope.OpenOCDProbe('fake-cli','TESTSERIAL')
        with patch.object(scope.subprocess,'Popen',side_effect=start) as popen, \
             patch.object(scope.socket,'create_connection',return_value=connection) as connect:
            self.assertEqual(probe.read_blocks([(0x20000100,4)]),[b'1234'])
            self.assertEqual(probe.read_blocks([(0x20000104,4)]),[b'5678'])
            self.assertEqual(popen.call_count,1)
            self.assertEqual(connect.call_count,1)
            commands=popen.call_args.args[0][2::2]
            self.assertTrue(all(b'read_memory ' in call.args[0] for call in connection.sendall.call_args_list))
            self.assertTrue(all(call.args[0].endswith(b'\x1a') for call in connection.sendall.call_args_list))
            probe.close();probe.close()
            process.terminate.assert_called_once()
            connection.close.assert_called_once()
        self.assertIn('adapter serial TESTSERIAL',commands)
        self.assertIn('target create fc.cpu cortex_m -dap fc.dap -ap-num 1',commands)
        self.assertIn('bindto 127.0.0.1',commands)
        self.assertIn('tcl_port 0',commands)
        self.assertFalse(any(re.search(r'\b(reset|halt|resume|mww|mwh|mwb|flash)\b',c) for c in commands))
        self.assertFalse(any('board/' in c or 'target/' in c for c in commands))
    def test_serial_cannot_inject_tcl_commands(self):
        with self.assertRaisesRegex(ValueError,'Invalid ST-Link serial'):
            scope.OpenOCDProbe('fake','123;reset run')
    def test_short_reads_and_rpc_errors_close_without_reset(self):
        for result in ('0x1','0x1 0x2|0x3','not-memory'):
            probe=scope.OpenOCDProbe('fake','TEST')
            with patch.object(probe,'request',return_value=result) as request:
                with self.assertRaisesRegex(RuntimeError,'Short/invalid'):
                    probe.read_blocks([(0x20000100,8)])
                self.assertEqual(request.call_count,1)
        for response in (b'ERROR:busy\x1a',b'',TimeoutError()):
            probe=scope.OpenOCDProbe('fake','TEST')
            connection=Mock();probe.connection=connection
            connection.recv.side_effect=[response]
            with self.assertRaises(RuntimeError):probe.request('read_memory 0x20000100 32 1')
            connection.close.assert_called_once()
            self.assertIsNone(probe.connection)
    def test_batched_word_and_unaligned_byte_reads_preserve_order(self):
        probe=scope.OpenOCDProbe('fake','TEST')
        with patch.object(probe,'request',return_value='0x04030201|5 6 7') as request:
            self.assertEqual(probe.read_blocks([(0x20000100,4),(0x20000105,3)]),
                             [bytes([1,2,3,4]),bytes([5,6,7])])
            self.assertIn('read_memory 0x20000100 32 1',request.call_args.args[0])
            self.assertIn('read_memory 0x20000105 8 3',request.call_args.args[0])
    def test_large_flash_verification_uses_binary_dump_on_same_session(self):
        probe=scope.OpenOCDProbe('fake','TEST')
        expected=bytes(range(256))*300
        def dump(command):
            fields=shlex.split(command.split(';')[0])
            self.assertEqual(fields[0],'dump_image')
            self.assertEqual(int(fields[3]),len(expected))
            Path(fields[1]).write_bytes(expected)
            return '1'
        with patch.object(probe,'request',side_effect=dump) as request:
            self.assertEqual(probe.read_blocks([(0x08002200,len(expected))]),[expected])
            self.assertEqual(request.call_count,1)
            self.assertNotIn('read_memory',request.call_args.args[0])
    @unittest.skipUnless(os.environ.get('FC_BAROSCOPE_RPC_TEST'),'Opt-in local OpenOCD RPC integration test')
    def test_real_openocd_rpc_without_hardware(self):
        class LocalProbe(scope.OpenOCDProbe):
            def commands(self):
                return ['adapter driver dummy','transport select jtag','adapter speed 1000',
                        'bindto 127.0.0.1','gdb_port disabled','telnet_port disabled',
                        'tcl_port 0','init',
                        'proc read_memory {address width count} {lrepeat $count 0}']
        probe=LocalProbe(shutil.which('openocd') or 'openocd','TEST')
        try:
            self.assertEqual(probe.read_blocks([(0x20000000,8),(0x20000003,3)]),
                             [bytes(8),bytes(3)])
            process=probe.process
            for _ in range(20):self.assertEqual(probe.read_blocks([(0x20000000,4)]),[bytes(4)])
            self.assertIs(probe.process,process)
        finally:probe.close()
        self.assertIsNone(probe.process)
    def test_snapshot_allows_clock_and_imu_progress_rejects_torn_baro_and_halt(self):
        fw=types.SimpleNamespace(resolved={'meas':scope.Symbol(0x20000100,48),'uwTick':scope.Symbol(0x20000200,4),'last_tick':scope.Symbol(0x20000204,4)})
        for mode in ('good','torn','halt'):
            p=scope.OpenOCDProbe('unused','TEST')
            def read(requests):
                result=[];half=len(requests)//2
                for i,(addr,size) in enumerate(requests):
                    data=bytearray(size)
                    for name,sym in fw.resolved.items():
                        if addr<=sym.address<addr+size:
                            v=values(tick=10000+int(i>=half))[name]
                            if mode=='torn' and i>=half and name=='meas':v=bytes(48)
                            if mode=='good' and i>=half and name=='meas':v=b'IMU!'+v[4:]
                            data[sym.address-addr:sym.address-addr+len(v)]=v
                    if addr==scope.DHCSR and mode=='halt':struct.pack_into('<I',data,0,1<<17)
                    result.append(bytes(data))
                return result
            p.read_blocks=read
            if mode=='good':self.assertEqual(scope.decode(p.snapshot(fw))['pressure_pa'],100100.)
            else:
                with self.assertRaisesRegex(RuntimeError,'halted' if mode=='halt' else 'Inconsistent'):p.snapshot(fw)
    def test_decode_actual_offsets_units_and_tick_wrap(self):
        d=scope.decode(values(tick=3,last=0xfffffffe))
        self.assertEqual(d['barometer_age_ms'],5)
        self.assertEqual(d['pressure_pa'],100100.)
        self.assertEqual(d['temperature_c'],23.5)
        self.assertEqual(d['altitude_m'],1.25)
        self.assertEqual(d['before_bias_altitude_m'],1.5)
        self.assertAlmostEqual(d['predicted_velocity_m_s'],.1)
    def test_continuous_acquisition_between_uploads_keeps_monitoring(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE',resolved={
            'meas':scope.Symbol(0x20000100,48),
            'last_tick':scope.Symbol(0x20000140,4),
            'alt_bias':scope.Symbol(0x20000144,4),
            'uwTick':scope.Symbol(0x20000200,4)})
        probe=scope.OpenOCDProbe('unused','TEST')
        sample=0
        def read(requests):
            nonlocal sample
            result=[]
            half=len(requests)//2
            for i,(addr,size) in enumerate(requests):
                tick=10000+sample*1000+(20 if i>=half else 0)
                v=values(tick=tick,last=tick-1)
                m=bytearray(v['meas'])
                struct.pack_into('<f',m,16,100000.+sample+(1 if i>=half else 0))
                v['meas']=m
                v['alt_bias']=struct.pack('<f',tick/100000.)
                data=bytearray(size)
                for name,sym in fw.resolved.items():
                    if addr<=sym.address and sym.address+sym.size<=addr+size:
                        offset=sym.address-addr
                        data[offset:offset+sym.size]=v[name]
                result.append(bytes(data))
            sample+=1
            return result
        probe.read_blocks=read
        monitor=scope.Scope(fw,probe,1,10)
        for _ in range(5):monitor.sample()
        self.assertEqual(len(monitor.rows),5)
        self.assertEqual([r['pressure_pa'] for r in monitor.rows],
                         [100001.,100002.,100003.,100004.,100005.])
        self.assertEqual([r['sample_tick_ms'] for r in monitor.rows],
                         [10019,11019,12019,13019,14019])
        self.assertTrue(all(not r['stale'] for r in monitor.rows))
        self.assertFalse(monitor.stop.is_set())
    def test_invalid_measurement_is_not_published(self):
        v=values();v['meas']=bytes(48)
        with self.assertRaises(RuntimeError):scope.decode(v)
    def test_inconsistent_read_retries_are_bounded_and_can_recover(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        probe=types.SimpleNamespace(snapshot=None)
        monitor=scope.Scope(fw,probe,1,10)
        with patch.object(probe,'snapshot',side_effect=[
                scope.InconsistentRead('changing'),values()]) as read:
            monitor.sample()
            self.assertEqual(read.call_count,2)
            self.assertFalse(monitor.latest['stale'])
            self.assertEqual(len(monitor.rows),1)
        with patch.object(probe,'snapshot',side_effect=scope.InconsistentRead('changing')) as read:
            monitor.sample()
            self.assertEqual(read.call_count,3)
            self.assertTrue(monitor.latest['stale'])
        v=values();m=bytearray(v['meas']);struct.pack_into('<f',m,16,math.nan);v['meas']=m
        with self.assertRaises(RuntimeError):scope.decode(v)
    def test_scope_staleness_reset_history_bounds_and_error_gap(self):
        class Probe:
            def snapshot(self,_):return values()
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        p=Probe();s=scope.Scope(fw,p,1,2)
        s.sample();self.assertFalse(s.latest['stale'])
        s.last_progress=time_old=s.start-10;s.sample();self.assertTrue(s.latest['stale'])
        p.snapshot=lambda _:values(tick=20,last=10);s.sample();self.assertTrue(s.stop.is_set())
        p.snapshot=lambda _:(_ for _ in ()).throw(RuntimeError('probe disconnected'))
        s.sample();self.assertTrue(s.rows[-1]['stale']);self.assertIn('stale',s.status)
        self.assertLessEqual(len(s.rows),2)
    def test_incremental_history_cursors_and_overrun_recovery(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        probe=types.SimpleNamespace(snapshot=lambda _:values())
        monitor=scope.Scope(fw,probe,1,2)
        monitor.sample()
        first=monitor.payload()
        self.assertTrue(first['reset']);self.assertEqual(first['cursor'],1)
        monitor.sample()
        delta=monitor.payload(first['cursor'])
        self.assertFalse(delta['reset']);self.assertEqual(len(delta['rows']),1)
        self.assertEqual(monitor.payload(delta['cursor'])['rows'],[])
        for _ in range(3):monitor.sample()
        resumed=monitor.payload(first['cursor'])
        self.assertTrue(resumed['reset']);self.assertEqual(len(resumed['rows']),2)
        self.assertTrue(monitor.payload(10000)['reset'])
        self.assertGreaterEqual(resumed['read_duration_ms'],0)
        self.assertGreaterEqual(resumed['host_sample_age_ms'],0)
    def test_one_hour_of_direct_samples_is_retained_at_default_rate(self):
        monitor=scope.Scope(types.SimpleNamespace(),None,20,3600)
        self.assertEqual(monitor.rows.maxlen,72000)
    @unittest.skipUnless(shutil.which('node'),'Node needed for browser graph calculations')
    def test_browser_plots_spikes_directly_and_holds_paused_time_range(self):
        script=scope.PAGE.split('<script>',1)[1].split('</script>',1)[0]
        harness=r'''
const assert=require('node:assert/strict'),vm=require('node:vm');
const elements=new Map();
const context={console,Promise,Math,Number,Intl,
 setInterval(){},requestAnimationFrame(){},fetch:()=>new Promise(()=>{}),
 window:{devicePixelRatio:1,addEventListener(){}},
 document:{getElementById(id){if(!elements.has(id))elements.set(id,{addEventListener(){}});return elements.get(id)}}};
vm.createContext(context);
vm.runInContext(SCRIPT,context);
vm.runInContext(`
const strokes=[],c={strokeStyle:'',scale(){},clearRect(){},fillText(){},save(){},restore(){},rect(){},clip(){},arc(){},fill(){},setLineDash(){},
 beginPath(){this.path=[]},moveTo(x,y){this.path.push(['move',x,y])},lineTo(x,y){this.path.push(['line',x,y])},stroke(){if(this.strokeStyle==='#63d6c1')strokes.push(this.path)}};
const canvas={clientWidth:800,clientHeight:400,getContext:()=>c};
rows=[{elapsed_s:0,pressure_pa:0},{elapsed_s:1,pressure_pa:100},{elapsed_s:2,pressure_pa:0}];
const plot=graph(canvas,'pressure_pa',0,2);
globalThis.result={count:plot.stats.count,min:plot.stats.min,max:plot.stats.max,path:strokes.at(-1)};
`,context);
assert.equal(context.result.count,3);assert.equal(context.result.max,100);
assert.equal(context.result.path.length,3);
assert(context.result.path[1][2]<context.result.path[0][2]);
assert.equal(context.result.path[0][2],context.result.path[2][2]);
vm.runInContext(`
rows=Array.from({length:72000},(_,i)=>({elapsed_s:i/20,pressure_pa:2*i/20}));
globalThis.longStats=seriesStats(rows,'pressure_pa',0,3600);
globalThis.pausedBefore=bounds(10,20);rows.push({elapsed_s:4000,pressure_pa:8000});globalThis.pausedAfter=bounds(10,20);
`,context);
assert.equal(context.longStats.count,72000);
assert(Math.abs(context.longStats.perHour-7200)<1e-6);
assert.equal(context.pausedBefore.start,context.pausedAfter.start);
assert.equal(context.pausedBefore.end,context.pausedAfter.end);
'''.replace('SCRIPT',json.dumps(script))
        subprocess.run([shutil.which('node'),'-e',harness],check=True,capture_output=True,text=True)
    @unittest.skipUnless(os.environ.get('FC_BAROSCOPE_ELF'),'Set FC_BAROSCOPE_ELF for actual build offline resolution')
    def test_actual_selected_build_offline_symbol_resolution(self):
        fw=scope.Firmware(os.environ['FC_BAROSCOPE_ELF'])
        self.assertEqual(fw.resolved['meas'].size,48)
        self.assertGreater(sum(len(d) for _,d in fw.flash),1000)

if __name__=='__main__':unittest.main()

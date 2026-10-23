import importlib.util
import json
import math
import os
import re
import shutil
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

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
    def test_cli_uses_only_hotplug_uploads_and_probe_serial(self):
        calls=[]
        def run(command,**kwargs):
            calls.append(command)
            i=command.index('-u');Path(command[i+3]).write_bytes(b'1234')
            return types.SimpleNamespace(returncode=0)
        probe=scope.CubeProbe('fake-cli','TEST_SERIAL',runner=run)
        self.assertEqual(probe.read_blocks([(0x20000100,4)]),[b'1234'])
        self.assertIn('mode=HOTPLUG',calls[0]);self.assertIn('sn=TEST_SERIAL',calls[0])
        self.assertFalse(any(x in calls[0] for x in ['-halt','-run','-rst','-w','-w32','reset=HWrst','shared']))
    def test_short_read_busy_and_timeout_never_retry_with_reset(self):
        for mode in ('short','busy','timeout'):
            calls=[]
            def run(command,**_):
                calls.append(command)
                if mode=='timeout':raise subprocess.TimeoutExpired(command,1)
                if mode=='short':Path(command[-1]).write_bytes(b'x')
                return types.SimpleNamespace(returncode=1 if mode=='busy' else 0)
            with self.assertRaises(RuntimeError):scope.CubeProbe('fake','TEST',runner=run).read_blocks([(0x20000100,4)])
            self.assertEqual(len(calls),1)
    def test_snapshot_allows_clock_and_imu_progress_rejects_torn_baro_and_halt(self):
        fw=types.SimpleNamespace(resolved={'meas':scope.Symbol(0x20000100,48),'uwTick':scope.Symbol(0x20000200,4),'last_tick':scope.Symbol(0x20000204,4)})
        for mode in ('good','torn','halt'):
            p=scope.CubeProbe('unused','TEST')
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
    def test_invalid_measurement_is_not_published(self):
        v=values();v['meas']=bytes(48)
        with self.assertRaises(RuntimeError):scope.decode(v)
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
    @unittest.skipUnless(os.environ.get('FC_BAROSCOPE_ELF'),'Set FC_BAROSCOPE_ELF for actual build offline resolution')
    def test_actual_selected_build_offline_symbol_resolution(self):
        fw=scope.Firmware(os.environ['FC_BAROSCOPE_ELF'])
        self.assertEqual(fw.resolved['meas'].size,48)
        self.assertGreater(sum(len(d) for _,d in fw.flash),1000)

if __name__=='__main__':unittest.main()

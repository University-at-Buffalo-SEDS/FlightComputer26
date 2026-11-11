import importlib.util
import csv
import io
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

SCRIPT=Path(os.environ.get('FC_WATCH_CAUSAL_SCRIPT',str(Path(__file__).resolve().parents[1]/'tools'/'watch_barometer.py')))
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
    def _run_actual_main_monitor_boundary(self, options=(), mismatch=False):
        # Exercise actual CLI/main/Scope.sample/shutdown. Only physical probe
        # access and server scheduling are mocked; no hardware is opened.
        with tempfile.TemporaryDirectory() as tmp:
            fw=fixture(Path(tmp)/'fc.elf')
            fw.resolved['g_fc_monitor']=scope.Symbol(0x20000400,48)
            observed=values();observed['g_fc_monitor']=struct.pack('<6I6f',scope.MONITOR_MAGIC,0,0,100,0,0,0,0,1,0,0,0)
            probe=Mock();probe.snapshot.return_value=observed
            probe.read_blocks.return_value=[struct.pack('<I',scope.MONITOR_MAGIC)]
            class Worker:
                def __init__(self,target,**kwargs):self.target=target
                def start(self):self.target.__self__.sample()
                def join(self,**kwargs):pass
            class Server:
                server_port=9876
                def __init__(self,*args):pass
                def __enter__(self):return self
                def __exit__(self,*args):pass
                def serve_forever(self,**kwargs):pass
            matching={'side_effect':ValueError('firmware mismatch')} if mismatch else {'return_value':(fw,20)}
            with patch.object(scope,'OpenOCDProbe',return_value=probe),patch.object(scope,'matching_firmware',**matching),patch.object(scope,'Firmware',return_value=fw),patch.object(scope,'describe_firmware'),patch.object(scope,'ThreadingHTTPServer',Server),patch.object(scope.threading,'Thread',Worker),patch('builtins.print'):
                result=scope.main(['--elf',str(fw.path),'--serial','TEST','--openocd','mock-probe','--no-browser',*options])
            return result,probe

    def test_actual_main_default_matching_firmware_never_writes_monitor(self):
        result,probe=self._run_actual_main_monitor_boundary()
        self.assertEqual(result,0)
        probe.snapshot.assert_called_once()
        probe.monitor_command.assert_not_called()
        probe.request.assert_not_called()

    def test_actual_main_explicit_bench_renews_and_exits_only_command_mailbox(self):
        result,probe=self._run_actual_main_monitor_boundary(['--bench'])
        self.assertEqual(result,0)
        self.assertEqual(probe.monitor_command.call_count,2)
        self.assertEqual(probe.monitor_command.call_args_list[0].args[2:],(False,False))
        self.assertEqual(probe.monitor_command.call_args_list[1].kwargs,{'mode':False})
        probe.request.assert_not_called()

    def test_actual_main_mismatch_override_cannot_enable_bench(self):
        result,probe=self._run_actual_main_monitor_boundary(['--bench','--allow-mismatched-firmware'],mismatch=True)
        self.assertEqual(result,1)
        probe.monitor_command.assert_not_called()
        probe.request.assert_not_called()

    def test_explicit_bench_conflicts_refuse_before_probe_access(self):
        with patch.object(scope,'OpenOCDProbe') as probe:
            for options in [['--enable-kalman'],['--bench','--read-only'],['--bench','--inspect']]:
                with self.subTest(options=options),self.assertRaises(SystemExit) as error:
                    scope.main(options)
                self.assertEqual(error.exception.code,2)
            probe.assert_not_called()

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
        # The single auto-detected probe is opened without a serial filter.
        self.assertFalse(any(c.startswith('adapter serial') for c in scope.OpenOCDProbe('fake-cli',None).commands()))
        self.assertIn('target create fc.cpu cortex_m -dap fc.dap -ap-num 1',commands)
        self.assertIn('bindto 127.0.0.1',commands)
        self.assertIn('tcl_port 0',commands)
        self.assertFalse(any(re.search(r'\b(reset|halt|resume|mww|mwh|mwb|flash)\b',c) for c in commands))
        self.assertFalse(any('board/' in c or 'target/' in c for c in commands))
    def test_connection_failure_preserves_actual_openocd_error_after_cleanup(self):
        process=Mock();process.poll.return_value=1
        def start(command,**kwargs):
            kwargs['stdout'].write('OpenOCD startup\nError: libusb_open() failed with LIBUSB_ERROR_ACCESS\n')
            kwargs['stdout'].flush()
            return process
        probe=scope.OpenOCDProbe('fake','TEST')
        with patch.object(scope.subprocess,'Popen',side_effect=start):
            with self.assertRaisesRegex(RuntimeError,'LIBUSB_ERROR_ACCESS'):probe.connect()
        self.assertIsNone(probe.directory)
        self.assertIsNone(probe.process)

    def test_flagless_selection_verifies_flash_instead_of_assuming_debug_or_newest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            debug=root/'build'/'Debug';debug.mkdir(parents=True)
            release=root/'build'/'Release';release.mkdir()
            old=fixture(debug/'FlightComputer26.elf')
            fixture(release/'FlightComputer26.elf')
            raw=bytearray((release/'FlightComputer26.elf').read_bytes());raw[0x200]=255
            (release/'FlightComputer26.elf').write_bytes(raw)
            os.utime(old.path,ns=(1,1))
            paths=scope.local_firmware_paths(root)
            self.assertEqual(paths[0].parent,release)
            probe=types.SimpleNamespace(read_blocks=lambda requests:[dict(old.flash)[requests[0][0]]])
            selected,size=scope.matching_firmware(probe,paths)
            self.assertEqual(selected.path,old.path)
            self.assertEqual(size,20)
            with self.assertRaisesRegex(ValueError,'No local ELF matches'):
                scope.matching_firmware(probe,[paths[0]])

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
    def test_imu_offsets_units_and_fc_bench_outputs(self):
        v=values();m=bytearray(v['meas'])
        struct.pack_into('<6f',m,24,1,2,3,-4,-5,-6);v['meas']=bytes(m)
        v['g_fc_monitor']=struct.pack('<6I6f',scope.MONITOR_MAGIC,0,2,9999,5,2,10,20,1,0,0,0)
        row=scope.decode(v)
        self.assertEqual([row[f'accel_{axis}'] for axis in 'xyz'],[1,2,3])
        self.assertEqual([row[f'gyro_{axis}_dps'] for axis in 'xyz'],[-4,-5,-6])
        self.assertEqual(row['fc_kalman_altitude_m'],10)
        self.assertEqual(row['fc_kalman_velocity_m_s'],20)
    def test_bench_control_writes_only_versioned_mailbox_command_word(self):
        probe=scope.OpenOCDProbe('unused','TEST')
        firmware=types.SimpleNamespace(resolved={'g_fc_monitor':scope.Symbol(0x20000100,48)})
        with patch.object(probe,'read_blocks',return_value=[struct.pack('<I',scope.MONITOR_MAGIC)]),patch.object(probe,'request') as request:
            probe.monitor_command(firmware,7,True,True)
            self.assertEqual(request.call_args.args[0],'mww 0x20000104 0xe0000007')
            probe.monitor_command(firmware,8,False)
            self.assertEqual(request.call_args.args[0],'mww 0x20000104 0x80000008')
            probe.monitor_command(firmware,9,False,mode=False)
            self.assertEqual(request.call_args.args[0],'mww 0x20000104 0x9')
        with patch.object(probe,'read_blocks',return_value=[bytes(4)]),patch.object(probe,'request') as request:
            with self.assertRaisesRegex(ValueError,'ABI'):probe.monitor_command(firmware,9,True)
            request.assert_not_called()
    def test_kalman_start_stop_reset_are_queued_and_leased_by_sample_owner(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        probe=types.SimpleNamespace(snapshot=lambda _:values(),monitor_command=Mock())
        monitor=scope.Scope(fw,probe,20,10,kalman_control=True)
        for action,enabled,reset in [('start',True,False),('reset',True,True),('stop',False,False)]:
            monitor.kalman_action(action);monitor.sample()
            self.assertEqual(probe.monitor_command.call_args.args[2:],(enabled,reset))
        monitor.kalman_sent-=1;monitor.sample()
        self.assertEqual(probe.monitor_command.call_args.args[2:],(False,False))
        calls=probe.monitor_command.call_count;monitor.sample()
        self.assertEqual(probe.monitor_command.call_count,calls)
        monitor.kalman_action('reset');monitor.sample()
        self.assertEqual(probe.monitor_command.call_args.args[2:],(False,True))
        monitor.kalman_action('start');monitor.sample();monitor.kalman_sent-=1;monitor.sample()
        self.assertGreater(probe.monitor_command.call_count,calls+1)
        with self.assertRaises(ValueError):scope.Scope(fw,probe,20,10).kalman_action('start')
    def test_slow_sampling_renews_monitor_lease_while_filter_is_stopped(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        probe=types.SimpleNamespace(snapshot=lambda _:values(),monitor_command=Mock())
        monitor=scope.Scope(fw,probe,.1,10,kalman_control=True)
        import threading
        timer=threading.Timer(.8,monitor.stop.set)
        timer.start()
        try:monitor.run()
        finally:timer.cancel()
        self.assertEqual(len(monitor.rows),1)
        self.assertGreaterEqual(probe.monitor_command.call_count,2)
        self.assertEqual(probe.monitor_command.call_args.args[2:],(False,False))

    def test_csv_roundtrip_replay_seek_pause_speed_and_recorded_estimates(self):
        fw=types.SimpleNamespace(path='fake.elf',sha256='FAKE')
        monitor=scope.Scope(fw,types.SimpleNamespace(snapshot=lambda _:values()),20,10)
        monitor.sample();row=dict(monitor.latest,elapsed_s=0,fc_kalman_altitude_m=12.5,accel_x=3)
        monitor.rows.clear()
        for t in (0,1,2):monitor.rows.append(dict(row,elapsed_s=t))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'session.csv';p.write_text(monitor.export_csv())
            replay=scope.ReplayScope(p,10)
            first=replay.payload();self.assertEqual(first['latest']['accel_x'],3)
            self.assertEqual(first['latest']['fc_kalman_altitude_m'],12.5)
            replay.playback_action('seek',2)
            end=replay.payload(first['cursor']);self.assertTrue(end['reset']);self.assertEqual(len(end['rows']),3)
            replay.playback_action('restart');self.assertEqual(replay.payload()['replay']['position'],0)
            replay.playback_action('speed',2);replay.playback_action('play');replay.anchor-=.6
            self.assertGreaterEqual(replay.payload()['replay']['position'],1.2)
            replay.playback_action('pause');position=replay.position
            self.assertEqual(replay.payload()['replay']['position'],position)
            self.assertEqual(len(list(csv.DictReader(io.StringIO(replay.export_csv())))),3)
            with self.assertRaises(ValueError):replay.kalman_action('start')
            for value in (math.nan,math.inf):
                with self.assertRaises(ValueError):replay.playback_action('seek',value)
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

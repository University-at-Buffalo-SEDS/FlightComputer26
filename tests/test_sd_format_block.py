"""SEDS-SD format block: schema hash, frozen versions, grammar and label coverage."""
import hashlib
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

# Every released FC layout. A record layout change must bump SD_FORMAT_VERSION
# and add its hash here; the old entries stay so the GS keeps reading old cards.
FROZEN = {2: 'e59ae8a7'}
BUMP = ('SD record layout changed: bump SD format version, add the frozen hash '
        'and register the version in the GroundStation26 SD parser')

COLUMNS = ('# seds-sd-columns,u64:ms,u64:ms,label,vec,vec,vec,vec,clock,f32:degC,'
           'i32:code,u64:tick,u32:Hz,u64:seq')
UNIT = r'(?:[A-Za-z0-9/]+(?:\*[1-9][0-9]*)?)'
ENTRY = re.compile(r'^(?P<label>[A-Za-z0-9_]+\*?)'
                   r'(?P<cols>(?::[varc]=' + UNIT + r'(?:;' + UNIT + r')*)+)'
                   r'(?:@(?P<hz>[1-9][0-9]*))?$')


def header_text(defines=()):
    code = r'''#include "sd_record_format.h"
int main(void) { fputs(SD_FILE_HEADER, stdout); return 0; }
'''
    with tempfile.TemporaryDirectory() as tmp:
        exe = pathlib.Path(tmp) / 'header'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', *defines, '-I', str(ROOT / 'Core/Inc'),
                        '-x', 'c', '-', '-o', str(exe)], input=code, text=True, check=True)
        out = subprocess.run([str(exe)], capture_output=True, check=True).stdout
    return out.decode('ascii')


def schema(lines):
    hashed = ''.join(l + '\r\n' for l in lines if l.startswith(('# seds-sd-columns,', '# seds-sd-records,')))
    return hashlib.sha256(hashed.encode('ascii')).hexdigest()[:8]


class SdFormatBlockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = header_text()
        cls.lines = cls.text.split('\r\n')
        cls.ident = dict(kv.split('=', 1) for kv in cls.lines[1].split(',')[1:])
        cls.entries = [e for l in cls.lines if l.startswith('# seds-sd-records,')
                       for e in l.split(',')[1:]]

    def test_layout_is_frozen_per_version(self):
        text = self.text
        self.assertTrue(text.endswith('\r\n') and '\n' not in text.replace('\r\n', ''))
        computed = schema(self.lines)
        self.assertEqual(self.ident['schema'], computed, 'SD format header damaged: schema hash mismatch')
        version = int(self.ident['version'])
        self.assertIn(version, FROZEN, BUMP)
        self.assertEqual(FROZEN[version], computed, BUMP)
        # Hash excludes lines 1-2: the firmware revision never changes it.
        other = header_text(['-DFC_FIRMWARE_REV="0123456789ab"']).split('\r\n')
        self.assertIn(',firmware=0123456789ab,', other[1])
        self.assertEqual(schema(other), computed)

    def test_grammar(self):
        lines = self.lines
        self.assertEqual(lines[0], 'timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,'
                         'calibrated_value,time_source,adc_temperature_c,adc_temperature_code,'
                         'uptime_ticks,uptime_frequency_hz,record_sequence')
        self.assertRegex(lines[1], r'^# seds-sd,format=fc,version=[1-9][0-9]*,board=FC,'
                         r'firmware=(?:[0-9a-f]{12}(?:-dirty)?|unknown),tick_hz=1000,schema=[0-9a-f]{8}$')
        self.assertEqual(self.ident['firmware'], 'unknown')  # default without CMake
        self.assertEqual(lines[2], COLUMNS)
        self.assertEqual(len(lines[0].split(',')), len(COLUMNS.split(',')) - 1)
        records = [l for l in lines[3:] if l]
        self.assertTrue(records and all(l.startswith('# seds-sd-records,') for l in records))
        labels = set()
        for entry in self.entries:
            match = ENTRY.match(entry)
            self.assertIsNotNone(match, entry)
            self.assertNotIn(match['label'], labels, entry)
            labels.add(match['label'])
            cols = [c.split('=')[0] for c in match['cols'][1:].split(':')]
            self.assertEqual(len(cols), len(set(cols)), entry)
            self.assertEqual(cols[0], 'v', entry)
        self.assertLess(len(self.text), 1024)  # few hundred bytes of rodata

    def width(self, label, col='v'):
        entry = next(e for e in self.entries if e.startswith(label + ':'))
        units = dict(c.split('=') for c in entry.split(':')[1:])[col]
        return sum(int(u.split('*')[1]) if '*' in u else 1 for u in units.split(';'))

    def test_vector_widths_match_the_writers(self):
        dist = (ROOT / 'Core/Src/distribution.c').read_text()
        self.assertRegex(dist, r'\.size = 6,\s*\.kind_sd = SEDS_DT_IMU_LOCAL')
        self.assertRegex(dist, r'\.size = 3,\s*\.kind_sd = SEDS_DT_BAROMETER_LOCAL')
        self.assertEqual(self.width('IMU'), 6)
        self.assertEqual(self.width('BARO'), 3)
        self.assertEqual(self.width('EKF'), 6)
        self.assertEqual(self.width('DKF'), 4)
        self.assertEqual(self.width('QUAT'), 6)
        self.assertEqual(self.width('EULER'), 3)
        self.assertEqual(self.width('SIM'), 8)
        for sensor in ('accelerometer_sample', 'gyroscope_sample', 'barometer_sample'):
            for col in 'varc':
                self.assertEqual(self.width(sensor, col), 3, (sensor, col))
        self.assertIn('sd_append_f32(SEDS_DT_SIM_SAMPLE, v, 8)', (ROOT / 'Core/Src/ground_test.c').read_text())
        self.assertIn('sd_append_f32(SEDS_DT_EULER_ANGLES, (float *)&rep_in_deg, 3)',
                      (ROOT / 'Core/Src/kalman.c').read_text())

    def test_every_written_label_is_declared(self):
        """Best effort: the label literals of every SD writer path."""
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        found = set()
        for start, end in (('seds_msg(SedsDataType ty)', '\n}\n'), ('seds_f32(SedsDataType ty)', '\n}\n'),
                           ('names[3] = {', '};'), ('labels[] = {', '};')):
            body = src[src.index(start):]
            found |= set(re.findall(r'"([A-Za-z0-9_]+)"', body[:body.index(end)]))
        self.assertGreaterEqual(len(found), 20)
        self.assertTrue({'Message', 'Ordered', 'IMU', 'BARO', 'EKF', 'DKF', 'EULER', 'QUAT', 'SIM',
                         'launch_accepted', 'launch_detected', 'pyro_dry_run',
                         'accelerometer_sample'} <= found)
        labels = [ENTRY.match(e)['label'] for e in self.entries]
        for name in sorted(found):
            self.assertTrue(any(l == name or (l.endswith('*') and name.startswith(l[:-1])) for l in labels),
                            f'SD label {name!r} is written but not declared in SD_FORMAT_RECORDS')
        # No SD writer bypasses the labelled paths.
        for path in (ROOT / 'Core/Src').glob('*.c'):
            text = path.read_text()
            if path.name != 'sdpipeline.c':
                self.assertNotIn('sd_format_csv', text, path.name)
                self.assertNotIn('sd_append_ready', text, path.name)

    def test_every_file_and_part_starts_with_the_block(self):
        src = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        self.assertEqual(src.count('fx_file_create('), 1)
        self.assertEqual(src.count('FX_OPEN_FOR_WRITE'), 1)
        opener = src[src.index('static inline int sd_pipeline_open(const char *surprise)\n{'):]
        opener = opener[:opener.index('\nvoid sd_pipeline_task()')]
        self.assertIn('fx_file_create(', opener)
        self.assertIn('sd_file_open = 2U;', opener)
        self.assertIn('static const char header[] = SD_FILE_HEADER;', opener)
        # Boot, launch rotation, Landed cut-off and 3 GiB parts all reopen through it.
        self.assertGreaterEqual(src.count('sd_pipeline_open("'), 2)
        for path in list((ROOT / 'Core/Src').glob('*.c')) + list((ROOT / 'FileX/App').glob('*.c')):
            text = path.read_text()
            self.assertNotRegex(text, r'\bSD_CSV_HEADER\b', path.name)
            if path.name != 'sdpipeline.c':
                self.assertNotIn('FX_OPEN_FOR_WRITE', text, path.name)

    def test_cmake_injects_the_revision_with_fallback(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('git rev-parse --short=12 HEAD', cmake)
        self.assertIn('set(FC_FIRMWARE_REV "unknown")', cmake)
        self.assertIn('COMPILE_DEFINITIONS FC_FIRMWARE_REV="${FC_FIRMWARE_REV}"', cmake)


if __name__ == '__main__':
    unittest.main()

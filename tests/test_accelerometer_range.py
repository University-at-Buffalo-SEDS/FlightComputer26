"""BMI088 accelerometer range/scale regression (no hardware).

The driver used to write ACC_CONF and ACC_RANGE back to back while the
accelerometer was still in suspend mode after its soft reset, so the range
write was lost and the part stayed at its +/-6 g reset range. A hand-tuned
FC26_LINEAR_SCALE then made gravity read correctly at rest, but every boost
above ~6 g saturated. The driver must power the part on first, read the range
back and scale with the datasheet factor in m/s^2.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'Core/Src/accelerometer.c').read_text()
HEADER = (ROOT / 'Core/Inc/accelerometer.h').read_text()


def init_body():
    start = SOURCE.index('accl_init(SPI_HandleTypeDef *hspi')
    return SOURCE[start:SOURCE.index('\n}\n', start)]


class AccelerometerRangeTests(unittest.TestCase):
    def test_power_on_precedes_configuration_writes(self):
        body = init_body()
        position = {name: body.index(f'accl_write_reg(hspi, {name},') for name in
                    ('ACCL_RESET', 'ACCL_PWR_CONF', 'ACCL_PWR_CTRL', 'ACCL_CONF', 'ACCL_RANGE')}
        self.assertLess(position['ACCL_RESET'], position['ACCL_PWR_CONF'])
        self.assertLess(position['ACCL_PWR_CONF'], position['ACCL_PWR_CTRL'])
        self.assertLess(position['ACCL_PWR_CTRL'], position['ACCL_CONF'])
        self.assertLess(position['ACCL_CONF'], position['ACCL_RANGE'])
        between = body[position['ACCL_PWR_CTRL']:position['ACCL_CONF']]
        self.assertIn('HAL_Delay(ACCL_WAIT_MS)', between)
        self.assertIn('HAL_Delay(', body[position['ACCL_CONF']:position['ACCL_RANGE']])

    def test_scale_comes_from_the_range_read_back(self):
        body = init_body()
        readback = body.index('accl_read_reg(hspi, ACCL_RANGE, &range)')
        self.assertGreater(readback, body.index('accl_write_reg(hspi, ACCL_RANGE,'))
        self.assertGreater(body.index('lsb_to_g = accl_lsb_to_mps2(range)'), readback)
        self.assertNotIn('valid.rng + 0x01', body)

    def test_no_hand_tuned_scale_remains(self):
        for text in (SOURCE, HEADER):
            self.assertNotIn('FC26_LINEAR_SCALE', text)
            self.assertNotIn('SCALE_HEURISTIC', text)

    def test_datasheet_scale_in_mps2(self):
        cc = shutil.which('cc') or shutil.which('gcc')
        if not cc:
            self.skipTest('host C compiler unavailable')
        macro = re.search(r'#define accl_lsb_to_mps2\(rng\)\s*\\\n(.*?)\n\n', HEADER, re.S)[1]
        program = ('#include <stdio.h>\n#define accl_lsb_to_mps2(rng) ' + macro.replace('\\\n', ' ') +
                   '\nint main(void){for(unsigned r=0;r<4;r++)printf("%.9f\\n",(double)accl_lsb_to_mps2(r));return 0;}\n')
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'scale.c', Path(tmp) / 'scale'
            src.write_text(program)
            subprocess.run([cc, '-std=c11', '-Wall', '-Werror', str(src), '-o', str(exe)], check=True)
            values = [float(v) for v in subprocess.check_output([str(exe)], text=True).split()]
        for rng, full_g in enumerate((3, 6, 12, 24)):
            self.assertAlmostEqual(values[rng] * 32768, full_g * 9.80665, places=3)
        # 1 g at +/-24 g is 1365 LSB; the boost of a 9.5 g motor no longer saturates.
        self.assertAlmostEqual(1365.33 * values[3], 9.80665, places=2)
        self.assertGreater(32767 * values[3], 9.5 * 9.80665)


if __name__ == '__main__':
    unittest.main()

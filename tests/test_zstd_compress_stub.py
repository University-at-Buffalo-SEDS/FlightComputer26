"""The FC never compresses SEDSnet payloads, so it links a ZSTD_compress stub
instead of zstd's compressor (Core/Src/zstd_compress_stub.c, ~27 KB of flash).

Guards the two halves of that contract: the router keeps compression off,
and the stub reports a zstd error so any call falls back to an uncompressed
payload."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class ZstdCompressStubTests(unittest.TestCase):
    def test_compression_stays_off_where_the_stub_is_linked(self):
        tel = (ROOT / 'Core/Src/telemetry.c').read_text()
        self.assertIn('tuning.payload_compress_threshold = SIZE_MAX;', tel)
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        sources = cmake[cmake.index('${CMAKE_SOURCE_DIR}/Core/Src/telemetry.c'):]
        sources = sources[:sources.index(')')]
        self.assertIn('Core/Src/zstd_compress_stub.c', sources, 'linked exactly where telemetry.c is')

    def test_stub_reports_a_zstd_error(self):
        code = (ROOT / 'Core/Src/zstd_compress_stub.c').read_text() + r'''
#include <stdio.h>
int main(void)
{
    unsigned char out[8];
    const size_t r = ZSTD_compress(out, sizeof out, "abc", 3, 1);
    /* zstd: ZSTD_isError(code) is code > (size_t)-ZSTD_error_maxCode (120). */
    printf("%d\n", r > (size_t)-120);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe = pathlib.Path(tmp) / 'stub'
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-x', 'c', '-', '-o', str(exe)],
                           input=code, text=True, check=True)
            self.assertEqual(subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.strip(), '1')
        self.assertIsNotNone(re.search(r'size_t ZSTD_compress\(void \*dst, size_t dst_capacity, const void \*src, size_t src_size,\s+int compression_level\)\n\{',
                                       (ROOT / 'Core/Src/zstd_compress_stub.c').read_text()))


if __name__ == '__main__':
    unittest.main()

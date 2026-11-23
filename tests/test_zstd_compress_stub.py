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


PROGRAM = r'''
#include "sedsnet.h"
#include <stdio.h>
#include <string.h>
static void hex(const uint8_t *p, size_t n) { for (size_t i = 0; i < n; ++i) printf("%02x", p[i]); printf("\n"); }
static int from_hex(const char *s, uint8_t *out) { int n = 0; while (s[0] && s[1] && sscanf(s, "%2hhx", &out[n]) == 1) { ++n; s += 2; } return n; }
int main(int argc, char **argv)
{
    SedsRuntimeTuningConfig tuning;
    if (seds_get_runtime_tuning_config(&tuning) != SEDS_OK) return 2;
    tuning.payload_compress_threshold = 16;           /* try to compress everything */
    if (seds_set_runtime_tuning_config(&tuning) != SEDS_OK) return 3;
    char text[400];
    for (unsigned i = 0; i < sizeof text; ++i) text[i] = "compressible SD service text "[i % 29];
    const uint32_t ep = SEDS_EP_TELEMETRY_ERROR;
    SedsPacketView v = {.ty = SEDS_DT_TELEMETRY_ERROR, .data_size = sizeof text, .sender = "FC", .sender_len = 2,
                        .endpoints = &ep, .num_endpoints = 1, .timestamp = 1234, .payload = (const uint8_t *)text,
                        .payload_len = sizeof text};
    static uint8_t frame[4096];
    if (argc > 1) {                                   /* receive a frame packed elsewhere */
        const int n = from_hex(argv[1], frame);
        SedsOwnedPacket *pkt = seds_pkt_unpack_owned(frame, (size_t)n);
        SedsPacketView got;
        if (!pkt || seds_owned_pkt_view(pkt, &got) != SEDS_OK) { puts("unpack failed"); return 4; }
        printf("RX %d %d\n", (int)got.payload_len, got.payload_len == sizeof text && !memcmp(got.payload, text, sizeof text));
        seds_owned_pkt_free(pkt);
    }
    const int32_t n = seds_pkt_pack(&v, frame, sizeof frame);
    if (n <= 0) return 5;
    printf("TX %d %d ", (int)n, frame[0] & 1);       /* frame flag bit0: payload compressed */
    hex(frame, (size_t)n);
    return 0;
}
'''


class FcDecompressionTests(unittest.TestCase):
    """With the stub linked first, the FC still unpacks compressed frames from
    other nodes, and its own packing falls back to uncompressed frames."""

    def test_compressed_frames_still_decode_with_the_stub(self):
        sys_path = str(ROOT / 'tests')
        import sys
        if sys_path not in sys.path:
            sys.path.insert(0, sys_path)
        from native_sdk import prepare_native_sdk
        sdk = prepare_native_sdk(ROOT)
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / 'p.c'
            src.write_text(PROGRAM)
            base = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-I' + str(pathlib.Path(sdk['header']).parent)]
            libs = [sdk['archive'], '-lpthread', '-lm', '-ldl']
            peer, fc = pathlib.Path(tmp) / 'peer', pathlib.Path(tmp) / 'fc'
            subprocess.run(base + [str(src)] + libs + ['-o', str(peer)], check=True)
            # The host SDK is an unoptimised archive that still pulls zstd's
            # compressor for other symbols; the stub, linked first, wins.
            subprocess.run(base + [str(src), str(ROOT / 'Core/Src/zstd_compress_stub.c')] + libs +
                           ['-Wl,--allow-multiple-definition', '-o', str(fc)], check=True)
            tx = subprocess.run([str(peer)], capture_output=True, text=True, check=True).stdout.split()
            self.assertEqual(tx[2], '1', 'the peer compresses this payload')
            out = subprocess.run([str(fc), tx[3]], capture_output=True, text=True, check=True).stdout.splitlines()
            self.assertEqual(out[0], 'RX 400 1', 'the FC decompresses it')
            own = out[1].split()
            self.assertEqual(own[2], '0', 'the FC packs it uncompressed')
            back = subprocess.run([str(peer), own[3]], capture_output=True, text=True, check=True).stdout.splitlines()
            self.assertEqual(back[0], 'RX 400 1', 'and the peer reads that')

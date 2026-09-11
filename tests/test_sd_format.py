import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class SdFormatTests(unittest.TestCase):
    def test_vectors_are_bounded_and_preserve_float_values(self):
        code = r'''#include "sd_vector_format.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
int main(void) {
    const float values[] = {FLT_MAX, -FLT_MAX, FLT_MIN, 0.1f, -0.0f};
    char text[384];
    assert(sd_format_vector(text, sizeof(text), values, 5) > 0);
    char *end = text;
    for (unsigned i=0; i<5; ++i) {
        float restored = strtof(end, &end);
        assert(memcmp(&restored, &values[i], sizeof(float)) == 0);
    }
    assert(*end == 0);
    struct { char output[4]; unsigned char guard[4]; } small = {{0}, {1,2,3,4}};
    assert(sd_format_vector(small.output, sizeof(small.output), values, 5) == -1);
    assert(small.output[0] == 0 && small.guard[0] == 1 && small.guard[3] == 4);
    assert(sd_format_vector(NULL, 0, values, 5) == -1);
    assert(sd_format_vector(text, sizeof(text), NULL, 5) == -1);
    float special[] = {NAN, INFINITY};
    assert(sd_format_vector(text, sizeof(text), special, 2) > 0);
    assert(strcmp(text, "nan inf") == 0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "format"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I",
                str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(binary)],
                input=code, text=True, check=True)
            subprocess.run([str(binary)], check=True)

    def test_filename_collisions_are_bounded_without_waiting_for_clock(self):
        source = (ROOT / "Core/Src/sdpipeline.c").read_text()
        self.assertIn("suffix < 1000U", source)
        self.assertNotIn("while (st == FX_ALREADY_CREATED)", source)
        self.assertNotIn("tx_thread_sleep(now_ms() % 61)", source)
        self.assertIn("g_sd_format_errors++", source)

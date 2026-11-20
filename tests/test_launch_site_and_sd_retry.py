"""The station-set launch site (LAUNCH_SITE) and the operator SD retry.

Runs the actual distribution.c handler for LAUNCH_SITE: applied only on the
pad and in range, refused after Launch or during a ground-test replay, and the
site in use is echoed either way. Checks that FcSdRetry (compat 33) exists in
every image, that a failed mount waits for it instead of spinning forever, and
that BOARD_HEALTH word 6 carries the SD state in bits 30..31.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def extract(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


STUB = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef enum { SEDS_OK } SedsResult;
enum { Startup, Postinit, Armed, Launch };
#define Launch_Requested 1U
#define option(x) (x)
#define load(p, o) (*(p))
#define within(expr, bound) (fabsf((float)(expr)) <= (bound))
#define SEDS_DT_LAUNCH_SITE 7
static unsigned g_conf, state, echoes;
static bool sim;
static float echoed[3];
float g_launch_site[3] = {31.758f, -102.046f, 890.016f};
static bool beyond(unsigned s) { return state > s; }
static bool igt_sim_running(void) { return sim; }
static void log_f32(int type, int n, const float *v) { assert(type == SEDS_DT_LAUNCH_SITE && n == 3); memcpy(echoed, v, sizeof echoed); ++echoes; }
'''

MAIN = r'''
static void send(float lat, float lon, float alt) { float s[3] = {lat, lon, alt}; assert(update_launch_site((const uint8_t *)s, sizeof s) == SEDS_OK); }
int main(void)
{
  send(43.003556f, -78.768556f, 182.0f);              /* on the pad: applied */
  assert(g_launch_site[0] == 43.003556f && g_launch_site[2] == 182.0f && echoes == 1 && echoed[1] == -78.768556f);
  send(95.0f, 0.0f, 0.0f);                             /* out of range: kept, echoed */
  send(0.0f, 200.0f, 0.0f);
  send(0.0f, 0.0f, 9500.0f);
  send(NAN, 0.0f, 0.0f);
  assert(g_launch_site[0] == 43.003556f && echoes == 5 && echoed[0] == 43.003556f);
  state = Launch; send(10.0f, 10.0f, 10.0f);           /* after Armed: refused */
  state = Armed; g_conf = Launch_Requested; send(10.0f, 10.0f, 10.0f);
  g_conf = 0; sim = true; send(10.0f, 10.0f, 10.0f);   /* during a replay: refused */
  assert(g_launch_site[0] == 43.003556f && echoes == 8);
  sim = false; send(10.0f, 10.0f, 10.0f);
  assert(g_launch_site[0] == 10.0f);
  assert(update_launch_site(NULL, 12) == SEDS_OK && echoes == 9);
  return 0;
}
'''


class LaunchSite(unittest.TestCase):
    def test_applied_on_the_pad_only_and_always_echoed(self):
        text = (ROOT / 'Core/Src/distribution.c').read_text()
        handler = extract(text, 'static inline SedsResult update_launch_site(')
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'site.c'
            c.write_text(STUB + handler + MAIN)
            exe = Path(tmp) / 'site'
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-fsanitize=address,undefined', str(c), '-o', str(exe), '-lm'], check=True)
            subprocess.run([str(exe)], check=True)

    def test_station_controller_only_and_dispatched(self):
        text = (ROOT / 'Core/Src/distribution.c').read_text()
        gate = text[text.index('static SedsResult on_fc_packet_impl'):]
        self.assertRegex(gate, r'pkt->ty == SEDS_DT_LAUNCH_SITE\) &&\s+!\(operator_pyro_command')
        self.assertIn('case SEDS_DT_LAUNCH_SITE:\n      return update_launch_site(', gate)
        # The configured site survives a watchdog resume like the GPS rail.
        self.assertIn('FC_RESUME_REGION(g_launch_site)', text)
        config = (ROOT / 'Core/Inc/fcconfig.h').read_text()
        self.assertIn('#define LAUNCH_SITE_LAT (g_launch_site[0])', config)
        self.assertIn('#define LAUNCH_SITE_LAT_DEFAULT 31.758f', config)


class SdRetry(unittest.TestCase):
    def test_retry_command_in_every_image(self):
        types = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        compat = types[types.index('typedef enum remote_cmd_compat'):types.index('} compat;')]
        names = re.findall(r'^\s+(\w+),', compat, re.M)
        self.assertEqual(names.index('Compat_SD_Retry'), 33)
        self.assertIn('SD_Retry         = Actionable_Decrees + 21', types)
        action = extract((ROOT / 'Core/Src/recovery.c').read_text(),
                         'static inline void process_action(fc_msg cmd, bool internal)')
        case = action[action.index('case SD_Retry:'):]
        before = action[:action.index('case SD_Retry:')]
        self.assertEqual(before.count('#if') - before.count('#endif'), 0)  # not inside any conditional
        self.assertIn('g_fc_sd_state |= FC_SD_RETRY_PENDING;', case.split('break;')[0])

    def test_failed_mount_waits_for_retry_and_health_reports_it(self):
        filex = (ROOT / 'FileX/App/app_filex.c').read_text()
        region = filex[filex.index('USER CODE BEGIN SD DRIVER get info error'):filex.index('USER CODE END SD DRIVER get info error')]
        self.assertNotIn('while(1);', region)
        self.assertIn('FC_SD_RETRY_PENDING', region)
        self.assertIn('fx_media_open(', region)
        health = (ROOT / 'Core/Src/board_health.c').read_text()
        self.assertIn('words[6] |= (uint32_t)g_fc_sd_state << 30U;', health)
        events = (ROOT / 'Core/Inc/fc_sd_events.h').read_text()
        self.assertIn('#define FC_SD_NOT_READY     1U', events)
        self.assertIn('#define FC_SD_RETRY_PENDING 2U', events)
        pipe = (ROOT / 'Core/Src/sdpipeline.c').read_text()
        self.assertIn('g_fc_sd_state = 0U;', pipe)


if __name__ == '__main__':
    unittest.main()

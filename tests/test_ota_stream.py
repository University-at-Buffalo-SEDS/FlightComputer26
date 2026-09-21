"""Exercise the production receiver: offsets, alignment, reconnect and reboot."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class OtaStreamTests(unittest.TestCase):
    def test_chunked_upload_and_reconnect(self):
        source = (ROOT / "Core/Src/ota_stream.c").read_text()
        source = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
        code = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#define BOARD_DELTA_SIZE 16384U
#define BOARD_FLASH_WRITE_ALIGNMENT 16U
#define OTA_STREAM_PORT 4510U
#define OTA_STREAM_MAX_CHUNK 112U
typedef int SedsResult;
typedef int SedsRouter;
typedef int launchcore_storage_driver_t;
typedef int launchcore_status_t;
typedef int ota_stream_status_t;
enum { SEDS_OK, SEDS_BAD_ARG, SEDS_ERR };
enum { LAUNCHCORE_OK, LAUNCHCORE_ERR_NO_SPACE, LAUNCHCORE_ERR_BAD_STATE,
       LAUNCHCORE_ERR_BAD_IMAGE, LAUNCHCORE_ERR_VERIFY, LAUNCHCORE_ERR_FLASH,
       LAUNCHCORE_ERR_METADATA };
enum { OTA_STREAM_OK, OTA_STREAM_BAD_MESSAGE, OTA_STREAM_BAD_STATE,
       OTA_STREAM_BAD_OFFSET, OTA_STREAM_NO_SPACE, OTA_STREAM_STORAGE_ERROR,
       OTA_STREAM_BAD_IMAGE, OTA_STREAM_INTERNAL_ERROR };
enum { SEDS_P2P_STREAM_ACCEPTED, SEDS_P2P_STREAM_DATA,
       SEDS_P2P_STREAM_CLOSED, SEDS_P2P_STREAM_RESET };
#define LAUNCHCORE_STORAGE_OK 0
typedef struct { int kind; uint32_t stream_id; const uint8_t *payload; size_t payload_len; } SedsP2pStreamEventView;
const launchcore_storage_driver_t launchcore_board_storage_driver=1;
static unsigned resets, aborted, written, confirmed;
static uint64_t now;
static uint8_t reply[13];
static void launchcore_storage_set_driver(const launchcore_storage_driver_t *d) { assert(d); }
static int launchcore_storage_init(void) { return 0; }
static uint64_t telemetry_now_ms(void) { return now; }
static void NVIC_SystemReset(void) { ++resets; }
static int launchcore_delta_update_begin(uint32_t n) { written=0; return n>16384 ? LAUNCHCORE_ERR_NO_SPACE : LAUNCHCORE_OK; }
static int launchcore_delta_update_write(const void *p,size_t n) { assert(p); written+=n; return 0; }
static int launchcore_delta_update_finish(void) { return 0; }
static int launchcore_delta_update_abort(void) { ++aborted; return 0; }
static int launchcore_confirm_boot(void) { ++confirmed; return 0; }
static int seds_router_send_p2p_stream(SedsRouter *r,uint32_t id,const void *p,size_t n) {
  assert(r && id && n==13); memcpy(reply,p,n); return 0;
}
static int seds_router_bind_p2p_stream_port(SedsRouter *r,unsigned port,
  SedsResult (*cb)(const SedsP2pStreamEventView*,void*),void *user) {
  assert(r && port==4510 && cb && user==NULL); return 0;
}
uint32_t ota_stream_max_patch_size(void);
''' + source + r'''
static unsigned send(uint32_t id,const uint8_t *p,size_t n) {
  SedsP2pStreamEventView e={SEDS_P2P_STREAM_DATA,id,p,n};
  assert(ota_stream_event(&e,NULL)==SEDS_OK);
  assert(reply[0]==(p[0]|0x80));
  assert(read_u32_le(reply+9)==16384);
  return read_u32_le(reply+1);
}
int main(void) {
  SedsRouter router=1;
  assert(ota_stream_init(&router)==SEDS_OK);
  SedsP2pStreamEventView e={SEDS_P2P_STREAM_ACCEPTED,7,NULL,0};
  assert(ota_stream_event(&e,NULL)==SEDS_OK);
  uint8_t begin[]={1,120,0,0,0}, chunk[117]={2}, finish[]={3};
  assert(send(7,begin,sizeof(begin))==OTA_STREAM_OK);
  chunk[1]=1;
  assert(send(7,chunk,sizeof(chunk))==OTA_STREAM_BAD_OFFSET);
  chunk[1]=0;
  assert(send(7,chunk,12)==OTA_STREAM_BAD_MESSAGE); /* Unaligned nonfinal chunk. */
  assert(send(7,chunk,sizeof(chunk))==OTA_STREAM_OK && written==112);
  e.kind=SEDS_P2P_STREAM_RESET; ota_stream_event(&e,NULL);
  assert(aborted==1 && resets==0);
  e.kind=SEDS_P2P_STREAM_ACCEPTED; e.stream_id=8; ota_stream_event(&e,NULL);
  assert(send(8,begin,sizeof(begin))==OTA_STREAM_OK);
  assert(send(8,chunk,sizeof(chunk))==OTA_STREAM_OK);
  chunk[1]=112;
  assert(send(8,chunk,13)==OTA_STREAM_OK && written==120); /* Final partial quadword. */
  assert(send(8,finish,1)==OTA_STREAM_OK);
  assert(send(8,begin,sizeof(begin))==OTA_STREAM_BAD_STATE);
  now=249; ota_stream_poll(); assert(resets==0);
  now=250; ota_stream_poll(); assert(resets==1);
  now=0; assert(ota_stream_init(&router)==SEDS_OK);
  now=5000; ota_stream_poll(); assert(confirmed==1);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "ota-stream"
            result = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                     "-x", "c", "-", "-o", str(exe)],
                                    input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()

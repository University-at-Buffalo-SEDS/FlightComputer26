from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class UnderglowIndicatorTests(unittest.TestCase):
    def test_latest_state_restored_without_blocking_and_across_tick_wrap(self):
        source=(ROOT/"Core/Src/av_bay_underglow.c").read_text()
        # Compile production implementation with GPIO, clock and network stubs.
        source="\n".join(line for line in source.splitlines() if not line.startswith('#include "'))
        stub=r"""
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
typedef int SedsResult;
typedef int SedsRouter;
typedef struct { int ty; const uint8_t *payload; size_t payload_len; } SedsPacketView;
#define SEDS_DT_AV_BAY_UNDERGLOW 7
#define SEDS_OK 0
#define SEDS_HANDLER_ERROR 1
#define SEDS_BAD_ARG 2
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
typedef int launchcore_persist_status_t;
#define LED2_PORT 0
#define LED2_PIN 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
volatile uint32_t g_telemetry_discovery_seen;
static uint32_t tick, mask;
static int pin, requests;
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __set_PRIMASK(uint32_t m) { mask=m; }
static void HAL_GPIO_WritePin(int port,int number,int value) { (void)port;(void)number; assert(mask); pin=value; }
static int persistent_store_init(void) { return 0; }
static int persistent_store_get(uint32_t k, void *v, size_t *n) { (void)k;(void)v;(void)n; return 1; }
static int persistent_store_set(uint32_t k,const void *v,size_t n) { (void)k;(void)v;(void)n; return 0; }
static int seds_router_enable_network_variable(SedsRouter *r,int ty,bool a,bool b) { (void)r;(void)ty;(void)a;(void)b;return 0; }
static int seds_router_on_network_variable_update(SedsRouter *r,int ty,SedsResult (*fn)(const SedsPacketView *,void *),void *u) { (void)r;(void)ty;(void)fn;(void)u;return 0; }
static int seds_router_request_managed_variable(SedsRouter *r,int ty) { (void)r;(void)ty;requests++;return 0; }
"""
        code=stub+source+r"""
static void update(uint8_t value) {
    SedsPacketView p={SEDS_DT_AV_BAY_UNDERGLOW,&value,1};
    assert(apply_underglow(&p,NULL)==SEDS_OK);
}
int main(void) {
    SedsRouter router=0; assert(av_bay_underglow_init(&router)==SEDS_OK);
    update(0); av_bay_underglow_signal(2); assert(pin==1 && tick==0);
    tick=100; av_bay_underglow_poll(&router); assert(pin==0);
    update(1); assert(pin==0); /* latest desired state, preserve current blink phase */
    tick=200; av_bay_underglow_poll(&router); assert(pin==1);
    tick=400; av_bay_underglow_poll(NULL); assert(pin==1 && g_indicator_edges==0);
    av_bay_underglow_signal(4); update(0);
    tick=1200; av_bay_underglow_poll(&router); assert(pin==0);
    tick=UINT32_MAX-49; av_bay_underglow_signal(2);
    tick=50; av_bay_underglow_poll(&router); assert(pin==0);
    update(1); tick=350; av_bay_underglow_poll(&router); assert(pin==1);
    av_bay_underglow_signal(4); tick+=50; av_bay_underglow_signal(2);
    tick+=400; av_bay_underglow_poll(&router); assert(pin==1);
    assert(mask==0 && requests==0); /* animation also finishes before discovery */
    mask=1; av_bay_underglow_reapply(); assert(mask==1);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(Path(tmp)/"indicator")
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-x","c","-","-o",exe],input=code,text=True,check=True)
            subprocess.run([exe],check=True)

"""Live FC checked ownership functions with controlled ThreadX primitives."""
from pathlib import Path
from router_lifecycle_fixture import function

def checked_hooks(root):
    source=(Path(root)/'Core/Src/fchooks.c').read_text()
    return r"""
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER (~0U)
#define TX_NULL NULL
#define TX_MUTEX_ID 0x4D555445U
typedef struct { unsigned tx_mutex_id; } TX_MUTEX;
static TX_MUTEX telemetry_mu={TX_MUTEX_ID};
static unsigned lock_fails,unlock_fails,fixture_isr,fixture_get_failure,fixture_put_failure;
static void *tx_thread_identify(void){return (void*)1;}
static unsigned __get_IPSR(void){return fixture_isr;}
static unsigned tx_mutex_get(TX_MUTEX *mu,unsigned timeout){assert(mu==&telemetry_mu&&timeout==TX_WAIT_FOREVER);if(fixture_get_failure)return 1;telemetry_lock();return TX_SUCCESS;}
static unsigned tx_mutex_put(TX_MUTEX *mu){assert(mu==&telemetry_mu);telemetry_unlock();return fixture_put_failure?1:TX_SUCCESS;}
"""+'\n'.join(function(source,name) for name in ('telemetry_router_context_ready','telemetry_router_enter','telemetry_router_leave'))

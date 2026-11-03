
#define TX_NULL NULL
static struct task telemetry_task;
static struct task *current_thread=&telemetry_task;
static struct {struct task *tx_mutex_owner;unsigned tx_mutex_ownership_count;} telemetry_mu;
static struct task *tx_thread_identify(void){return current_thread;}
static int telemetry_router_enter(void){assert(!irq);telemetry_mu.tx_mutex_owner=current_thread;++telemetry_mu.tx_mutex_ownership_count;return 1;}
static int telemetry_router_leave(void){assert(!irq&&telemetry_mu.tx_mutex_ownership_count);if(!--telemetry_mu.tx_mutex_ownership_count)telemetry_mu.tx_mutex_owner=NULL;return 1;}
static int telemetry_poll_discovery_locked(void){assert(telemetry_mu.tx_mutex_owner==current_thread);ota_stream_poll();return SEDS_OK;}

#include "av_bay_underglow.h"
#include "platform.h"

#include "persistent_store.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define UNDERGLOW_PERSIST_KEY 0x55474C57u
#define NETWORK_VARIABLE_UNSYNCED_RETRY_MS 500U
#define NETWORK_VARIABLE_REFRESH_MS 2000U

volatile uint32_t g_av_bay_underglow_enabled = 0U;
volatile uint32_t g_av_bay_underglow_updates = 0U;
volatile uint32_t g_av_bay_underglow_persist_restores = 0U;
volatile uint32_t g_av_bay_underglow_persist_writes = 0U;
volatile uint32_t g_av_bay_underglow_persist_errors = 0U;
volatile uint32_t g_av_bay_underglow_boot_restore_valid = 0U;
volatile uint32_t g_av_bay_underglow_boot_restored_value = 0U;

static bool g_persist_ready = false;
static bool g_persist_has_value = false;
static bool g_restore_attempted = false;
static bool g_network_value_seen = false;
static uint32_t g_last_refresh_ms = 0U;

/* The recovery task must never busy-wait for an indicator. Only these short
 * GPIO/state updates mask interrupts; packet processing continues between edges. */
#define INDICATOR_HALF_PERIOD_MS 100U
static uint32_t g_indicator_edges;
static uint32_t g_indicator_deadline;

static void reapply_locked(void)
{
    const bool enabled = g_indicator_edges != 0U
        ? (g_indicator_edges & 1U) == 0U
        : g_av_bay_underglow_enabled != 0U;
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void drive_underglow(bool enabled)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    g_av_bay_underglow_enabled = enabled ? 1U : 0U;
    reapply_locked();
    __set_PRIMASK(mask);
}

void av_bay_underglow_signal(uint32_t pulses)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    g_indicator_edges = (pulses > 4U ? 4U : pulses) * 2U;
    g_indicator_deadline = HAL_GetTick() + INDICATOR_HALF_PERIOD_MS;
    reapply_locked();
    __set_PRIMASK(mask);
}

static void poll_indicator(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = HAL_GetTick();
    if (g_indicator_edges != 0U && (int32_t)(now - g_indicator_deadline) >= 0) {
        const uint32_t elapsed_edges =
            (now - g_indicator_deadline) / INDICATOR_HALF_PERIOD_MS + 1U;
        g_indicator_edges = elapsed_edges >= g_indicator_edges
            ? 0U : g_indicator_edges - elapsed_edges;
        g_indicator_deadline = now + INDICATOR_HALF_PERIOD_MS -
            (now - g_indicator_deadline) % INDICATOR_HALF_PERIOD_MS;
        reapply_locked();
    }
    __set_PRIMASK(mask);
}

void av_bay_underglow_restore(void)
{
    uint8_t enabled = 0U;
    size_t enabled_size = sizeof(enabled);

    if (g_restore_attempted) return;
    g_restore_attempted = true;

    if (persistent_store_init() != LAUNCHCORE_PERSIST_OK)
    {
        g_av_bay_underglow_persist_errors++;
        return;
    }
    g_persist_ready = true;

    const launchcore_persist_status_t status = persistent_store_get(
        UNDERGLOW_PERSIST_KEY, &enabled, &enabled_size);
    if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
    if (status != LAUNCHCORE_PERSIST_OK || enabled_size != sizeof(enabled) ||
        enabled > 1U)
    {
        g_av_bay_underglow_persist_errors++;
        return;
    }

    g_persist_has_value = true;
    g_av_bay_underglow_persist_restores++;
    g_av_bay_underglow_boot_restore_valid = 1U;
    g_av_bay_underglow_boot_restored_value = enabled;
    drive_underglow(enabled != 0U);
}

void av_bay_underglow_reapply(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    reapply_locked();
    __set_PRIMASK(mask);
}

static SedsResult apply_underglow(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_AV_BAY_UNDERGLOW ||
        packet->payload == NULL || packet->payload_len != 1U)
    {
        return SEDS_HANDLER_ERROR;
    }

    const bool enabled = packet->payload[0] != 0U;
    const bool needs_persist = !g_persist_has_value ||
                               g_av_bay_underglow_enabled != (uint32_t)enabled;
    drive_underglow(enabled);
    g_network_value_seen = true;
    g_last_refresh_ms = HAL_GetTick();
    g_av_bay_underglow_updates++;

    if (needs_persist)
    {
        const uint8_t stored = enabled ? 1U : 0U;
        if (!g_persist_ready ||
            persistent_store_set(UNDERGLOW_PERSIST_KEY, &stored,
                                 sizeof(stored)) != LAUNCHCORE_PERSIST_OK)
        {
            g_av_bay_underglow_persist_errors++;
            return SEDS_HANDLER_ERROR;
        }
        g_persist_has_value = true;
        g_av_bay_underglow_persist_writes++;
    }
    return SEDS_OK;
}

SedsResult av_bay_underglow_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    av_bay_underglow_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW, true, false);
    if (result != SEDS_OK) return result;
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_AV_BAY_UNDERGLOW, apply_underglow, NULL);
    if (result != SEDS_OK) return result;
    g_last_refresh_ms = HAL_GetTick();
    return SEDS_OK;
}

SedsResult av_bay_underglow_poll(SedsRouter *router)
{
    poll_indicator();
    if (router == NULL) return SEDS_BAD_ARG;
    if (g_telemetry_discovery_seen == 0U) return SEDS_OK;
    const uint32_t now_ms = HAL_GetTick();
    /* A successfully received value does not guarantee delivery of the next
     * broadcast. Refresh the read-only replica after an idle interval so a
     * dropped update or a restarted relay cannot leave the LED stale forever.
     * Received updates reset this timer; never flood requests while toggling. */
    const uint32_t interval = g_network_value_seen
        ? NETWORK_VARIABLE_REFRESH_MS : NETWORK_VARIABLE_UNSYNCED_RETRY_MS;
    if ((uint32_t)(now_ms - g_last_refresh_ms) < interval) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    return seds_router_request_managed_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW);
}

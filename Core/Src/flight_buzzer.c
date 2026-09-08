#include "flight_buzzer.h"
#include "main.h"

#include "persistent_store.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FLIGHT_BUZZER_PERSIST_KEY 0x42555A5Au
#define FLIGHT_BUZZER_PERSIST_RECORD_SIZE 9U
#define NETWORK_VARIABLE_UNSYNCED_RETRY_MS 500U

volatile uint32_t g_flight_buzzer_enabled = 0U;
volatile uint32_t g_flight_buzzer_updates = 0U;
volatile uint32_t g_flight_buzzer_persist_restores = 0U;
volatile uint32_t g_flight_buzzer_persist_writes = 0U;
volatile uint32_t g_flight_buzzer_persist_errors = 0U;
volatile uint32_t g_flight_buzzer_stale_updates = 0U;
volatile uint32_t g_flight_buzzer_boot_restore_valid = 0U;
volatile uint32_t g_flight_buzzer_boot_restored_value = 0U;

static bool g_persist_ready = false;
static bool g_persist_has_value = false;
static bool g_persist_has_timestamp = false;
static bool g_restore_attempted = false;
static bool g_network_value_seen = false;
static uint32_t g_last_refresh_ms = 0U;
static uint64_t g_last_source_timestamp_ms = 0U;

static uint64_t decode_u64_le(const uint8_t *bytes)
{
    uint64_t value = 0U;
    for (size_t index = 0U; index < sizeof(value); index++)
    {
        value |= ((uint64_t)bytes[index]) << (index * 8U);
    }
    return value;
}

static void encode_u64_le(uint8_t *bytes, uint64_t value)
{
    for (size_t index = 0U; index < sizeof(value); index++)
    {
        bytes[index] = (uint8_t)(value >> (index * 8U));
    }
}

static void drive_buzzer(bool enabled)
{
    g_flight_buzzer_enabled = enabled ? 1U : 0U;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void flight_buzzer_restore(void)
{
    if (g_restore_attempted) return;
    g_restore_attempted = true;

    uint8_t record[FLIGHT_BUZZER_PERSIST_RECORD_SIZE] = {0U};
    size_t record_size = sizeof(record);

    if (persistent_store_init() != LAUNCHCORE_PERSIST_OK)
    {
        g_flight_buzzer_persist_errors++;
        return;
    }
    g_persist_ready = true;

    const launchcore_persist_status_t status = persistent_store_get(
        FLIGHT_BUZZER_PERSIST_KEY, record, &record_size);
    if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
    if (status != LAUNCHCORE_PERSIST_OK ||
        (record_size != 1U && record_size != sizeof(record)) || record[0] > 1U)
    {
        g_flight_buzzer_persist_errors++;
        return;
    }
    g_persist_has_value = true;
    if (record_size == sizeof(record))
    {
        g_last_source_timestamp_ms = decode_u64_le(&record[1]);
        g_persist_has_timestamp = true;
    }
    g_flight_buzzer_persist_restores++;
    g_flight_buzzer_boot_restore_valid = 1U;
    g_flight_buzzer_boot_restored_value = record[0];
    drive_buzzer(record[0] != 0U);
}

static SedsResult apply_buzzer(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_FLIGHT_BUZZER ||
        packet->payload == NULL || packet->payload_len != 1U)
    {
        return SEDS_HANDLER_ERROR;
    }

    const bool enabled = packet->payload[0] != 0U;
    if (g_persist_has_timestamp &&
        (packet->timestamp < g_last_source_timestamp_ms ||
         (packet->timestamp == g_last_source_timestamp_ms &&
          g_flight_buzzer_enabled != (uint32_t)enabled)))
    {
        g_flight_buzzer_stale_updates++;
        return SEDS_OK;
    }
    const bool needs_persist = !g_persist_has_value ||
                               !g_persist_has_timestamp ||
                               g_flight_buzzer_enabled != (uint32_t)enabled ||
                               packet->timestamp > g_last_source_timestamp_ms;
    drive_buzzer(enabled);
    g_network_value_seen = true;
    g_flight_buzzer_updates++;
    if (needs_persist)
    {
        uint8_t record[FLIGHT_BUZZER_PERSIST_RECORD_SIZE] = {
            enabled ? 1U : 0U,
        };
        encode_u64_le(&record[1], packet->timestamp);
        if (!g_persist_ready ||
            persistent_store_set(FLIGHT_BUZZER_PERSIST_KEY, record,
                                 sizeof(record)) != LAUNCHCORE_PERSIST_OK)
        {
            g_flight_buzzer_persist_errors++;
            return SEDS_HANDLER_ERROR;
        }
        g_persist_has_value = true;
        g_persist_has_timestamp = true;
        g_last_source_timestamp_ms = packet->timestamp;
        g_flight_buzzer_persist_writes++;
    }
    return SEDS_OK;
}

SedsResult flight_buzzer_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    flight_buzzer_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_FLIGHT_BUZZER, true, false);
    if (result != SEDS_OK) return result;
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_FLIGHT_BUZZER, apply_buzzer, NULL);
    if (result != SEDS_OK) return result;
    g_last_refresh_ms = HAL_GetTick();
    result = seds_router_request_managed_variable(
        router, SEDS_DT_FLIGHT_BUZZER);
    return result == SEDS_IO ? SEDS_OK : result;
}

SedsResult flight_buzzer_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    if (g_network_value_seen) return SEDS_OK;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_refresh_ms) <
        NETWORK_VARIABLE_UNSYNCED_RETRY_MS) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    return seds_router_request_managed_variable(
        router, SEDS_DT_FLIGHT_BUZZER);
}

#include "platform.h"
#include "fc_watchdog_recovery.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fcconfig.h"
#include "fc_sd_events.h"

extern atomic_uint_fast32_t g_conf;

/* SysTick remains the independent runtime clock when TIM6 stops delivering
 * interrupts. No scheduler, telemetry, allocation or logging calls here. */
_Static_assert(TX_TIMER_TICKS_PER_SECOND == 1000, "pulse timing requires 1 kHz SysTick");
volatile uint32_t g_fc_runtime_clock_active;
volatile uint32_t g_deployment_pulse_completions;
static volatile uint32_t co2_ticks;
static volatile uint32_t reef_ticks;

/* The only interrupt work added is a timestamp and fixed-size ring copy.
 * Single-core IRQ exclusion serializes task/ISR producers; no heap, locks,
 * filesystem, networking or scheduler calls occur in this capture path. */
#define FC_SD_EVENT_CAPACITY 32U
static fc_sd_event sd_events[FC_SD_EVENT_CAPACITY];
static uint8_t sd_event_head, sd_event_tail;
volatile uint32_t g_sd_event_pending;
volatile uint32_t g_sd_event_queue_drops;
volatile uint32_t g_sd_event_queue_peak;

void fc_sd_capture_event(fc_sd_event_kind kind, bool reef)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (g_sd_event_pending == FC_SD_EVENT_CAPACITY) ++g_sd_event_queue_drops;
    else {
        sd_events[sd_event_head] = (fc_sd_event){HAL_GetTick(), kind, reef};
        sd_event_head = (sd_event_head + 1U) % FC_SD_EVENT_CAPACITY;
        if (++g_sd_event_pending > g_sd_event_queue_peak)
            g_sd_event_queue_peak = g_sd_event_pending;
    }
    __set_PRIMASK(mask);
}

bool fc_sd_peek_event(fc_sd_event *event)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = g_sd_event_pending != 0U;
    if (available) *event = sd_events[sd_event_tail];
    __set_PRIMASK(mask);
    return available;
}

void fc_sd_consume_event(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (g_sd_event_pending) {
        --g_sd_event_pending;
        sd_event_tail = (sd_event_tail + 1U) % FC_SD_EVENT_CAPACITY;
    }
    __set_PRIMASK(mask);
}

void deployment_pulse_start(bool reef)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    volatile uint32_t *remaining = reef ? &reef_ticks : &co2_ticks;
    /* Repeated commands during a pulse must not extend its on-time. */
    bool started = false;
    if (*remaining == 0U && fc_watchdog_can_actuate() &&
        fc_watchdog_record_deployment(reef)) {
        started = true;
        *remaining = reef ? REEF_ASSERT_INTERVAL : CO2_ASSERT_INTERVAL;
        if (reef) {
            g_conf |= option(Parachute_Expanded | REEF_Asserted);
            reef_high();
        } else {
            g_conf |= option(Parachute_Deployed | CO2_Asserted);
            co2_high();
        }
    }
    fc_sd_capture_event(started ? FC_LOG_PYRO_HIGH :
                        (*remaining ? FC_LOG_PYRO_ACTIVE : FC_LOG_PYRO_REJECTED), reef);
    __set_PRIMASK(mask);
}

void fc_runtime_tick(void)
{
    if (g_fc_runtime_clock_active == 0U) {
        /* TIM6 supplies the HAL boot clock. Once ThreadX starts, transfer its
         * ownership to SysTick so runtime timeouts share the scheduler clock. */
        HAL_SuspendTick();
        g_fc_runtime_clock_active = 1U;
    }
    HAL_IncTick();
    if (co2_ticks != 0U && --co2_ticks == 0U) {
        co2_low();
        g_conf &= ~option(CO2_Asserted);
        g_deployment_pulse_completions++;
        fc_sd_capture_event(FC_LOG_PYRO_LOW, false);
    }
    if (reef_ticks != 0U && --reef_ticks == 0U) {
        reef_low();
        g_conf &= ~option(REEF_Asserted);
        g_deployment_pulse_completions++;
        fc_sd_capture_event(FC_LOG_PYRO_LOW, true);
    }
}

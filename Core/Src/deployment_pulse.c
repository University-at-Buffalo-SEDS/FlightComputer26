#include "platform.h"
#include "fc_watchdog_recovery.h"
#include "fcstructs.h"
#include "fccommon.h"
#include "fcconfig.h"

extern atomic_uint_fast32_t g_conf;

/* SysTick remains the independent runtime clock when TIM6 stops delivering
 * interrupts. No scheduler, telemetry, allocation or logging calls here. */
_Static_assert(TX_TIMER_TICKS_PER_SECOND == 1000, "pulse timing requires 1 kHz SysTick");
volatile uint32_t g_fc_runtime_clock_active;
volatile uint32_t g_deployment_pulse_completions;
static volatile uint32_t co2_ticks;
static volatile uint32_t reef_ticks;

void deployment_pulse_start(bool reef)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    volatile uint32_t *remaining = reef ? &reef_ticks : &co2_ticks;
    /* Repeated commands during a pulse must not extend its on-time. */
    if (*remaining == 0U && fc_watchdog_can_actuate() &&
        fc_watchdog_record_deployment(reef)) {
        *remaining = reef ? REEF_ASSERT_INTERVAL : CO2_ASSERT_INTERVAL;
        if (reef) {
            g_conf |= option(Parachute_Expanded | REEF_Asserted);
            reef_high();
        } else {
            g_conf |= option(Parachute_Deployed | CO2_Asserted);
            co2_high();
        }
    }
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
    }
    if (reef_ticks != 0U && --reef_ticks == 0U) {
        reef_low();
        g_conf &= ~option(REEF_Asserted);
        g_deployment_pulse_completions++;
    }
}

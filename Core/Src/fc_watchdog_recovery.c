/* Board-owned retained flight checkpoint; SEDSnet remains generic. */
#include "platform.h"
#include "fcapi.h"
#include "board_watchdog.h"
#include "fc_watchdog_recovery.h"
#include <string.h>
#if BOARD_WATCHDOG_ENABLE
#define RECORD_SEAL 0x46575231U
#define PAYLOAD_CAPACITY 2048U
#define ACTION_SEAL 0x46414331U
#define RECOVERY_MAX_GAP_MS 60000U

typedef struct {
    uint32_t seal, sequence, image, size, time_ms, rtc_tick, crc;
    uint8_t payload[PAYLOAD_CAPACITY];
} flight_record;
typedef struct { uint32_t seal, sequence, image, actions, crc; } action_record;
static struct {
    flight_record flight[2];
    action_record action[2];
} retained __attribute__((section(".fc_watchdog_retained"), aligned(8)));
/* Fingerprint the full loaded application, including initial data. Incremental
 * builds must also invalidate a checkpoint when another source file changes. */
extern const uint8_t g_pfnVectors[], __data_source_end[];
extern const fc_resume_region fc_resume_evaluation[], fc_resume_kalman[];
extern const fc_resume_region fc_resume_recovery[], fc_resume_barometer[];
#ifdef GPS_AVAILABLE
extern const fc_resume_region fc_resume_distribution[];
#endif
static const fc_resume_region *const regions[] = {
    fc_resume_evaluation, fc_resume_kalman, fc_resume_recovery, fc_resume_barometer,
#ifdef GPS_AVAILABLE
    fc_resume_distribution,
#endif
};
volatile uint32_t g_fc_watchdog_resumed;
volatile uint32_t g_fc_watchdog_resume_rejected;
volatile uint32_t g_fc_watchdog_checkpoints;
volatile uint32_t g_fc_watchdog_resume_gap_ms;
static uint32_t image_id, actions, next_slot, sequence, action_slot, action_sequence;
static bool busy, clock_ready;

static uint32_t checksum(uint32_t crc, const void *buffer, uint32_t length)
{
    const uint8_t *p = buffer;
    while (length--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return crc;
}
static uint32_t flight_crc(const flight_record *r)
{
    return checksum(checksum(~0U, &r->sequence, 5U * sizeof(uint32_t)), r->payload, r->size);
}
static uint32_t action_crc(const action_record *r)
{
    return checksum(~0U, &r->sequence, 3U * sizeof(uint32_t));
}
static uint32_t payload_transfer(uint8_t *buffer, bool restore)
{
    uint32_t offset = 0U;
    for (unsigned group = 0; group < sizeof(regions) / sizeof(regions[0]); ++group) {
        for (const fc_resume_region *r = regions[group]; r->size; ++r) {
            if (r->size > PAYLOAD_CAPACITY - offset) return 0U;
            if (buffer) {
                if (restore) memcpy(r->data, buffer + offset, r->size);
                else memcpy(buffer + offset, r->data, r->size);
            }
            offset += r->size;
        }
    }
    return offset;
}
/* RTC binary down-counter survives IWDG reset. LSI / 32 is nominally 1 kHz;
 * its tolerance must be included in flight timing qualification. No UTC use. */
static bool resume_clock_init(void)
{
    PWR->DBPCR |= PWR_DBPCR_DBP;
    RCC->APB3ENR |= RCC_APB3ENR_RTCAPBEN;
    RCC->BDCR |= RCC_BDCR_LSION;
    uint32_t spins = 1000000U;
    while (!(RCC->BDCR & RCC_BDCR_LSIRDY) && --spins) { __NOP(); }
    if (!spins || !(PWR->DBPCR & PWR_DBPCR_DBP)) return false;
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_1) {
        /* Never reset someone else's backup domain to acquire the clock. */
        if (RCC->BDCR & RCC_BDCR_RTCSEL) return false;
        RCC->BDCR |= RCC_BDCR_RTCSEL_1;
    }
    RCC->BDCR |= RCC_BDCR_RTCEN;
    if ((RTC->ICSR & RTC_ICSR_BIN) != RTC_ICSR_BIN_0 ||
        RTC->PRER != (31U << RTC_PRER_PREDIV_A_Pos)) {
        RTC->WPR = 0xCAU; RTC->WPR = 0x53U;
        RTC->ICSR |= RTC_ICSR_INIT;
        spins = 1000000U;
        while (!(RTC->ICSR & RTC_ICSR_INITF) && --spins) { __NOP(); }
        if (!spins) { RTC->WPR = 0xFFU; return false; }
        RTC->ICSR = (RTC->ICSR & ~RTC_ICSR_BIN) | RTC_ICSR_BIN_0;
        RTC->PRER = 31U << RTC_PRER_PREDIV_A_Pos;
        RTC->CR |= RTC_CR_BYPSHAD;
        RTC->ICSR &= ~RTC_ICSR_INIT;
        RTC->WPR = 0xFFU;
    }
    const uint32_t initial = RTC->SSR;
    spins = 1000000U;
    while (RTC->SSR == initial && --spins) { __NOP(); }
    return spins != 0U;
}
static bool flight_valid(const flight_record *r, uint32_t size)
{
    return r->seal == RECORD_SEAL && r->image == image_id && r->size == size &&
           r->size <= PAYLOAD_CAPACITY && r->crc == flight_crc(r);
}
static bool action_valid(const action_record *r)
{
    return r->seal == ACTION_SEAL && r->image == image_id && r->actions <= 3U &&
           r->crc == action_crc(r);
}
static bool newer(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
static bool finite_float(float value)
{
    uint32_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}
static bool live_state_valid(void)
{
    const uint32_t conf = g_conf;
    if (!(conf & option(Launch_Requested)) || current() < Armed || current() > Recovery ||
        sm.idx >= STATE_HISTORY || sm.global_state >= Global_States) return false;
    /* Validate float-only groups without relying on -ffast-math isfinite. */
    for (unsigned group = 0; group < sizeof(regions) / sizeof(regions[0]); ++group) {
        if (group == 2U) continue; /* config/timer integers */
        for (const fc_resume_region *r = regions[group]; r->size; ++r) {
            if (r->data == (void *)&sm) continue;
            for (uint32_t offset = 0U; offset < r->size; offset += sizeof(float)) {
                float value; memcpy(&value, (uint8_t *)r->data + offset, sizeof(value));
                if (!finite_float(value)) return false;
            }
        }
    }
    float pressure; memcpy(&pressure, fc_resume_barometer[0].data, sizeof(pressure));
    return pressure > 0.0f;
}
bool fc_watchdog_can_actuate(void) { return g_fc_watchdog_resume_rejected == 0U; }
static void save_actions(void)
{
    action_record *r = &retained.action[action_slot];
    r->seal = 0U; __DMB();
    r->sequence = ++action_sequence; r->image = image_id; r->actions = actions;
    r->crc = action_crc(r); __DMB(); r->seal = ACTION_SEAL; __DMB();
    action_slot ^= 1U;
}
bool fc_watchdog_record_deployment(bool reef)
{
    if (!fc_watchdog_can_actuate()) return false;
    actions |= reef ? 2U : 1U;
    save_actions(); /* caller holds IRQ mask; commit precedes GPIO assertion */
    return true;
}
void fc_watchdog_recovery_boot(uint32_t flags)
{
    image_id = checksum(~0U, g_pfnVectors, (uint32_t)((uintptr_t)__data_source_end - (uintptr_t)g_pfnVectors));
    clock_ready = resume_clock_init();
    const uint32_t size = payload_transfer(NULL, false);
    const bool watchdog_reset = (flags & RCC_RSR_IWDGRSTF) && !(flags & RCC_RSR_BORRSTF);
    if (!watchdog_reset) {
        memset(&retained, 0, sizeof(retained));
        if (!clock_ready || !size) g_fc_watchdog_resume_rejected = 1U;
        save_actions();
        return;
    }
    unsigned chosen = 0U, journal = 0U;
    const bool valid0 = flight_valid(&retained.flight[0], size);
    const bool valid1 = flight_valid(&retained.flight[1], size);
    const bool action0 = action_valid(&retained.action[0]);
    const bool action1 = action_valid(&retained.action[1]);
    if (!clock_ready || !size || (!valid0 && !valid1) || (!action0 && !action1) ||
        (retained.action[0].seal == ACTION_SEAL && !action0) ||
        (retained.action[1].seal == ACTION_SEAL && !action1)) goto reject;
    if (valid1 && (!valid0 || newer(retained.flight[1].sequence, retained.flight[0].sequence))) chosen = 1U;
    if (action1 && (!action0 || newer(retained.action[1].sequence, retained.action[0].sequence))) journal = 1U;
    flight_record *r = &retained.flight[chosen];
    const uint32_t gap = r->rtc_tick - RTC->SSR;
    if (gap > RECOVERY_MAX_GAP_MS) goto reject;
    /* Keep a pristine boot image for rejection after semantic validation. */
    uint8_t *defaults = retained.flight[chosen ^ 1U].payload;
    retained.flight[chosen ^ 1U].seal = 0U;
    payload_transfer(defaults, false);
    payload_transfer(r->payload, true);
    if (!live_state_valid()) { payload_transfer(defaults, true); goto reject; }
    actions = retained.action[journal].actions;
    action_sequence = retained.action[journal].sequence; action_slot = journal ^ 1U;
    g_conf &= ~option(CO2_Asserted | REEF_Asserted | Ascent_KF_Staged | GPS_Available);
    sm.confidence = 0; /* pre-reset samples cannot establish a new transition */
    if (actions & 1U) g_conf |= option(Parachute_Deployed);
    if (actions & 2U) g_conf |= option(Parachute_Expanded);
    /* An actuation can precede the next completed estimator checkpoint. */
    if ((actions & 1U) && current() < Descent) sm.flight = Descent;
    if ((actions & 2U) && current() < Reefing) sm.flight = Reefing;
    const uint32_t now = now_ms();
    for (unsigned i = 0U; i < Time_Users; ++i)
        local_time[i] = now - (r->time_ms - local_time[i] + gap);
    kalman_resume_bindings((g_conf & option(Using_Ascent_KF)) != 0U);
    sequence = r->sequence; next_slot = chosen ^ 1U;
    g_fc_watchdog_resume_gap_ms = gap;
    g_fc_watchdog_resumed = 1U;
    return;
reject:
    g_fc_watchdog_resume_rejected = 1U;
}
void fc_watchdog_checkpoint(void)
{
    if (!clock_ready || !fc_watchdog_can_actuate() || !(g_conf & option(Launch_Requested))) return;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (busy) { __set_PRIMASK(mask); return; }
    busy = true;
    flight_record *r = &retained.flight[next_slot];
    r->seal = 0U; __DMB();
    r->sequence = ++sequence; r->image = image_id;
    r->time_ms = now_ms(); r->rtc_tick = RTC->SSR;
    r->size = payload_transfer(r->payload, false);
    __set_PRIMASK(mask);
    r->crc = flight_crc(r);
    __DMB(); r->seal = RECORD_SEAL; __DMB();
    next_slot ^= 1U; busy = false;
    g_fc_watchdog_checkpoints++;
}
#endif

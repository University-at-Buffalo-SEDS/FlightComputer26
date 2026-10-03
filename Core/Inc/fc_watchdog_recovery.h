#ifndef FC_WATCHDOG_RECOVERY_H
#define FC_WATCHDOG_RECOVERY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifndef BOARD_WATCHDOG_ENABLE
#define BOARD_WATCHDOG_ENABLE 0
#endif
typedef struct { void *data; uint32_t size; } fc_resume_region;
#define FC_RESUME_REGION(x) { (void *)&(x), sizeof(x) }
#if BOARD_WATCHDOG_ENABLE
extern volatile uint32_t g_fc_watchdog_resumed;
extern volatile uint32_t g_fc_watchdog_resume_rejected;
extern volatile uint32_t g_fc_watchdog_checkpoints;
void fc_watchdog_recovery_boot(uint32_t reset_flags);
void fc_watchdog_checkpoint(void);
bool fc_watchdog_record_deployment(bool reef);
bool fc_watchdog_can_actuate(void);
void kalman_resume_bindings(bool ascent);
#else
#define g_fc_watchdog_resumed 0U
#define g_fc_watchdog_resume_rejected 0U
static inline void fc_watchdog_recovery_boot(uint32_t f) { (void)f; }
static inline void fc_watchdog_checkpoint(void) {}
static inline bool fc_watchdog_record_deployment(bool r) { (void)r; return true; }
static inline bool fc_watchdog_can_actuate(void) { return true; }
#endif
#endif

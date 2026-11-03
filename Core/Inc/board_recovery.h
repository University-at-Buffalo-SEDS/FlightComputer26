#ifndef BOARD_RECOVERY_H
#define BOARD_RECOVERY_H
#include <stdint.h>
/* Application protocol only. Wire encoding is explicit LE, never memcpy of this struct. */
enum {
 BOARD_RECOVERY_ABORT_LATCHED=1u<<0,
 BOARD_RECOVERY_SAFE_OUTPUTS=1u<<1,
 BOARD_RECOVERY_SUPPORTED=1u<<2,
 BOARD_RECOVERY_STATE_VALID=1u<<3,
 BOARD_RECOVERY_OUTPUTS_VALID=1u<<4,
 /* Hardened application transport; independent of readiness or output proof. */
 BOARD_RECOVERY_VERIFIED_IMPLEMENTATION=1u<<5,
 /* Exact known-UTC manual Stop retirement, independent of Ready/Applied. */
 BOARD_RECOVERY_MANUAL_UTC_IMPLEMENTATION=1u<<6
};
typedef struct {
 uint8_t actual_state;
 uint8_t flags;
 uint16_t reserved;
 uint32_t abort_epoch;
 uint32_t actual_outputs_mask;
} BoardRecoverySnapshot;
/* SEDS_OK (0) or negative SedsResult. Snapshot must be a serialized actual readback. */
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out);
/* FC concrete owner: Startup(0) only, asynchronously quiescent evaluator and
 * LOW-only pulse cancellation, then telemetry-owned verified persistent cache
 * commit. Return PENDING until actual owner finalization and epoch advance.
 * Idle/resync is unsupported; GPIO readback is not physical deployment proof. */
int32_t board_recovery_apply_safe(uint8_t desired_state,
                                uint32_t expected_abort_epoch,
                                BoardRecoverySnapshot *out);

#endif

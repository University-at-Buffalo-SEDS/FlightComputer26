/* FC uses C and Rust panic=abort; it cannot unwind through ThreadX tasks.
 * Rust's allocation shim still emits an ARM EHABI personality reference.
 * Return the ABI failure result if unwinding is ever attempted, rather than
 * linking the full libgcc unwinder into this non-unwinding firmware.
 */
#include <unwind.h>

_Unwind_Reason_Code __aeabi_unwind_cpp_pr0(
    _Unwind_State state, _Unwind_Control_Block *exception,
    _Unwind_Context *context)
{
    (void)state;
    (void)exception;
    (void)context;
    return _URC_FAILURE;
}

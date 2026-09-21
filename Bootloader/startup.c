#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern void SystemInit(void);
extern int main(void);

static void halt(void) { for (;;) {} }
void Reset_Handler(void)
{
    /* This polling-only bootloader never uses external interrupts. Leave them
     * masked until LaunchCore's application handoff installs the full vectors. */
    __asm volatile ("cpsid i" ::: "memory");
    SystemInit();
    const uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
    for (uint32_t *dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    (void)main();
    halt();
}

/* The application retains the complete CubeMX vector table. Only the
 * bootloader uses this core-exception table (no SysTick or NVIC IRQs). */
__attribute__((section(".isr_vector"), used, aligned(128)))
const uintptr_t boot_vectors[16] = {
    (uintptr_t)&_estack, (uintptr_t)Reset_Handler,
    (uintptr_t)halt, (uintptr_t)halt, (uintptr_t)halt, (uintptr_t)halt,
    (uintptr_t)halt, (uintptr_t)halt, 0, 0, 0, (uintptr_t)halt,
    (uintptr_t)halt, 0, (uintptr_t)halt, (uintptr_t)halt
};

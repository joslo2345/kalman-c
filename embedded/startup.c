/* Cortex-M4F startup for the benchmark firmware: vector table, .data/.bss
 * init, FPU enable, and stack painting for the high-water measurement. */

#include <stdint.h>

#include "semihost.h"

#define STACK_PAINT 0xC5C5C5C5u

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);

static void default_handler(void) {
    sh_puts("fw: unexpected exception\n");
    sh_exit(1);
}

void Reset_Handler(void);

__attribute__((section(".isr_vector"), used)) static void (*const vectors[16])(void) = {
    (void (*)(void))&_estack, /* initial stack pointer */
    Reset_Handler,
    default_handler, /* NMI */
    default_handler, /* HardFault */
    default_handler, /* MemManage */
    default_handler, /* BusFault */
    default_handler, /* UsageFault */
};

/* Words below the stack pointer still holding STACK_PAINT were never used. */
uint32_t fw_stack_used_bytes(void) {
    const uint32_t *p = &_ebss;
    while (p < &_estack && *p == STACK_PAINT)
        ++p;
    return (uint32_t)((const char *)&_estack - (const char *)p);
}

__attribute__((naked, noreturn)) void Reset_Handler(void) {
    /* Paint the stack region before anything uses it (r0..r2 only, no stack). */
    __asm__ volatile("ldr r0, =_ebss\n"
                     "ldr r1, =_estack\n"
                     "ldr r2, =0xC5C5C5C5\n"
                     "1: cmp r0, r1\n"
                     "   bhs 2f\n"
                     "   str r2, [r0], #4\n"
                     "   b 1b\n"
                     "2: b fw_reset_c\n");
}

__attribute__((noreturn, used)) void fw_reset_c(void) {
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ++dst)
        *dst = 0;

#if defined(__ARM_FP)
    /* Enable CP10 and CP11 (the FPU) before any floating-point code runs. */
    *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;
    __asm__ volatile("dsb\n isb" ::: "memory");
#endif

    sh_exit(main());
}

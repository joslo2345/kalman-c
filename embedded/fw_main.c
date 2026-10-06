/*
 * Benchmark firmware: runs the first FW_STEPS steps of scenario S2 through
 * one filter, then reports the final state and the stack high-water mark over
 * semihosting. Two images per library, differing only in fw_steps (1000 or
 * 0), give the instruction count of 1000 predict+update steps by difference.
 */
#include <stdint.h>
#include <string.h>

#include "fw_data.h"
#include "fw_filter.h"
#include "semihost.h"

uint32_t fw_stack_used_bytes(void);

/* volatile so the 1000-step and 0-step images contain identical code. */
static volatile int fw_steps = FW_STEPS;

int main(void) {
    const int steps = fw_steps;

    filter_init();
    for (int k = 0; k < steps; ++k) {
        if (filter_step(s2_z[k]) != 0) {
            sh_puts("fw: step failed at ");
            sh_put_int(k);
            sh_puts("\n");
            return 1;
        }
    }

    const unsigned char *x = filter_x();
    sh_puts("state:");
    for (int i = 0; i < S2_N; ++i) {
        uint32_t bits;
        memcpy(&bits, x + (4 * i), sizeof bits);
        sh_puts(" ");
        sh_put_hex(bits);
    }
    sh_puts("\nstack_bytes: ");
    sh_put_int((long)fw_stack_used_bytes());
    sh_puts("\n");
    return 0;
}

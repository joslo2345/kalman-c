#include "semihost.h"

#define SYS_WRITE0 0x04
#define SYS_EXIT 0x18
#define ADP_STOPPED_APPLICATION_EXIT 0x20026

static int semihost(int op, const void *arg) {
    register int r0 __asm__("r0") = op;
    register const void *r1 __asm__("r1") = arg;
    __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
    return r0;
}

void sh_puts(const char *s) {
    semihost(SYS_WRITE0, s);
}

void sh_put_int(long v) {
    char buf[24];
    char *p = &buf[sizeof buf - 1];
    unsigned long u = v < 0 ? 0ul - (unsigned long)v : (unsigned long)v;
    *p = '\0';
    do {
        *--p = (char)('0' + u % 10);
        u /= 10;
    } while (u != 0);
    if (v < 0)
        *--p = '-';
    sh_puts(p);
}

void sh_put_hex(unsigned v) {
    char buf[11] = "0x";
    for (int i = 0; i < 8; ++i) {
        const unsigned d = (v >> (28 - 4 * i)) & 0xF;
        buf[2 + i] = (char)(d < 10 ? '0' + d : 'a' + d - 10);
    }
    buf[10] = '\0';
    sh_puts(buf);
}

void sh_exit(int code) {
    /* On 32-bit Arm, SYS_EXIT takes the reason code directly; 0x20026 means success. */
    semihost(SYS_EXIT, (const void *)(code == 0 ? ADP_STOPPED_APPLICATION_EXIT : 0x20023));
    for (;;) {
    }
}

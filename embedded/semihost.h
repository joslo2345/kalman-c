#ifndef FW_SEMIHOST_H
#define FW_SEMIHOST_H

/* Minimal Arm semihosting: text output and exit, as supported by QEMU. */

void sh_puts(const char *s);
void sh_put_int(long v);
void sh_put_hex(unsigned v);
void sh_exit(int code) __attribute__((noreturn));

#endif

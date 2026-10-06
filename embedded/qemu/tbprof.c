/*
 * QEMU TCG plugin: execution profile by translation block. At exit it prints
 * one line per block, "tbprof <vaddr> <instructions> <executions>", for
 * scripts/profile_firmware.py to map to source lines and functions.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct block {
    uint64_t vaddr;
    uint64_t insns;
    uint64_t count;
    struct block *next;
} block;

static block *blocks;

static void tb_exec(unsigned int vcpu, void *userdata) {
    (void)vcpu;
    ((block *)userdata)->count++;
}

static void tb_trans(struct qemu_plugin_tb *tb, void *userdata) {
    (void)userdata;
    /* A block may be translated more than once; each translation counts on its own. */
    block *b = calloc(1, sizeof *b);
    b->vaddr = qemu_plugin_tb_vaddr(tb);
    b->insns = qemu_plugin_tb_n_insns(tb);
    b->next = blocks;
    blocks = b;
    qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_exec, QEMU_PLUGIN_CB_NO_REGS, b);
}

static void at_exit(void *userdata) {
    char line[96];
    (void)userdata;
    for (block *b = blocks; b != NULL; b = b->next) {
        if (b->count == 0)
            continue;
        snprintf(line, sizeof line, "tbprof %" PRIx64 " %" PRIu64 " %" PRIu64 "\n", b->vaddr,
                 b->insns, b->count);
        qemu_plugin_outs(line);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info, int argc,
                                           char **argv) {
    (void)info;
    (void)argc;
    (void)argv;
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, at_exit, NULL);
    return 0;
}

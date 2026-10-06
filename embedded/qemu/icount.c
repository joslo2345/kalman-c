/*
 * QEMU TCG plugin that counts guest instructions executed and prints the
 * total at exit as "icount: <n>". Used for instructions-per-step on emulated
 * Cortex-M targets, since QEMU does not model the DWT cycle counter.
 */
#include <inttypes.h>
#include <stdio.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static struct qemu_plugin_scoreboard *counts;
static qemu_plugin_u64 insns;

static void tb_trans(struct qemu_plugin_tb *tb, void *userdata) {
    (void)userdata;
    qemu_plugin_register_vcpu_tb_exec_inline_per_vcpu(tb, QEMU_PLUGIN_INLINE_ADD_U64, insns,
                                                       qemu_plugin_tb_n_insns(tb));
}

static void at_exit(void *userdata) {
    char buf[64];
    (void)userdata;
    snprintf(buf, sizeof buf, "icount: %" PRIu64 "\n", qemu_plugin_u64_sum(insns));
    qemu_plugin_outs(buf);
    qemu_plugin_scoreboard_free(counts);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info, int argc,
                                           char **argv) {
    (void)info;
    (void)argc;
    (void)argv;
    counts = qemu_plugin_scoreboard_new(sizeof(uint64_t));
    insns = qemu_plugin_scoreboard_u64(counts);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, at_exit, NULL);
    return 0;
}

"""Run the Cortex-M firmware under QEMU and print benchmark CSV rows.

Usage: python scripts/run_embedded.py <build-fw dir> [--plugin path] [--commit HASH]

For each library and target this runs <prefix><lib>_1000.elf and
<prefix><lib>_0.elf with the icount plugin, and reports:

  instructions_per_step  (icount(1000) - icount(0)) / 1000, exact for QEMU's
                         instruction stream. QEMU does not model pipeline
                         timing or flash wait states, so this is not cycles.
  flash_bytes            text + data of the 1000-step image (arm-none-eabi-size)
  ram_bytes              data + bss of the same image
  stack_bytes            peak stack use, from the painted-stack high-water mark

Rows use the schema of results/results.csv. The plugin is built from
embedded/qemu/icount.c with the host compiler if --plugin isn't given.
"""
import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
from datetime import date

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Targets: image prefix, QEMU machine, CPU label, and the suffix that keeps
# their rows apart in the table's precision column.
TARGETS = {
    "m4": ("fw_", "netduinoplus2", "cortex-m4f (qemu netduinoplus2)", ""),
    "m3": ("fw_m3_", "netduino2", "cortex-m3 no FPU (qemu netduino2)", "@m3"),
}
LIBS = [  # (target, firmware name, library name, version source, filter, precision)
    ("m4", "kalman_c", "kalman-c", "kalman", "KF", "float32"),
    ("m4", "kalman_c_specialized", "kalman-c-specialized", "kalman", "KF", "float32"),
    ("m4", "kalman_c_sr", "kalman-c", "kalman", "SRKF", "float32"),
    ("m4", "kalman_c_fixed", "kalman-c", "kalman", "KF", "q20"),
    ("m4", "naive", "naive", "naive", "KF", "float32"),
    ("m4", "tinyekf", "tinyekf", "tinyekf", "KF", "float32"),
    ("m3", "kalman_c", "kalman-c", "kalman", "KF", "float32"),
    ("m3", "kalman_c_fixed", "kalman-c", "kalman", "KF", "q20"),
    ("m3", "tinyekf", "tinyekf", "tinyekf", "KF", "float32"),
]
STEPS = 1000


def run(cmd, **kw):
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw)


def build_plugin(out_dir):
    path = os.path.join(out_dir, "libicount.so")
    src = os.path.join(ROOT, "embedded", "qemu", "icount.c")
    if os.path.exists(path) and os.path.getmtime(path) > os.path.getmtime(src):
        return path
    glib = run(["pkg-config", "--cflags", "glib-2.0"]).stdout.split()
    qemu_inc = os.path.join(os.path.dirname(os.path.dirname(shutil.which("qemu-system-arm"))),
                            "include")
    flags = ["-undefined", "dynamic_lookup"] if sys.platform == "darwin" else []
    run([os.environ.get("CC", "cc"), "-shared", "-fPIC", "-O2", *glib, f"-I{qemu_inc}", *flags,
         src, "-o", path])
    return path


def qemu(elf, plugin, machine):
    out = subprocess.run(
        ["qemu-system-arm", "-M", machine, "-nographic", "-monitor", "none",
         "-serial", "none", "-semihosting-config", "enable=on,target=native",
         "-kernel", elf, "-plugin", plugin, "-d", "plugin"],
        capture_output=True, text=True, timeout=600)
    text = out.stdout + out.stderr
    if out.returncode != 0:
        sys.exit(f"{elf} failed (exit {out.returncode}):\n{text}")
    icount = int(re.search(r"icount: (\d+)", text).group(1))
    state = re.search(r"state:((?: 0x[0-9a-f]{8})+)", text).group(1).split()
    stack = int(re.search(r"stack_bytes: (\d+)", text).group(1))
    return icount, state, stack


def size(elf, size_tool):
    line = run([size_tool, elf]).stdout.splitlines()[1].split()
    text, data, bss = (int(v) for v in line[:3])
    return text + data, data + bss


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build_dir")
    ap.add_argument("--plugin")
    ap.add_argument("--commit")
    args = ap.parse_args()
    bdir = os.path.abspath(args.build_dir)

    cache = open(os.path.join(bdir, "CMakeCache.txt")).read()
    size_tool = re.search(r"^CMAKE_SIZE:FILEPATH=(.*)$", cache, re.M).group(1)
    gcc = re.sub(r"size$", "gcc", size_tool)  # same toolchain directory
    gcc_version = run([gcc, "-dumpversion"]).stdout.strip()
    qemu_version = re.search(r"version ([\d.]+)", run(["qemu-system-arm", "--version"]).stdout).group(1)
    plugin = args.plugin or build_plugin(bdir)

    commit = args.commit or run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"]).stdout.strip()
    config = open(os.path.join(ROOT, "include", "kalman", "kf_config.h")).read()
    versions = {
        "kalman": re.search(r'#define KALMAN_C_VERSION "([\d.]+)"', config).group(1),
        "naive": "textbook",
        "tinyekf": run(["git", "-C", os.path.join(ROOT, "tests", "baselines", "tinyekf"),
                        "rev-parse", "--short", "HEAD"]).stdout.strip(),
    }
    toolchain = f"arm-none-eabi-gcc-{gcc_version}+qemu-{qemu_version}"

    def elf(target, name, steps):
        return os.path.join(bdir, f"{TARGETS[target][0]}{name}_{steps}.elf")

    for target, (_, machine, cpu, _) in TARGETS.items():
        n1, _, _ = qemu(elf(target, "empty", STEPS), plugin, machine)
        n0, _, _ = qemu(elf(target, "empty", 0), plugin, machine)
        flash, ram = size(elf(target, "empty", STEPS), size_tool)
        print(f"# {target} harness only (empty): flash {flash} B, ram {ram} B, "
              f"{(n1 - n0) / STEPS:.1f} instructions per loop iteration", file=sys.stderr)

    for target, fw, lib, ver, filt, precision in LIBS:
        _, machine, cpu, suffix = TARGETS[target]
        env = ",".join([commit, cpu, "bare-metal", toolchain, date.today().isoformat()])
        n1, state, stack = qemu(elf(target, fw, STEPS), plugin, machine)
        n0, _, _ = qemu(elf(target, fw, 0), plugin, machine)
        flash, ram = size(elf(target, fw, STEPS), size_tool)
        rows = [("instructions_per_step", f"{(n1 - n0) / STEPS:.1f}", "instructions"),
                ("flash_bytes", flash, "bytes"),
                ("ram_bytes", ram, "bytes"),
                ("stack_bytes", stack, "bytes")]
        for metric, value, unit in rows:
            print(f"{lib},{versions[ver]},S2,{filt},{precision}{suffix},{metric},{value},{unit},{env}")
        print(f"# {target} {lib} {filt} {precision}: final state {decode(state, precision)}", file=sys.stderr)


def decode(words, precision):
    """Final state words as numbers: IEEE float32, or Q-format int32."""
    values = []
    for w in words:
        u = int(w, 16)
        if precision.startswith("q"):
            frac = int(precision[1:])
            values.append((u - (1 << 32) if u >= 1 << 31 else u) / (1 << frac))
        else:
            values.append(struct.unpack("<f", struct.pack("<I", u))[0])
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


if __name__ == "__main__":
    main()

"""Run the Cortex-M4F firmware under QEMU and print benchmark CSV rows.

Usage: python scripts/run_embedded.py <build-fw dir> [--plugin path] [--commit HASH]

For each library this runs fw_<lib>_1000.elf and fw_<lib>_0.elf on QEMU's
netduinoplus2 (STM32F405) with the icount plugin, and reports:

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
import subprocess
import sys
from datetime import date

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIBS = [  # (firmware name, library name in the table, version source, filter)
    ("kalman_c", "kalman-c", "kalman", "KF"),
    ("kalman_c_specialized", "kalman-c-specialized", "kalman", "KF"),
    ("kalman_c_sr", "kalman-c", "kalman", "SRKF"),
    ("naive", "naive", "naive", "KF"),
    ("tinyekf", "tinyekf", "tinyekf", "KF"),
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


def qemu(elf, plugin):
    out = subprocess.run(
        ["qemu-system-arm", "-M", "netduinoplus2", "-nographic", "-monitor", "none",
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
    env = ",".join([commit, "cortex-m4f (qemu netduinoplus2)", "bare-metal",
                    f"arm-none-eabi-gcc-{gcc_version}+qemu-{qemu_version}", date.today().isoformat()])

    def elf(name, steps):
        return os.path.join(bdir, f"fw_{name}_{steps}.elf")

    base_icount, _, _ = qemu(elf("empty", STEPS), plugin)
    base_icount0, _, _ = qemu(elf("empty", 0), plugin)
    base_flash, base_ram = size(elf("empty", STEPS), size_tool)
    print(f"# harness only (fw_empty): flash {base_flash} B, ram {base_ram} B, "
          f"{(base_icount - base_icount0) / STEPS:.1f} instructions per loop iteration",
          file=sys.stderr)

    states = {}
    for fw, lib, ver, filt in LIBS:
        n1, state, stack = qemu(elf(fw, STEPS), plugin)
        n0, _, _ = qemu(elf(fw, 0), plugin)
        flash, ram = size(elf(fw, STEPS), size_tool)
        states[f"{lib} {filt}"] = state
        rows = [("instructions_per_step", f"{(n1 - n0) / STEPS:.1f}", "instructions"),
                ("flash_bytes", flash, "bytes"),
                ("ram_bytes", ram, "bytes"),
                ("stack_bytes", stack, "bytes")]
        for metric, value, unit in rows:
            print(f"{lib},{versions[ver]},S2,{filt},float32,{metric},{value},{unit},{env}")

    ref = states["kalman-c KF"]
    for lib, state in states.items():
        print(f"# {lib} final state: {' '.join(state)}"
              f"{'' if state == ref else '  (differs from kalman-c in the last bits)'}",
              file=sys.stderr)


if __name__ == "__main__":
    main()

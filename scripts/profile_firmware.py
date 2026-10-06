"""Instruction profile of a firmware image by function and source line.

Usage: python scripts/profile_firmware.py <image.elf> [--machine netduino2] [--top 25]

Runs the image under QEMU with the tbprof plugin (embedded/qemu/tbprof.c,
built on demand next to the image), then maps each executed block to its
innermost function and source line with addr2line (inlined code is
attributed to the inlined function). Build the image with -g.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_plugin(out_dir):
    path = os.path.join(out_dir, "libtbprof.so")
    src = os.path.join(ROOT, "embedded", "qemu", "tbprof.c")
    if os.path.exists(path) and os.path.getmtime(path) > os.path.getmtime(src):
        return path
    glib = subprocess.run(["pkg-config", "--cflags", "glib-2.0"], check=True, capture_output=True,
                          text=True).stdout.split()
    inc = os.path.join(os.path.dirname(os.path.dirname(shutil.which("qemu-system-arm"))), "include")
    flags = ["-undefined", "dynamic_lookup"] if sys.platform == "darwin" else []
    subprocess.run([os.environ.get("CC", "cc"), "-shared", "-fPIC", "-O2", *glib, f"-I{inc}", *flags,
                    src, "-o", path], check=True)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--machine", default="netduino2")
    ap.add_argument("--top", type=int, default=25)
    ap.add_argument("--addr2line", default=None)
    args = ap.parse_args()
    plugin = build_plugin(os.path.dirname(os.path.abspath(args.elf)))
    out = subprocess.run(["qemu-system-arm", "-M", args.machine, "-nographic", "-monitor", "none",
                          "-serial", "none", "-semihosting-config", "enable=on,target=native",
                          "-kernel", args.elf, "-plugin", plugin, "-d", "plugin"],
                         capture_output=True, text=True, timeout=600)
    blocks = [(int(a, 16), int(n), int(c)) for a, n, c in
              re.findall(r"tbprof ([0-9a-f]+) (\d+) (\d+)", out.stdout + out.stderr)]
    if not blocks:
        sys.exit("no profile output:\n" + out.stdout + out.stderr)
    a2l = args.addr2line or shutil.which("arm-none-eabi-addr2line") or os.path.join(
        os.environ.get("ARM_TOOLCHAIN_DIR", ""), "bin", "arm-none-eabi-addr2line")
    addrs = sorted({a for a, _, _ in blocks})
    # One addr2line call per block address keeps the address-to-line mapping exact
    # (with -i, a batched call prints a variable number of lines per address).
    where = {}
    for a in addrs:
        lines = subprocess.run([a2l, "-f", "-i", "-e", args.elf, hex(a)], capture_output=True,
                               text=True, check=True).stdout.splitlines()
        func, loc = (lines[0], lines[1]) if len(lines) >= 2 else ("??", "??:0")
        where[a] = (func, os.path.basename(loc.split(" ")[0]))
    by_func, by_line = Counter(), Counter()
    total = 0
    for a, n, c in blocks:
        func, loc = where[a]
        by_func[func] += n * c
        by_line[f"{func:28s} {loc}"] += n * c
        total += n * c
    print(f"{total:,} instructions executed\n")
    print("by function (innermost, inlined code attributed to the inlined function):")
    for f, v in by_func.most_common(args.top):
        print(f"  {v / total:6.1%} {v:>12,}  {f}")
    print("\nby source line:")
    for f, v in by_line.most_common(args.top):
        print(f"  {v / total:6.1%} {v:>12,}  {f}")


if __name__ == "__main__":
    main()

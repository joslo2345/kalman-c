"""Check the library sources against the enforced subset of MISRA C:2012.

Usage: python scripts/check_misra.py

Runs cppcheck's MISRA addon (misra.py) over src/ in the generic and the
KF_SPECIALIZE configurations, and fails on any violation except:

  Advisory rules not enforced (reported in the summary):
    12.1  explicit precedence parentheses
    15.5  single point of exit (early returns keep the error paths readable)
    20.5  #undef (used to scope the KF_SIZE helper macro)
  Documented deviation:
    21.15 memcpy between kf_real and an unsigned integer of the same width,
          on lines marked "misra-c2012-21.15 deviation" (the finiteness check)

Everything else the addon reports, Required rules included, fails the check.
The addon runs without the licensed rule texts, so findings are rule numbers.
"""
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADVISORY_NOT_ENFORCED = {"12.1", "15.5", "20.5"}
DEVIATION_MARKER = "misra-c2012-21.15 deviation"
CONFIGS = [["-UKF_SPECIALIZE"], ["-DKF_SPECIALIZE=KF_SIZE(4,2)"]]


def find_misra_py():
    cppcheck = shutil.which("cppcheck")
    if cppcheck is None:
        sys.exit("cppcheck not found")
    prefix = os.path.dirname(os.path.dirname(os.path.realpath(cppcheck)))
    for cand in (os.path.join(prefix, "share", "cppcheck", "addons", "misra.py"),
                 "/usr/share/cppcheck/addons/misra.py",
                 "/usr/lib/x86_64-linux-gnu/cppcheck/addons/misra.py"):
        if os.path.exists(cand):
            return cand
    sys.exit("cannot find cppcheck's misra.py addon")


def main():
    misra_py = find_misra_py()
    failures, advisory = [], {}
    with tempfile.TemporaryDirectory() as tmp:
        # cppcheck writes .dump files next to the sources, so work on a copy.
        for f in glob.glob(os.path.join(ROOT, "src", "*")):
            shutil.copy(f, tmp)
        sources = sorted(glob.glob(os.path.join(tmp, "*.c")))
        for cfg in CONFIGS:
            for d in glob.glob(os.path.join(tmp, "*.dump")):
                os.remove(d)
            subprocess.run(["cppcheck", "--dump", "--quiet", "--std=c99",
                            f"-I{os.path.join(ROOT, 'include')}", f"-I{tmp}", *cfg, *sources],
                           check=True, capture_output=True)
            out = subprocess.run([sys.executable, misra_py, *glob.glob(os.path.join(tmp, "*.dump"))],
                                 capture_output=True, text=True)
            for line in (out.stdout + out.stderr).splitlines():
                m = re.match(r"\[(.+?):(\d+)\].*\[misra-c2012-([\d.]+)\]", line)
                if not m:
                    continue
                path, lineno, rule = m.group(1), int(m.group(2)), m.group(3)
                shown = os.path.relpath(path, tmp) if path.startswith(tmp) else os.path.relpath(path, ROOT)
                src = os.path.join(ROOT, "src", shown) if path.startswith(tmp) else path
                text = open(src).read().splitlines()[lineno - 1] if os.path.exists(src) else ""
                if rule in ADVISORY_NOT_ENFORCED:
                    advisory[rule] = advisory.get(rule, 0) + 1
                elif rule == "21.15" and DEVIATION_MARKER in text:
                    continue
                else:
                    failures.append(f"{shown}:{lineno}: misra-c2012-{rule} ({' '.join(cfg)}): {text.strip()}")

    for f in sorted(set(failures)):
        print(f)
    summary = ", ".join(f"{r}: {n}" for r, n in sorted(advisory.items()))
    print(f"advisory (not enforced): {summary or 'none'}")
    if failures:
        print(f"{len(set(failures))} enforced MISRA C:2012 violation(s)")
        return 1
    print("MISRA C:2012 enforced subset: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())

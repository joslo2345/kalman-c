"""Fail if kalman-c got more than 10% slower than the last release.

Usage:
  python scripts/check_regression.py --build build-bench [--build-spec build-bench-spec]
         [--baseline results/regression-baseline.json] [--markdown out.md] [--record]

Wall-clock timings on shared CI machines vary by 10-20%, too much for a 10%
gate. This check counts instructions instead: valgrind's callgrind, collecting
only inside regress_loop() of bench/regress (so file parsing and process setup
are excluded), gives a deterministic count for a given binary and toolchain.
Each case reports instructions per predict+update step.

The baseline (results/regression-baseline.json) is recorded on the CI machine
from the last release's code, with --record. A case fails when it needs more
than (1 + threshold) times its baseline. A case that is more than threshold
faster is reported, so the baseline can be refreshed at the next release.
Counts depend on the compiler, so the baseline stores the toolchain; refresh
it (--record) when CI's compiler changes.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (case name, build, scenario, runner)
CASES = [
    ("S1 KF", "generic", "S1", "kf"),
    ("S2 KF", "generic", "S2", "kf"),
    ("S5 KF", "generic", "S5", "kf"),
    ("S2 KF specialized", "spec", "S2", "kf"),
    ("S5 KF specialized", "spec", "S5", "kf"),
    ("S3 EKF", "generic", "S3", "ekf"),
    ("S3 UKF", "generic", "S3", "ukf"),
    ("S2 square-root (UD)", "generic", "S2", "sr"),
    ("S5 square-root (UD)", "generic", "S5", "sr"),
    ("S2 fixed point (q18)", "generic", "S2", "fx"),
]


def run(cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True)


def count(regress, vectors, scenario, runner, steps):
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "callgrind.out")
        run(["valgrind", "--tool=callgrind", "--toggle-collect=regress_loop",
             f"--callgrind-out-file={out}", regress, vectors, scenario, runner, str(steps)])
        text = open(out).read()
    m = re.search(r"^summary:\s+(\d+)", text, re.M) or re.search(r"^totals:\s+(\d+)", text, re.M)
    if not m:
        sys.exit(f"no instruction count in callgrind output for {scenario} {runner}")
    return int(m.group(1)) / steps


def toolchain(build_dir):
    cache = open(os.path.join(build_dir, "CMakeCache.txt")).read()
    cc = re.search(r"^CMAKE_C_COMPILER:\w+=(.*)$", cache, re.M).group(1)
    return {
        "compiler": run([cc, "--version"]).stdout.splitlines()[0],
        "valgrind": run(["valgrind", "--version"]).stdout.strip(),
        "machine": os.uname().machine,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True, help="bench build (generic)")
    ap.add_argument("--build-spec", help="bench build with KF_SPECIALIZE")
    ap.add_argument("--vectors", default=os.path.join(ROOT, "tests", "vectors"))
    ap.add_argument("--baseline", default=os.path.join(ROOT, "results", "regression-baseline.json"))
    ap.add_argument("--steps", type=int, default=500)
    ap.add_argument("--threshold", type=float, default=0.10)
    ap.add_argument("--markdown", help="write the result table here")
    ap.add_argument("--record", action="store_true", help="write the baseline instead of comparing")
    args = ap.parse_args()

    builds = {"generic": args.build, "spec": args.build_spec}
    tools = toolchain(args.build)
    current = {}
    for name, build, scenario, runner in CASES:
        if builds[build] is None:
            continue
        regress = os.path.join(builds[build], "bench", "regress")
        current[name] = round(count(regress, args.vectors, scenario, runner, args.steps), 1)

    if args.record or not os.path.exists(args.baseline):
        with open(args.baseline, "w") as f:
            json.dump({"toolchain": tools, "steps": args.steps, "instructions_per_step": current},
                      f, indent=2)
            f.write("\n")
        print(f"recorded baseline {args.baseline}:")
        for name, v in current.items():
            print(f"  {name}: {v}")
        if not args.record:
            print("no baseline existed: commit this file, then the check compares against it")
            return 1
        return 0

    base = json.load(open(args.baseline))
    lines = ["| Case | Baseline | Current | Change | Status |", "|---|---:|---:|---:|---|"]
    failed = []
    for name, v in current.items():
        b = base["instructions_per_step"].get(name)
        if b is None:
            lines.append(f"| {name} | – | {v:,.0f} | – | new case, not in baseline |")
            continue
        change = v / b - 1
        if change > args.threshold:
            status = f"❌ more than {args.threshold:.0%} slower"
            failed.append(name)
        elif change < -args.threshold:
            status = "⬇️ faster: refresh the baseline at the next release"
        else:
            status = "✅ ok"
        lines.append(f"| {name} | {b:,.0f} | {v:,.0f} | {change:+.1%} | {status} |")

    notes = [f"Instructions per predict+update step, counted by callgrind inside the filter loop "
             f"({args.steps} steps per case). Fails above +{args.threshold:.0%}."]
    if base.get("toolchain") != tools:
        notes.append(f"⚠️ The toolchain differs from the baseline's ({base.get('toolchain')} vs {tools}); "
                     "counts may shift. Re-record the baseline if the change is expected.")
    report = "\n".join(notes) + "\n\n" + "\n".join(lines) + "\n"
    print(report)
    if args.markdown:
        open(args.markdown, "w").write(report)
    if failed:
        print(f"regression in: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

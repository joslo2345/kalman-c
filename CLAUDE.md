# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

`kalman-c-repo-guide.md` is the design spec and build plan for this embedded-friendly Kalman filter library in C99. Treat it as the source of truth, and read the relevant step before you implement anything.

Steps 1–5 (scaffolding) and Steps 6.1–6.2 are done. `kf_linalg` and the linear KF (`kf_init`, `kf_predict`, `kf_update` with the Joseph form and NIS) are implemented and tested. The EKF and UKF headers are still empty placeholders. Next up is Step 6.3 (EKF), followed by the Step 7 baselines and comparison tests.

Deviations from the guide:

- The error codes and `kf_state` live in `include/kalman/kf_types.h`, so that every filter header can share them.
- `kf_config.h` adds `KF_MAX_DIM`, the larger of `KF_MAX_STATE` and `KF_MAX_MEAS`, for sizing square scratch buffers.

Implementation conventions:

- Filter functions compute into local stack buffers and copy into `*kf` only after the whole step has succeeded. That is how "state unchanged on error" is guaranteed. Keep this pattern in new filters.
- Never invert S explicitly. Factor it with `kf_cholesky`, then use `kf_cholesky_solve` and `kf_solve_lower`.
- `kf_linalg` outputs must not alias their inputs, except where the header says they may.
- Unity's double assertions are enabled (`UNITY_INCLUDE_DOUBLE`). Tests compare through `(double)` with tolerances that depend on precision.

## Non-negotiable design constraints

These apply to every change to library code (`src/`, `include/kalman/`):

- **No dynamic allocation.** No `malloc`, `calloc` or `realloc` in the library. All storage lives in caller-owned structs sized by `KF_MAX_STATE` and `KF_MAX_MEAS`. A test (`test_no_malloc`) enforces this by wrapping these calls with `-Wl,--wrap=...`.
- **Precision is chosen at compile time.** Use `kf_real` everywhere, never a literal `float` or `double` type. `kf_real` is `float` by default and `double` with `KF_USE_DOUBLE`. `kf_config.h` also defines `KF_PRECISION_NAME` (`"float32"` or `"float64"`) for the benchmark CSV output.
- **Numerically robust updates.** Covariance updates use the Joseph form, or a square-root form (Cholesky/UD). Never use the textbook `P = (I - KH)P` update. That form exists only as the `naive_kf` baseline that the library is compared against.
- **Errors are returned, never silent.** Every API function returns an error code (`KF_OK`, `KF_ERR_INVALID_INPUT`, `KF_ERR_NOT_POSITIVE_DEFINITE`, ...). On error, the filter state must be left **byte-for-byte unchanged**. Tests check this with `TEST_ASSERT_EQUAL_MEMORY`, so validate inputs (NaN values, a non-positive-definite innovation covariance) before you write to `kf_state`.
- **C99 portability.** The library must build with `-Wall -Wextra -Wpedantic` on GCC, Clang, and `arm-none-eabi-gcc`.

## Architecture

- `include/kalman/`: public headers. `kalman.h` is the umbrella API. `kf_config.h` sets the precision and the maximum dimensions. `kf_linalg.h` has the fixed-size matrix routines. `kf_linear.h`, `kf_ekf.h` and `kf_ukf.h` declare one header per filter type, all behind a consistent API (`kf_init`, `kf_predict`, `kf_update` on a `kf_state`).
- Everything sits on top of `kf_linalg` (multiply, transpose, Cholesky, triangular solve). The EKF takes user-supplied function pointers for the models and Jacobians. The UKF generates its sigma points with Cholesky.
- Matrices are flat, row-major `kf_real` arrays sized for the maximum dimensions (`KF_MAX_STATE * KF_MAX_STATE`). The runtime sizes are `n` and `m`.
- The `kf_state` struct stores `nis` after each update, for consistency diagnostics (NIS/NEES).

Implementation order (from the guide): linalg → linear KF → EKF → UKF → square-root variant → fixed-point.

## Commands

```bash
# Configure, build, and test (Unity is fetched via FetchContent at v2.6.0, so the first configure needs network)
cmake -B build                                # add -DKF_USE_DOUBLE=ON for double precision
cmake --build build
ctest --test-dir build --output-on-failure
ctest --test-dir build -R test_errors         # run a single test by name
```

Add new tests in `tests/CMakeLists.txt` with `kf_add_test(<name> <source>)`. The `no_malloc` test uses `-Wl,--wrap`, so it's only registered with non-Apple linkers.

On this macOS machine, the default SDK (MacOSX27.0, from Command Line Tools) can't be read by Xcode's `ld`, so any link fails with "tapi error: malformed file". Work around it with `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk` before you configure.

Benchmarks (planned; `bench/` doesn't exist yet, so `KF_BUILD_BENCH=ON` currently stops with a fatal error):

```bash

# Benchmarks (Release build, appends to results/results.csv, regenerates README table)
cmake -B build-bench -DCMAKE_BUILD_TYPE=Release -DKF_BUILD_BENCH=ON
cmake --build build-bench
./build-bench/bench "$(git rev-parse --short HEAD),$(uname -m),$(uname -s),gcc-$(gcc -dumpversion),$(date -I)" >> results/results.csv
python scripts/make_table.py results/results.csv kalman-c
```

## Testing and benchmarking model

- `tests/unit/` tests our code in isolation. `tests/comparison/` runs our code against the baselines for equivalence, stability, zero allocations, and NIS consistency. `tests/scenarios/` holds problem generators with fixed seeds.
- On accuracy, the goal is to **match** the baselines (within 1e-5 for float, 1e-9 for double), not to beat them. The gains are in stability, memory use, robustness, consistency, and speed.
- Comparison tests print `[report] ...` lines for baseline behavior that isn't guaranteed, such as the step where the naive filter fails. Those lines feed the CI comparison report. Don't assert on baseline failures.
- Timing belongs in `bench/`, not in unit tests. Every library gets a runner with the signature `int run_X(const scenario *sc, run_result *out)`. Only predict+update is timed.
- The benchmark scenarios S1–S5 are shared data files in a test-vectors Git submodule, and the C, C++, Python and Rust sibling repos all use them. Treat them as frozen: don't change them to improve results.
- The baselines are TinyEKF (a submodule pinned to one commit; record the hash as `library_version`) and a textbook `naive_kf.c`.

## Open question from the guide

Step 7 puts baselines in `tests/baselines/`, but Step 8 puts TinyEKF in `bench/baselines/tinyekf`. Pick one location, ideally a single submodule shared by tests and bench. The tests already use the Step 7 layout (`unit/`, `comparison/`, ...), not the flat layout from Step 2.

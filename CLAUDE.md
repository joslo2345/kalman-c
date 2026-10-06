# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

`kalman-c-repo-guide.md` is the design spec and build plan for this embedded-friendly Kalman filter library in C99. Treat it as the source of truth, and read the relevant step before you implement anything.

Done so far:

- Steps 1–5: scaffolding.
- Steps 6.1–6.4: `kf_linalg`, the linear KF, the EKF, and the UKF.
- Step 7: baselines, scenarios, and comparison tests.

Still to do: Step 8 (benchmarks and comparison numbers), Steps 9–12, and Steps 6.5 (square-root variant) and 6.6 (fixed-point). The stability test shows the Joseph-form KF stays positive-definite for 1M float steps where the baselines fail, so the square-root variant is not urgent.

The KF and EKF share one covariance implementation. `src/kf_internal.h` declares `kf_core_predict`, which takes a predicted x and F, and `kf_core_update`, which takes an innovation y and H. The public KF and EKF functions only compute those inputs and delegate. As a result, an EKF with linear callbacks is bit-identical to the KF, and `test_ekf` asserts this. EKF callbacks return the model value and its Jacobian together, take a `void *ctx`, and turn a non-zero return into `KF_ERR_MODEL_FAILED`.

The UKF (`src/kf_ukf.c`) is self-contained and doesn't use the `kf_core_*` helpers:

- Its callbacks return only the model value, with no Jacobian.
- It takes a `kf_ukf_params` (alpha, beta, kappa). Passing NULL uses alpha=1, beta=2, kappa=0, chosen because a small alpha produces a huge negative centre weight that ruins float precision.
- The update uses `P - K S K^T`, followed by symmetrization and a Cholesky check, since there's no H for a Joseph form.
- Predict and update both reject a resulting P that isn't positive-definite.
- Sigma points are stored one per row, `X[j * n + i]`.

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

- **No dynamic allocation.** No `malloc`, `calloc` or `realloc` in the library. All storage lives in caller-owned structs sized by `KF_MAX_STATE` and `KF_MAX_MEAS`. Two tests enforce this. `no_malloc_symbols` runs everywhere and fails if `nm -u libkalman.a` lists any heap function. `no_malloc` wraps the allocators with `-Wl,--wrap=...` and runs the KF, EKF and UKF; it only runs with non-Apple linkers.
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

TinyEKF is a git submodule, and configure fails without it. After cloning, run `git submodule update --init`, or clone with `--recursive`.

Add new tests in `tests/CMakeLists.txt` with `kf_add_test(<name> <source>)`. Every test links against `kf_test_support`, which contains the scenarios and baselines. `test_stability` runs 2M filter steps and takes a few seconds. Exclude it with `ctest -E stability` for a quick loop. To see the `[report]` lines, use `ctest -V -R 'stability|consistency'`.

On this macOS machine, the default SDK (MacOSX27.0, from Command Line Tools) can't be read by Xcode's `ld`, so any link fails with "tapi error: malformed file". Work around it with `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk` before you configure.

Benchmarks (planned; `bench/` doesn't exist yet, so `KF_BUILD_BENCH=ON` currently stops with a fatal error):

```bash
# Release build, appends to results/results.csv, regenerates README table
cmake -B build-bench -DCMAKE_BUILD_TYPE=Release -DKF_BUILD_BENCH=ON
cmake --build build-bench
./build-bench/bench "$(git rev-parse --short HEAD),$(uname -m),$(uname -s),gcc-$(gcc -dumpversion),$(date -I)" >> results/results.csv
python scripts/make_table.py results/results.csv kalman-c
```

## Testing and benchmarking model

- `tests/unit/` tests our code in isolation. `tests/comparison/` runs our code against the baselines: equivalence, stability, consistency, and no-malloc.
- `tests/scenarios/` holds deterministic problem generators. `rng` is splitmix64 with Box-Muller. `constant_velocity` builds a 4/2 CV model and stores truth and measurements; its truth is simulated in double, so float and double builds see the same data. `ill_conditioned` is the same model with a tiny R and a large Q, observing a stationary target, and generates measurements on the fly for million-step runs.
- `tests/baselines/` holds the comparison filters:
  - `naive_kf.c`: the textbook filter, with explicit inverse and `(I - KH)P`. Don't "fix" it.
  - `tinyekf/`: a submodule pinned to `ba0d2a9`. Record that hash as `library_version`. Step 8's benchmarks should reuse this copy rather than adding a second one under `bench/`.
  - `baseline_tinyekf.{h,c}`: the adapter. TinyEKF fixes its dimensions at compile time, so the adapter is built for 4/2 (`tinyekf_4x2`). It is compiled with `-w` because the header uses non-C99 `= {}` initializers.
- On accuracy, the goal is to **match** the baselines, not to beat them. Tolerances are relative: 1e-9 for double, and 5e-5 for float rather than the guide's 1e-5. In float, TinyEKF's own rounding error over 1,000 steps (1.4e-5 relative to a float64 run) exceeds 1e-5. The comment in `test_equivalence.c` has the numbers.
- Comparison tests print `[report] ...` lines for baseline behaviour that isn't guaranteed, such as the step where a baseline loses positive-definiteness. Those lines feed the CI comparison report. Don't assert on baseline failures.
- Consistency checks the average NIS over all trials and steps, using chi-squared bounds with TRIALS·STEPS·m degrees of freedom. NEES is checked only at the final step across trials, because errors are correlated over time. The bounds use the Wilson–Hilferty approximation.
- Timing belongs in `bench/`, not in unit tests. Every library gets a runner with the signature `int run_X(const scenario *sc, run_result *out)`. Only predict+update is timed.
- The benchmark scenarios S1–S5 are shared data files in a test-vectors Git submodule, and the C, C++, Python and Rust sibling repos all use them. Treat them as frozen: don't change them to improve results. The `tests/scenarios/` generators are for tests, not for those published numbers.

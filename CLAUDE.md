# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

`kalman-c-repo-guide.md` is the design spec and build plan for this embedded-friendly Kalman filter library in C99. It is kept locally only: it is git-ignored and was removed from the public repository and its history. When it is present, treat it as the source of truth and read the relevant step before you implement anything; the "Steps" referred to below are its steps.

Done so far:

- Steps 1–5: scaffolding.
- Steps 6.1–6.4: `kf_linalg`, the linear KF, the EKF, and the UKF.
- Step 7: baselines, scenarios, and comparison tests.
- Step 8: the frozen S1–S5 vectors, the desktop benchmark harness, and the README table.
- Step 8, embedded numbers: Cortex-M4F firmware run under QEMU (no physical board).

Still to do: Steps 9–12, and Steps 6.5 (square-root variant) and 6.6 (fixed-point). Real-board DWT cycle counts are still missing, because QEMU can't provide them (see below).

Performance (see the README table): the generic build is about 2–3× slower than TinyEKF. With `KF_SPECIALIZE` listing the sizes in use, it matches or beats TinyEKF (S2: 57 vs 63 ns; S5: 2.3 vs 2.05 µs on an M3 Pro). TinyEKF's speed comes from compile-time dimensions, and `KF_SPECIALIZE` gives us the same. The stability test shows the Joseph-form KF stays positive-definite for 1M float steps where the baselines fail, so the square-root variant is not urgent.

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

- Every public filter entry point checks `kf_dims_ok(kf)` (in `src/kf_internal.h`) before indexing. `kf->n` and `kf->m` are plain fields a caller can overwrite after `kf_init`, and every scratch buffer is sized by `KF_MAX_*`. Without the check, clang-tidy's analyzer reports stack buffer overruns. New entry points must call it too.
- Filter functions compute into local stack buffers and copy into `*kf` only after the whole step has succeeded. That is how "state unchanged on error" is guaranteed. Keep this pattern in new filters.
- Never invert S explicitly. Factor it with `kf_cholesky`, then use `kf_cholesky_solve` and `kf_solve_lower`.
- The matrix kernels live in `src/kf_linalg_impl.h` as `KFI_INLINE` (always-inline) `kfi_*` functions. Filter sources call those. `kf_linalg.c` holds only the public `kf_*` wrappers. Inlining is what lets `KF_SPECIALIZE` propagate constant sizes, so don't call the public wrappers from filter code.
- The KF/EKF Joseph update runs in O(n²m), without forming `I - KH`: `B = P - K(HP)`, then `B - (BHᵀ)Kᵀ + KRKᵀ`, computing only the upper triangle. Predict also computes only the upper triangle.
- Performance experiments that failed, measured on M3 Pro (don't retry them without new evidence):
  - Row-axpy matmul: slower for n ≤ 4, only about 8% faster at n = 15.
  - Full-matrix axpy update: slower everywhere.
- `kfi_all_finite` tests the exponent bits (all ones means NaN or Inf), reading each value through `memcpy` into an integer. An earlier version summed `v * 0`, but that built a floating-point dependency chain; replacing it made S5 about 22% faster. The bit test also stays correct under `-ffast-math`.
- `core_update` computes `1 / diag(L)` once and multiplies by it in the triangular solves (`kfi_solve_lower_rd` and `kfi_solve_lower_t_rd`), instead of dividing for every element.
- **Square-root filter (`src/kf_sqrt.c`, `kf_sr_*`) is UD, not QR, on purpose.**
  - A Householder-QR array square-root filter was built first. In float32 it was *less* accurate than the Joseph form, because the 1 − 0.999… cancellation in the array update loses digits.
  - NumPy's float32 QR silently computes in float64, so it is not a valid float32 reference.
  - Bierman/Thornton UD forms every new variance from ratios and sums of non-negative terms. It beats the Joseph form by up to about 800× at tiny R, and `test_sqrt_accuracy` guards that.
  - `kfi_ud_factor`: with `psd == 0` it requires only d > 0. A relative tolerance there wrongly rejected S5's P0, where a 1e-6 bias variance sits next to variances of 1. With `psd != 0`, a near-zero pivot is accepted only if its column is negligible, and it keeps tiny positive values.
- **Fixed-point filter (`src/kf_fixed.c`, `kf_fx_*`):**
  - It's integer-only, with no `kf_real` and no `kf_linalg_impl.h`.
  - Products are `int64_t` values with 2F fractional bits. `wide_dot` divides each one by 2^`FX_GUARD` (2^g ≥ `KF_MAX_DIM`), so the sum cannot overflow, and checks once at the end; other sums use the checked `wide_add` and `wide_sub`. Results are rounded once (`round_shift`).
  - **Fixed-point performance, measured on the M3** with `scripts/profile_firmware.py` (firmware is built with `-g`):
    - Per-term overflow checks in `wide_dot` were 46% of the instructions. Guard bits brought the step from 14,064 to 12,576.
    - Tried and rejected, so don't retry them without new evidence:
      - Exact 96-bit carry accumulation: 14,518, because GCC doesn't emit add-with-carry for it.
      - The unsigned offset-and-shift floor: 13,885.
      - Replacing `memcpy` with loops: GCC turns them back into `memcpy` calls. newlib-nano's byte-wise `memcpy` is the real cost.
    - What's left: 64-bit division (`div_wide` → `__udivmoddi4`, about 11%) and `isqrt64` (about 6%).
    - On x86-64 the guard bits cost about 9% (CI regression case "S2 fixed point (q18)": 8,109 → 8,814, within the 10% gate). That's accepted, because the filter targets 32-bit MCUs. Re-record the regression baseline at the next release so that case gets headroom again.
  - The fixed-point KF and EKF share `fx_core_predict` and `fx_core_update`, which re-check `n` and `m`. The test helper `fx_dot` in `test_fixed.c` mirrors `wide_dot`'s rounding, so the EKF stays bit-identical to the KF. Keep the two in sync.
  - It never right-shifts a negative number (implementation-defined in C99) and never negates a sum (`INT64_MIN`); use `wide_sub`.
  - Every narrowing goes through `narrow()`, which sets `ovf`. Any `ovf` makes the call return `KF_ERR_OVERFLOW` before it commits.
  - Use the `KF_FX_FRAC` CMake option (applied PUBLIC), never a per-target define: headers and library must agree. The bench build defaults to 18.
  - 64-bit division (`div_wide`) is a library call on Cortex-M. It's the main cost on the M3.
- Filter cores (`kf_core_*`, `sr_core_*`) re-check `n` and `m` themselves: model callbacks receive `ctx`, which may alias the filter state.
- Specialized and generic builds agree to rounding, not bit-for-bit, because constant sizes change vectorization and FMA contraction. Tests use tolerances, so both pass. The EKF-vs-KF bit-identity test still holds, since both share one core.
- `kf_linalg` outputs must not alias their inputs, except where the header says they may.
- Unity's double assertions are enabled (`UNITY_INCLUDE_DOUBLE`). Tests compare through `(double)` with tolerances that depend on precision.

## Versioning and releases

- **The version has one source:** `#define KALMAN_C_VERSION` in `include/kalman/kf_config.h`. The top-level CMake reads `project(VERSION)` from it, and so do the bench and `run_embedded.py`. `library.json` and `CHANGELOG.md` must be updated by hand to match.
- **To release:**
  1. Bump `KALMAN_C_VERSION` and `library.json`.
  2. Add a `## [X.Y.Z]` section to `CHANGELOG.md`.
  3. Re-run the benchmarks (desktop and embedded) so the README table matches the released code, then `python3 scripts/make_readme_charts.py`.
  4. Commit, wait for CI to pass, then push the tag `vX.Y.Z`.
  5. Re-record the regression baseline from the released code. Download `results/regression-baseline.json` from the `benchmarks` job's `comparison-report` artifact after running `check_regression.py --record`, or delete the file so the next run records it; then commit it.

  `.github/workflows/release.yml` then checks that the tag matches the version, builds and tests the single header, and publishes a GitHub Release. The release body is that version's changelog section, and the assets are `kalman_c.h` and the Arduino zip.
- **Packaging:**
  - `scripts/amalgamate.py` builds `kalman_c.h` from `include/` and `src/` in a fixed file order. Implementation sources go behind `KALMAN_C_IMPLEMENTATION`.
  - Static names must stay unique across the `src/*.c` files, because they all end up in one translation unit.
  - `--arduino` also writes the Arduino library zip. The Arduino Library Manager isn't used, because the repository's `include/` layout doesn't match Arduino's `src/` convention.
- **Every function-declaring public header has `extern "C"` guards.** The `test_cpp_caller` and `test_cpp_single_header` tests fail to link without them.

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

README layout. It's styled after the CubeSandbox README: centered header, badges, a News table, a Highlights card grid, a comparison table with collapsed details, and a Mermaid architecture diagram. Constraints:
- **Keep these exactly:** the `## Quick start` heading followed by its ```` ```c ```` block (the build extracts and runs it), and the `<!-- BENCH:START/END -->` markers (`scripts/update_readme_table.py` rewrites between them; they sit inside a `<details>` block).
- **Charts are generated.** `python3 scripts/make_readme_charts.py` writes `docs/assets/bench-{desktop,embedded}-{light,dark}.svg` from the latest CSV rows, and the README swaps them with `<picture>` for dark mode.
  - Series colours are fixed per library, in the validated categorical order: blue kalman-c, orange specialized, aqua fixed point, yellow textbook, magenta TinyEKF.
  - Re-run after recording new results.
  - `docs/assets/logo.svg` is hand-made: a prediction and a measurement Gaussian fusing into the blue estimate.
- **Every number in the README prose comes from `results/results.csv` or a test `[report]`.** Update the prose when the numbers change.

Add new tests in `tests/CMakeLists.txt` with `kf_add_test(<name> <source>)`. The examples in `examples/` also run as tests: each exits non-zero if its filter doesn't clearly beat the raw sensor. So does the README's quick start, which `examples/CMakeLists.txt` extracts from the `## Quick start` C block at configure time. Keep that heading and block intact.

API docs: `doxygen` writes `build-docs/html`, with `WARN_AS_ERROR`, so every public symbol must have a `/** */` comment. Doxygen reads the default branch of `#ifdef`s (`float`, no `KF_SPECIALIZE`), so put docs there. Every test links against `kf_test_support`, which contains the scenarios and baselines. `test_stability` runs 2M filter steps and takes a few seconds. Exclude it with `ctest -E stability` for a quick loop. To see the `[report]` lines, use `ctest -V -R 'stability|consistency'`.

On this macOS machine, the default SDK (MacOSX27.0, from Command Line Tools) can't be read by Xcode's `ld`, so any link fails with "tapi error: malformed file". Work around it with `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk` before you configure.

Benchmarks. Run once per precision, then regenerate the README table:

```bash
cmake -B build-bench -DCMAKE_BUILD_TYPE=Release -DKF_BUILD_BENCH=ON -DKF_BUILD_TESTS=OFF   # add -DKF_USE_DOUBLE=ON for float64
cmake --build build-bench
./build-bench/bench/bench "$(git rev-parse --short HEAD),$(uname -m),$(uname -s),clang-$(cc -dumpversion),$(date +%F)" >> results/results.csv
python3 scripts/update_readme_table.py results/results.csv kalman-c
```

- The executable is at `build-bench/bench/bench`, not `build-bench/bench` as the guide says.
- `KF_BUILD_BENCH` raises `KF_MAX_STATE` to 15 for S5. The `KF_MAX_STATE` and `KF_MAX_MEAS` cache variables override the defaults in any build.
- For the specialized column, configure a second bench build with `'-DKF_SPECIALIZE=KF_SIZE(2,1)KF_SIZE(4,2)KF_SIZE(15,6)'`. It reports as `kalman-c-specialized`. Always pass `KF_SPECIALIZE` as a cache variable: in `CMAKE_C_FLAGS`, the Makefile shell silently drops it.
- Commit the code before running the benchmark, so the recorded commit hash matches the code that produced the numbers.

Static analysis. CI runs all of this, in the `static-analysis` job:

```bash
clang-format --dry-run --Werror <files>          # pinned: clang-format 23.1.2 (PyPI wheel); use CI's exact list:
#   python3 -c "import yaml,glob; r=[s for s in yaml.safe_load(open('.github/workflows/ci.yml'))['jobs']['static-analysis']['steps'] if s.get('name')=='clang-format'][0]['run']; print(' '.join(sorted({f for p in r.split('--Werror',1)[1].split() for f in glob.glob(p)})))" | xargs clang-format --dry-run --Werror
clang-tidy --quiet src/*.c -- -std=c99 -Iinclude -Isrc [-DKF_USE_DOUBLE | "-DKF_SPECIALIZE=KF_SIZE(4,2)"]   # pinned: 22.1.8
cppcheck --std=c99 --enable=warning,style,performance,portability --inline-suppr --error-exitcode=1 \
  --suppress=missingIncludeSystem -Iinclude -Isrc -UKF_SPECIALIZE src/
python3 scripts/check_misra.py                    # enforced MISRA C:2012 subset
```

- `.clang-tidy` lists each disabled check with the reason. All enabled checks are errors.
- cppcheck's four `uninitvar` hits in `kf_linear.c` are false positives (arrays filled by run-time-bounded loops). They are suppressed inline, with the reason. Zero-initialising instead would add a `memset` to every step.
- MISRA:
  - Homebrew's `cppcheck --addon=misra` fails silently. `check_misra.py` therefore runs `misra.py` directly on dump files, in a temp copy of `src/`.
  - Everything is enforced except Advisory 12.1, 15.5 and 20.5.
  - Rule 21.15 has one documented deviation: the `memcpy` bit-read in `kfi_all_finite`, marked `misra-c2012-21.15 deviation`.
  - On a Mac, clang-tidy needs `-isysroot $SDKROOT`.

Embedded benchmark (STM32F405 / Cortex-M4F, emulated):

```bash
# Arm GNU Toolchain 14.2.Rel1 lives in ~/.local/opt (unpacked tarball, no sudo); QEMU comes from Homebrew.
export ARM_TOOLCHAIN_DIR=$HOME/.local/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi
cmake -S embedded -B build-fw -DCMAKE_TOOLCHAIN_FILE=$PWD/embedded/arm-none-eabi.cmake   # the toolchain path must be absolute
cmake --build build-fw -j 8
python3 scripts/run_embedded.py build-fw >> results/results.csv   # needs SDKROOT set, to build the QEMU plugin
```

- `embedded/` is a standalone CMake project, separate from the main one. It contains:
  - the startup code, linker script and semihosting;
  - one adapter per library (`fw_<lib>.c`), behind the `fw_filter.h` interface;
  - `fw_empty.c`, the harness on its own, used as the baseline.
- Each library is built twice, as `fw_<lib>_1000.elf` and `fw_<lib>_0.elf`. The two differ only in a volatile step count.
- `scripts/run_embedded.py` runs both images on QEMU's `netduinoplus2` board with the `embedded/qemu/icount.c` TCG plugin:
  - `instructions_per_step` is the difference in instruction count, divided by 1,000.
  - Flash and RAM come from `arm-none-eabi-size` on the 1000-step image. `stack_bytes` comes from the painted-stack high-water mark.
- **It is instructions, not cycles.** QEMU doesn't model the DWT counter, pipeline timing or flash wait states, so the guide's `cycles_per_step` needs a real board.
- Two firmware targets:
  - `fw_*`: STM32F405, Cortex-M4F, QEMU `netduinoplus2`.
  - `fw_m3_*`: STM32F205, Cortex-M3 with no FPU (software float), QEMU `netduino2`.

  `run_embedded.py` keeps their rows apart with a `@m3` suffix in the precision column. Fixed-point rows use precision `q<F>`.
- `fw_data.h` holds both float and Q-format (`s2_fx_*`) copies of S2. Its arrays are `static`, so each translation unit has its own copy: never compare pointers into them across files. The fixed-point adapter keeps its own step counter for that reason.
- Firmware is built with `KF_MAX_STATE=4` and `KF_MAX_MEAS=2`, the same as TinyEKF's fixed 4×2.
- GCC with `-std=c99` doesn't fuse multiply-adds, so the specialized and generic firmware are bit-identical, unlike on desktop clang.

Speed regression gate (guide Step 7). This is the CI job `benchmarks`, on `ubuntu-24.04` with gcc:
- **What it measures:** `bench/regress` runs one kalman-c runner on N steps inside `regress_loop()`. `scripts/check_regression.py` counts instructions with valgrind callgrind (`--toggle-collect=regress_loop`), so file parsing and setup are excluded. Ten cases cover the KF, specialized KF, EKF, UKF, UD and fixed point.
- **When it fails:** a case more than 10% above `results/regression-baseline.json` fails the job. Counts are deterministic: two runs on separate CI machines matched exactly. A deliberate slowdown in `kf_core_predict` failed exactly the six KF/EKF cases.
- **Wall-clock timings are deliberately not gated:** they vary 10–20% on shared runners.
- **Recording a baseline:** it can't be done on the Mac, which has no valgrind. With no baseline file, the job records one, uploads it as an artifact and fails, as a bootstrap step.
- **The report:** `scripts/make_comparison_report.py` builds `comparison.md` from the regression table and every `[report]` line in the `ctest -V` log. It's uploaded as the `comparison-report` artifact and shown in the job summary.

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
- The benchmark scenarios S1–S5 are the frozen files in `tests/vectors/`. The guide wants them in a test-vectors submodule shared by the C, C++, Python and Rust repos. That shared repo doesn't exist yet, so the folder is laid out to become that submodule unchanged.
  - `scripts/gen_vectors.py` generates them deterministically. It documents the file format, which is plain text with `kalman-vectors 1` at the top.
  - Treat them as frozen. Don't regenerate them or change them to improve results.
  - The `tests/scenarios/` generators are for tests, not for the published numbers.
- S4 stores `data zero`. A linear KF's covariance doesn't depend on z, so stability depends only on the model. R = 1e-8 was chosen because TinyEKF survives it; it fails at 1e-9, and picking that would be cherry-picking.
- The benchmark lives in `bench/`. Each runner in `bench/runners/` shares the signature `int run_X(const scenario *, run_result *)`.
  - With `measure = 0` a runner does only predict and update, for timing. With `measure = 1` it collects RMSE, NEES, and positive-definiteness through `metrics.c`.
  - TinyEKF is built once per dimension (`tinyekf_{2x1,4x2,15x6}.c`) from `tinyekf_template.h`, and uses `ekf_t` directly.
  - The bench reuses `tests/baselines/`. It doesn't need the test targets.

# kalman-c

Kalman filters for microcontrollers, in plain C99: no heap, no surprises, and numbers to back it up.

> **Status:** v0.1.0. The linear KF, EKF and UKF are implemented and tested against TinyEKF and a textbook filter, in `float` and `double`. The square-root variant and fixed-point support are planned; see `kalman-c-repo-guide.md`.

## Features

- **No dynamic allocation.** All state lives in a caller-owned `kf_state`. A test checks that the library references no heap function.
- **Numerically robust.** The Joseph-form covariance update and Cholesky solves keep P symmetric positive-definite. On an ill-conditioned problem, the textbook filter fails at its first step in `float`; kalman-c runs 1,000,000 steps.
- **Linear KF, EKF and UKF** share one state struct and one calling pattern.
- **Errors are returned, never silent.** NaN or Inf inputs, a singular innovation covariance, and failing model callbacks all return an error code, and leave the filter state byte-for-byte unchanged.
- **Configurable at compile time:** `float` or `double`, the maximum sizes, and optional constant-size specializations (`KF_SPECIALIZE`) that match TinyEKF's speed.
- **Diagnostics.** The normalized innovation squared (NIS) after every update. Monte Carlo tests check NIS and NEES against their chi-squared bounds.
- **Checked:** clang-tidy, cppcheck, and an enforced subset of MISRA C:2012 (see `scripts/check_misra.py`).

## Quick start

```c
#include <stdio.h>
#include "kalman/kalman.h"

int main(void) {
    const kf_real F[4] = {1, 1, 0, 1}; /* constant velocity, dt = 1 */
    const kf_real H[2] = {1, 0};       /* measure position only */
    const kf_real z[3][1] = {{1.1f}, {1.9f}, {3.2f}};
    kf_state kf;

    kf_init(&kf, 2, 1);            /* 2 states, 1 measurement; zeroes x, P, Q, R */
    kf.P[0] = kf.P[3] = 10;        /* initial uncertainty */
    kf.Q[0] = kf.Q[3] = 0.01f;     /* process noise */
    kf.R[0] = 0.25f;               /* measurement noise */

    for (int k = 0; k < 3; ++k) {
        if (kf_predict(&kf, F) != KF_OK || kf_update(&kf, z[k], H) != KF_OK) {
            return 1; /* the filter state is unchanged on error */
        }
        printf("position %.2f, velocity %.2f\n", (double)kf.x[0], (double)kf.x[1]);
    }
    return 0;
}
```

Matrices are flat, row-major arrays: element (i, j) of an n x n matrix is `A[i * n + j]`. For nonlinear models, see the EKF and UKF in `examples/imu_attitude.c`.

## Using it in your project

- **CMake:** `add_subdirectory(kalman-c)` and link the `kalman` target. Options: `KF_USE_DOUBLE`, `KF_MAX_STATE`, `KF_MAX_MEAS` and `KF_SPECIALIZE`.
- **Single header:** download `kalman_c.h` from the release. Include it everywhere, and in exactly one `.c` file `#define KALMAN_C_IMPLEMENTATION` before including it.
- **PlatformIO:** the repository includes a `library.json`.

Size the library for your problem with `KF_MAX_STATE` and `KF_MAX_MEAS`. They set the size of `kf_state` and of the stack scratch space, so the defaults (12 and 6) waste RAM on a 4-state filter.

## Examples and documentation

- `examples/constant_velocity.c`: 1D tracking with the linear KF.
- `examples/imu_attitude.c`: roll and pitch from a gyroscope and an accelerometer, with both the EKF and the UKF.
- API reference: run `doxygen`, then open `build-docs/html/index.html`.

## Build and test

The tests compare against [TinyEKF](https://github.com/simondlevy/TinyEKF), included as a git submodule. Clone with `--recursive`, or run `git submodule update --init` after cloning.

```bash
cmake -B build                  # add -DKF_USE_DOUBLE=ON for double precision
cmake --build build
ctest --test-dir build --output-on-failure
```

## Benchmarks

Every library runs the same frozen scenarios from `tests/vectors/`, with the same precision, initial state and noise matrices. Only predict+update is timed, as the median of 30 runs after a warm-up.

| ID | Scenario | State / measurement | Steps |
|---|---|---|---|
| S1 | 1D constant velocity | 2 / 1 | 10,000 |
| S2 | 2D constant velocity | 4 / 2 | 10,000 |
| S3 | Range-bearing tracking (EKF, UKF) | 4 / 2 | 500 × 200 trials |
| S4 | Ill-conditioned: tiny R, large Q | 4 / 2 | 1,000,000 |
| S5 | 15-state INS error-state | 15 / 6 | 10,000 |

**Measured on a development laptop**: Apple M3 Pro, macOS 27.0.1, Apple clang 21.0.0, `-O3`. This is not yet the dedicated, frequency-locked machine the published numbers should come from. Repeated runs agree within about 5%, but one run was 2× slower, so treat the timings as indicative.

How to read the table:

- **kalman-c** is the default build. **kalman-c-specialized** is built with `KF_SPECIALIZE` listing the scenario sizes, which compiles constant-size copies of the KF/EKF core (see `kf_config.h`). TinyEKF always fixes its sizes at compile time.
- **Ours vs best other** compares the default build with the best baseline, and is above 1.00x when kalman-c is better.
- **max_abs_diff** is each baseline's largest difference from kalman-c's estimates.
- **steps_to_failure** is the scenario length if the covariance never lost positive-definiteness.
- The S2 `instructions_per_step`, `flash_bytes`, `ram_bytes` and `stack_bytes` rows come from Cortex-M4F firmware. It's built for an STM32F405 and run under QEMU, not on a physical board. See "Embedded" below.

What it shows so far:

- **Accuracy:** kalman-c matches the baselines on every scenario.
- **Stability:** on S4 in float32, the textbook filter fails at the first step. kalman-c and TinyEKF both survive 1,000,000 steps. TinyEKF fails at R = 1e-9 (see `test_stability`), but the frozen scenario uses 1e-8.
- **Speed:**
  - The specialized build matches TinyEKF on S2 and is within about 12% on S5, while keeping the Joseph form, input validation, and leaving the state unchanged on error.
  - On the smallest problem (S1), fixed per-step costs still leave it behind: 1.3× in float32 and 1.8× in float64.
  - The default build is 2–3× slower than TinyEKF, because its sizes are only known at run time.

**Embedded** (STM32F405 / Cortex-M4F, emulated with QEMU 11.1.2 `netduinoplus2`, built with `arm-none-eabi-gcc` 14.2.1 using `-O2 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`):

- Each image runs the first 1,000 steps of S2, with the sizes set to the problem (`KF_MAX_STATE=4`, `KF_MAX_MEAS=2`).
- `instructions_per_step` is an exact count of the instructions QEMU executes per predict+update. **It is not cycles**: QEMU doesn't model the pipeline, flash wait states or the DWT counter. The guide's `cycles_per_step` still needs a real board.
- Flash includes the 9,068-byte harness and scenario data that every image shares. RAM is `data + bss`; `stack_bytes` is the measured peak stack use.
- Results:
  - The specialized build runs the fewest instructions: 3,602 per step, against 4,764 for the textbook filter and 6,608 for TinyEKF. It costs about 1.7 KB more flash than the default build.
  - TinyEKF's count includes software double-precision math. It calls `sqrt()` on a double, and the M4F has only a single-precision FPU. kalman-c links no double-precision code.
  - kalman-c also uses the least stack of the three filters: 512 bytes, against 688 for naive and 740 for TinyEKF.

<!-- BENCH:START -->
| Scenario | Filter | Precision | Metric | kalman-c | kalman-c-specialized | naive | tinyekf | Ours vs best other |
|---|---|---|---|---|---|---|---|---|
| S1 | KF | float32 | max_abs_diff (state) | n/a | n/a | 0.0006236 | 0.0006274 | n/a |
| S1 | KF | float32 | rmse (state) | 0.2635 | 0.2635 | 0.2635 | 0.2635 | 1.00x |
| S1 | KF | float32 | time_per_step (ns) | 94.5 | 33.5 | 55.8 | 26.4 | 0.28x |
| S1 | KF | float64 | max_abs_diff (state) | n/a | n/a | 1.364e-12 | 9.095e-13 | n/a |
| S1 | KF | float64 | rmse (state) | 0.2635 | 0.2635 | 0.2635 | 0.2635 | 1.00x |
| S1 | KF | float64 | time_per_step (ns) | 104.1 | 49.7 | 65.7 | 27.8 | 0.27x |
| S2 | KF | float32 | flash_bytes (bytes) | 1.231e+04 | 1.4e+04 | 1.072e+04 | 1.372e+04 | 0.87x |
| S2 | KF | float32 | instructions_per_step (instructions) | 5527 | 3602 | 4764 | 6608 | 0.86x |
| S2 | KF | float32 | max_abs_diff (state) | n/a | n/a | 0.0005245 | 0.0006514 | n/a |
| S2 | KF | float32 | ram_bytes (bytes) | 176 | 176 | 268 | 164 | 0.93x |
| S2 | KF | float32 | rmse (state) | 0.266 | 0.266 | 0.266 | 0.266 | 1.00x |
| S2 | KF | float32 | stack_bytes (bytes) | 512 | 504 | 688 | 740 | 1.34x |
| S2 | KF | float32 | time_per_step (ns) | 192.9 | 54.4 | 192.5 | 57.2 | 0.30x |
| S2 | KF | float64 | max_abs_diff (state) | n/a | n/a | 3.258e-12 | 2.832e-12 | n/a |
| S2 | KF | float64 | rmse (state) | 0.266 | 0.266 | 0.266 | 0.266 | 1.00x |
| S2 | KF | float64 | time_per_step (ns) | 192.4 | 69.1 | 196.7 | 56.5 | 0.29x |
| S3 | EKF | float32 | nees (-) | 3.958 | 3.958 | n/a | 3.958 | – |
| S3 | EKF | float32 | rmse (state) | 0.2144 | 0.2144 | n/a | 0.2144 | 1.00x |
| S3 | EKF | float64 | nees (-) | 3.958 | 3.958 | n/a | 3.958 | – |
| S3 | EKF | float64 | rmse (state) | 0.2144 | 0.2144 | n/a | 0.2144 | 1.00x |
| S3 | UKF | float32 | nees (-) | 3.958 | 3.958 | n/a | n/a | n/a |
| S3 | UKF | float32 | rmse (state) | 0.2144 | 0.2144 | n/a | n/a | n/a |
| S3 | UKF | float64 | nees (-) | 3.958 | 3.958 | n/a | n/a | n/a |
| S3 | UKF | float64 | rmse (state) | 0.2144 | 0.2144 | n/a | n/a | n/a |
| S4 | KF | float32 | steps_to_failure (steps) | 1e+06 | 1e+06 | 0 | 1e+06 | 1.00x |
| S4 | KF | float64 | steps_to_failure (steps) | 1e+06 | 1e+06 | 1e+06 | 1e+06 | 1.00x |
| S5 | KF | float32 | max_abs_diff (state) | n/a | n/a | 0.0009766 | 0.0009766 | n/a |
| S5 | KF | float32 | rmse (state) | 0.06123 | 0.06123 | 0.06123 | 0.06123 | 1.00x |
| S5 | KF | float32 | time_per_step (ns) | 4748 | 2263 | 5417 | 2019 | 0.43x |
| S5 | KF | float64 | max_abs_diff (state) | n/a | n/a | 1.364e-12 | 1.364e-12 | n/a |
| S5 | KF | float64 | rmse (state) | 0.06123 | 0.06123 | 0.06123 | 0.06123 | 1.00x |
| S5 | KF | float64 | time_per_step (ns) | 4400 | 2338 | 5100 | 2113 | 0.48x |
<!-- BENCH:END -->

## License

MIT

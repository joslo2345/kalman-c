# kalman-c

An embedded-friendly Kalman filter library in C99.

> **Status:** in development. The linear KF, EKF and UKF are implemented and tested against TinyEKF and a textbook filter. Benchmarks, the square-root variant and fixed-point support are still to come. See `kalman-c-repo-guide.md` for the full plan.

## Goals

- **No dynamic allocation.** All memory is static or caller-provided, so the library runs on microcontrollers.
- **Numerical robustness.** Covariance updates use the Joseph form or a square-root (Cholesky/UD) form.
- **Multiple filter types.** Linear KF, EKF and UKF behind a consistent API.
- **Configurable precision.** Choose `float` or `double` at compile time, with optional fixed-point later.
- **Diagnostics.** NIS/NEES for checking filter consistency.

## Build and test

The tests compare against [TinyEKF](https://github.com/simondlevy/TinyEKF), included as a git submodule. Clone with `--recursive`, or run `git submodule update --init` after cloning.

```bash
cmake -B build                  # add -DKF_USE_DOUBLE=ON for double precision
cmake --build build
ctest --test-dir build --output-on-failure
```

Override the maximum dimensions at compile time with `-DKF_MAX_STATE=<n>` and `-DKF_MAX_MEAS=<m>` in your C flags.

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

- **Ours vs best other** is above 1.00x when kalman-c is better.
- **max_abs_diff** is each baseline's largest difference from kalman-c's estimates.
- **steps_to_failure** is the scenario length if the covariance never lost positive-definiteness.
- Embedded metrics (cycles, flash, RAM on a Cortex-M4) haven't been measured yet.

What it shows so far:

- **Accuracy:** kalman-c matches the baselines on every scenario.
- **Stability:** on S4 in float32, the textbook filter fails at the first step. kalman-c and TinyEKF both survive 1,000,000 steps. TinyEKF fails at R = 1e-9 (see `test_stability`), but the frozen scenario uses 1e-8.
- **Speed:** kalman-c is currently **4–5× slower than TinyEKF** and about 1.5–2× slower than the textbook filter. Closing that gap is the next piece of work.

<!-- BENCH:START -->
| Scenario | Filter | Precision | Metric | kalman-c | naive | tinyekf | Ours vs best other |
|---|---|---|---|---|---|---|---|
| S1 | KF | float32 | max_abs_diff (state) | n/a | 0.0006256 | 0.0006294 | n/a |
| S1 | KF | float32 | rmse (state) | 0.2635 | 0.2635 | 0.2635 | 1.00x |
| S1 | KF | float32 | time_per_step (ns) | 130.4 | 59.5 | 29.9 | 0.23x |
| S1 | KF | float64 | max_abs_diff (state) | n/a | 9.828e-13 | 3.997e-15 | n/a |
| S1 | KF | float64 | rmse (state) | 0.2635 | 0.2635 | 0.2635 | 1.00x |
| S1 | KF | float64 | time_per_step (ns) | 133.6 | 61.3 | 30.7 | 0.23x |
| S2 | KF | float32 | max_abs_diff (state) | n/a | 0.0009766 | 0.0009766 | n/a |
| S2 | KF | float32 | rmse (state) | 0.266 | 0.266 | 0.266 | 1.00x |
| S2 | KF | float32 | time_per_step (ns) | 347.4 | 196.4 | 63.1 | 0.18x |
| S2 | KF | float64 | max_abs_diff (state) | n/a | 2.368e-12 | 5.684e-14 | n/a |
| S2 | KF | float64 | rmse (state) | 0.266 | 0.266 | 0.266 | 1.00x |
| S2 | KF | float64 | time_per_step (ns) | 316.9 | 205.4 | 75.4 | 0.24x |
| S3 | EKF | float32 | nees (-) | 3.958 | n/a | 3.958 | – |
| S3 | EKF | float32 | rmse (state) | 0.2144 | n/a | 0.2144 | 1.00x |
| S3 | EKF | float64 | nees (-) | 3.958 | n/a | 3.958 | – |
| S3 | EKF | float64 | rmse (state) | 0.2144 | n/a | 0.2144 | 1.00x |
| S3 | UKF | float32 | nees (-) | 3.958 | n/a | n/a | n/a |
| S3 | UKF | float32 | rmse (state) | 0.2144 | n/a | n/a | n/a |
| S3 | UKF | float64 | nees (-) | 3.958 | n/a | n/a | n/a |
| S3 | UKF | float64 | rmse (state) | 0.2144 | n/a | n/a | n/a |
| S4 | KF | float32 | steps_to_failure (steps) | 1e+06 | 0 | 1e+06 | 1.00x |
| S4 | KF | float64 | steps_to_failure (steps) | 1e+06 | 1e+06 | 1e+06 | 1.00x |
| S5 | KF | float32 | max_abs_diff (state) | n/a | 4.578e-05 | 4.578e-05 | n/a |
| S5 | KF | float32 | rmse (state) | 0.06123 | 0.06123 | 0.06123 | 1.00x |
| S5 | KF | float32 | time_per_step (ns) | 8000 | 5340 | 2034 | 0.25x |
| S5 | KF | float64 | max_abs_diff (state) | n/a | 3.553e-15 | 3.553e-15 | n/a |
| S5 | KF | float64 | rmse (state) | 0.06123 | 0.06123 | 0.06123 | 1.00x |
| S5 | KF | float64 | time_per_step (ns) | 7956 | 5287 | 2156 | 0.27x |
<!-- BENCH:END -->

## License

MIT

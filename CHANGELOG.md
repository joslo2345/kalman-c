# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/). Before 1.0.0, minor versions may
change the API.

## [Unreleased]

### Added

- A fixed-point EKF (`kf_fx_ekf_predict`, `kf_fx_ekf_update`) with Q-format model callbacks.
  - It shares the fixed-point KF's predict and update core, so with linear callbacks it is bit-identical to the KF.
  - On 300 range-bearing steps it tracks the float EKF to 4.9e-4 m.
- `scripts/profile_firmware.py` and the `embedded/qemu/tbprof.c` QEMU plugin: an instruction profile of a firmware image by function and source line.

### Changed

- The fixed-point filter needs about 10% fewer instructions per step: 12,576 instead of 13,996 on the Cortex-M3, and 12,503 instead of 14,328 on the M4F, with about 400 bytes less flash.
  - Profiling showed that checking every 64-bit multiply-add for overflow cost almost half the instructions.
  - Dot products now drop a few guard bits per product, so their sums cannot overflow, and check once at the end.
  - The precision cost is at most 2^-32 in real terms, far below the 2^-20 rounding step.
- Stack use of the fixed-point filter rose by 48 bytes (544 to 592 on the M4F), because the KF entry points and the shared core are separate frames.
- Firmware images are built with `-g`. This adds debug sections only; code, flash and RAM sizes are unchanged.

## [0.2.0] - 2026-10-06

Every step of the design plan is now implemented: a square-root (UD) filter for badly conditioned problems, and a fixed-point filter for microcontrollers without an FPU.

### Added

- Square-root Kalman filter in UD form (`kf_sqrt.h`: `kf_sr_*`), linear and extended. It uses Bierman's measurement update, applied to measurements whitened by R's UD factors, and Thornton's weighted Gram-Schmidt predict. In `float`, at R = 1e-10, the position variance stays within 2.8e-7 of a float64 reference over 100,000 steps; the Joseph-form filter drifts to 1.4e-4.
- `test_sqrt_accuracy`, which checks covariance accuracy against a float64 reference computed inside the test.
- An `SRKF` row in the desktop and Cortex-M4F benchmarks.
- A fixed-point linear Kalman filter (`kf_fixed.h`: `kf_fx_*`).
  - It uses Q-format `int32_t` with `KF_FX_FRAC` fractional bits (default 20; a CMake option), 64-bit exact dot products that are rounded once, symmetric rounding, an integer square root, and the same O(n²m) Joseph form.
  - Overflow returns the new `KF_ERR_OVERFLOW` and leaves the state unchanged.
  - `kf_fx_from_double` and `kf_fx_to_double` convert values, with range checks.
- Cortex-M3 firmware (STM32F205, no FPU, QEMU `netduino2`) for kalman-c in `float` and fixed point and for TinyEKF, plus a fixed-point image on the M4F.
- A fixed-point benchmark row on the desktop (`q18`), and a CI job with `KF_FX_FRAC=16`.

### Changed

- The KF/EKF predict and update cores re-check `n` and `m` after model callbacks run, since `ctx` may alias the filter state.
- The benchmark harness reports runners that fail instead of skipping them silently.
- The firmware's `filter_x` returns raw 32-bit state words, so float and Q-format states print alike.
- A restyled README: a logo, badges, news, feature highlights, a headline comparison table, and light and dark benchmark charts generated from `results/results.csv` by `scripts/make_readme_charts.py`.

## [0.1.0] - 2026-10-05

First release.

### Added

- Linear Kalman filter (`kf_init`, `kf_predict`, `kf_update`) with the Joseph-form covariance update, evaluated in O(n²m).
- Extended Kalman filter (`kf_ekf_predict`, `kf_ekf_update`) with user callbacks that return the model value and its Jacobian, plus a `ctx` pointer.
- Unscented Kalman filter (`kf_ukf_predict`, `kf_ukf_update`) with additive noise, Cholesky sigma points, and `kf_ukf_params` (NULL selects alpha = 1, beta = 2, kappa = 0).
- Small matrix routines (`kf_mat_*`, `kf_cholesky*`, `kf_solve_lower*`, and the checks `kf_is_symmetric`, `kf_cholesky_ok` and `kf_all_finite`).
- Compile-time configuration: `KF_USE_DOUBLE`, `KF_MAX_STATE`, `KF_MAX_MEAS`, and `KF_SPECIALIZE` for constant-size copies of the KF/EKF core.
- Error codes for NULL or invalid arguments, NaN/Inf inputs, covariances that aren't positive-definite, and failing model callbacks. Every error leaves the filter state byte-for-byte unchanged.
- The normalized innovation squared (NIS) after every update.
- A single-header distribution (`kalman_c.h`), an Arduino library zip, and a PlatformIO `library.json`.
- Tests:
  - unit, equivalence and error tests;
  - 1,000,000-step `float` stability;
  - Monte Carlo NIS/NEES consistency;
  - a no-heap-allocation check.
- The frozen benchmark scenarios S1–S5, with desktop and emulated Cortex-M4F benchmarks against TinyEKF and a textbook filter.
- Static analysis: clang-tidy, cppcheck, and an enforced subset of MISRA C:2012.

[Unreleased]: https://github.com/joslo2345/kalman-c/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/joslo2345/kalman-c/releases/tag/v0.2.0
[0.1.0]: https://github.com/joslo2345/kalman-c/releases/tag/v0.1.0

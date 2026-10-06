# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/). Before 1.0.0, minor versions may
change the API.

## [Unreleased]

### Added

- Square-root Kalman filter in UD form (`kf_sqrt.h`: `kf_sr_*`), linear and extended. It uses Bierman's measurement update, applied to measurements whitened by R's UD factors, and Thornton's weighted Gram-Schmidt predict. In `float`, at R = 1e-10, the position variance stays within 2.8e-7 of a float64 reference over 100,000 steps; the Joseph-form filter drifts to 1.4e-4.
- `test_sqrt_accuracy`, which checks covariance accuracy against a float64 reference computed inside the test.
- An `SRKF` row in the desktop and Cortex-M4F benchmarks.

### Changed

- The KF/EKF predict and update cores re-check `n` and `m` after model callbacks run, since `ctx` may alias the filter state.
- The benchmark harness reports runners that fail instead of skipping them silently.

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

[0.1.0]: https://github.com/joslo2345/kalman-c/releases/tag/v0.1.0

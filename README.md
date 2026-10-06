# kalman-c

An embedded-friendly Kalman filter library in C99.

> **Status:** early scaffolding. The build, configuration, and test setup are in place. The filters themselves aren't implemented yet. See `kalman-c-repo-guide.md` for the full plan.

## Goals

- **No dynamic allocation.** All memory is static or caller-provided, so the library runs on microcontrollers.
- **Numerical robustness.** Covariance updates use the Joseph form or a square-root (Cholesky/UD) form.
- **Multiple filter types.** Linear KF, EKF and UKF behind a consistent API.
- **Configurable precision.** Choose `float` or `double` at compile time, with optional fixed-point later.
- **Diagnostics.** NIS/NEES for checking filter consistency.

## Build and test

```bash
cmake -B build                  # add -DKF_USE_DOUBLE=ON for double precision
cmake --build build
ctest --test-dir build --output-on-failure
```

Override the maximum dimensions at compile time with `-DKF_MAX_STATE=<n>` and `-DKF_MAX_MEAS=<m>` in your C flags.

## Benchmarks

<!-- BENCH:START -->
No results yet.
<!-- BENCH:END -->

## License

MIT

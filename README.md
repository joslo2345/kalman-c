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

<!-- BENCH:START -->
No results yet.
<!-- BENCH:END -->

## License

MIT

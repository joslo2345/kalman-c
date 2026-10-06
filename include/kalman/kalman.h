#ifndef KALMAN_H
#define KALMAN_H

/**
 * @file kalman.h
 * @brief Umbrella header for the kalman-c public API.
 *
 * @mainpage kalman-c
 *
 * An embedded-friendly Kalman filter library in C99:
 * - **No dynamic allocation.** All state lives in a caller-owned ::kf_state.
 * - **Numerically robust.** Joseph-form covariance updates, Cholesky solves, no explicit inverses.
 * - **Linear KF, EKF and UKF** behind one state struct (kf_linear.h, kf_ekf.h, kf_ukf.h).
 * - **Configurable precision** (`float` or `double`) and sizes at compile time (kf_config.h).
 * - **Errors are returned, never silent**, and a failed step leaves the filter unchanged.
 * - **Diagnostics:** the normalized innovation squared (NIS) after every update.
 *
 * Matrices are flat, row-major ::kf_real arrays: element (i, j) of an r x c
 * matrix is `A[i * c + j]`.
 */

#include "kalman/kf_config.h"
#include "kalman/kf_ekf.h"
#include "kalman/kf_linalg.h"
#include "kalman/kf_linear.h"
#include "kalman/kf_types.h"
#include "kalman/kf_ukf.h"

#endif

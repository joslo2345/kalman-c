#ifndef KF_TYPES_H
#define KF_TYPES_H

/**
 * @file kf_types.h
 * @brief Status codes and the filter state shared by every filter type.
 */

#include "kalman/kf_config.h"

/** Every API function returns one of these. On error, the filter state is left unchanged. */
typedef enum {
    KF_OK = 0,                         /**< Success. */
    KF_ERR_INVALID_INPUT = -1,         /**< NULL pointer, bad dimension, or NaN/Inf input. */
    KF_ERR_NOT_POSITIVE_DEFINITE = -2, /**< A covariance (P or S) is not positive-definite. */
    KF_ERR_MODEL_FAILED = -3           /**< A user-supplied model callback returned non-zero. */
} kf_status;

/**
 * Filter state, owned by the caller. One struct serves the KF, EKF and UKF.
 *
 * Initialize it with kf_init(), then set the fields below the sizes directly.
 * Matrices are flat, row-major, and use only their leading n x n (or m x m)
 * elements: element (i, j) of P is `P[i * n + j]`.
 */
typedef struct {
    int n;                                  /**< State size, 1..KF_MAX_STATE. */
    int m;                                  /**< Measurement size, 1..KF_MAX_MEAS. */
    kf_real x[KF_MAX_STATE];                /**< State estimate (n). */
    kf_real P[KF_MAX_STATE * KF_MAX_STATE]; /**< State covariance (n x n), symmetric positive-definite. */
    kf_real Q[KF_MAX_STATE * KF_MAX_STATE]; /**< Process noise covariance (n x n). */
    kf_real R[KF_MAX_MEAS * KF_MAX_MEAS];   /**< Measurement noise covariance (m x m). */
    kf_real nis; /**< Normalized innovation squared from the last update; averages m when consistent. */
} kf_state;

#endif

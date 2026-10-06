#ifndef KF_SQRT_H
#define KF_SQRT_H

/**
 * @file kf_sqrt.h
 * @brief Square-root (UD) Kalman filter: linear and extended.
 *
 * Instead of P, the filter propagates the factors of P = U diag(D) U^T, with
 * U unit upper triangular: Bierman's scalar measurement update and Thornton's
 * weighted Gram-Schmidt predict. Every new variance comes from ratios and sums
 * of non-negative terms, never from a subtraction, so P cannot become
 * indefinite and keeps its accuracy when it is badly conditioned. With precise
 * sensors in float, P's small variances stay accurate where the Joseph-form
 * filter in kf_linear.h loses digits. Each step costs more than that filter.
 *
 * Set the covariances through kf_sr_set_P(), kf_sr_set_Q() and kf_sr_set_R(),
 * which factor them, and read P back with kf_sr_get_P(). Measurements are
 * processed one component at a time after whitening by R's factors, so any
 * positive-definite R is allowed. As everywhere in this library, a failed
 * call leaves the state unchanged.
 */

#include "kalman/kf_ekf.h"
#include "kalman/kf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Square-root (UD) filter state, owned by the caller. Matrices are flat, row-major. */
typedef struct {
    int n;                                   /**< State size, 1..KF_MAX_STATE. */
    int m;                                   /**< Measurement size, 1..KF_MAX_MEAS. */
    kf_real x[KF_MAX_STATE];                 /**< State estimate (n). */
    kf_real U[KF_MAX_STATE * KF_MAX_STATE];  /**< Unit upper-triangular factor of P (n x n). */
    kf_real D[KF_MAX_STATE];                 /**< Diagonal factor of P (n): P = U diag(D) U^T. */
    kf_real Uq[KF_MAX_STATE * KF_MAX_STATE]; /**< Unit upper-triangular factor of Q (n x n). */
    kf_real Dq[KF_MAX_STATE];                /**< Diagonal factor of Q (n). */
    kf_real Ur[KF_MAX_MEAS * KF_MAX_MEAS];   /**< Unit upper-triangular factor of R (m x m). */
    kf_real Dr[KF_MAX_MEAS];                 /**< Diagonal factor of R (m), all positive. */
    kf_real nis; /**< Normalized innovation squared from the last update. */
} kf_sr_state;

/**
 * Initialize a square-root filter: set the sizes and zero everything else.
 * @return KF_OK, or KF_ERR_INVALID_INPUT for a NULL sr or an out-of-range size.
 */
int kf_sr_init(kf_sr_state *sr, int n, int m);

/**
 * Set the state covariance P (n x n), stored as its UD factors.
 * @return KF_OK, KF_ERR_INVALID_INPUT, or KF_ERR_NOT_POSITIVE_DEFINITE if P is
 *         not positive-definite.
 */
int kf_sr_set_P(kf_sr_state *sr, const kf_real *P);

/**
 * Set the process noise Q (n x n), stored as its UD factors. Q may be only
 * positive semi-definite (for example zero for some states).
 * @return KF_OK, KF_ERR_INVALID_INPUT, or KF_ERR_NOT_POSITIVE_DEFINITE if Q is indefinite.
 */
int kf_sr_set_Q(kf_sr_state *sr, const kf_real *Q);

/**
 * Set the measurement noise R (m x m), stored as its UD factors.
 * @return KF_OK, KF_ERR_INVALID_INPUT, or KF_ERR_NOT_POSITIVE_DEFINITE if R is
 *         not positive-definite.
 */
int kf_sr_set_R(kf_sr_state *sr, const kf_real *R);

/**
 * Write P = U diag(D) U^T (n x n) to P.
 * @return KF_OK, or KF_ERR_INVALID_INPUT for NULL arguments or invalid sizes.
 */
int kf_sr_get_P(const kf_sr_state *sr, kf_real *P);

/**
 * Predict: x = F x, and the factors of F P F^T + Q.
 * @return KF_OK, or KF_ERR_INVALID_INPUT for NULL arguments, invalid sizes or
 *         non-finite values. On error the state is unchanged.
 */
int kf_sr_predict(kf_sr_state *sr, const kf_real *F);

/**
 * Update with a measurement z (m) and measurement matrix H (m x n), and set nis.
 * @return KF_OK, KF_ERR_INVALID_INPUT, or KF_ERR_NOT_POSITIVE_DEFINITE when the
 *         innovation covariance is singular. On error the state is unchanged.
 */
int kf_sr_update(kf_sr_state *sr, const kf_real *z, const kf_real *H);

/**
 * Extended predict with the EKF transition callback (see kf_ekf.h).
 * @return As kf_ekf_predict().
 */
int kf_sr_ekf_predict(kf_sr_state *sr, kf_ekf_transition_fn f, void *ctx);

/**
 * Extended update with the EKF measurement callback (see kf_ekf.h).
 * @return As kf_ekf_update().
 */
int kf_sr_ekf_update(kf_sr_state *sr, const kf_real *z, kf_ekf_measurement_fn h, void *ctx);

#ifdef __cplusplus
}
#endif

#endif

#ifndef KF_LINEAR_H
#define KF_LINEAR_H

/**
 * @file kf_linear.h
 * @brief Linear Kalman filter.
 *
 * A step is kf_predict() followed by kf_update(). The update uses the Joseph
 * form, P = (I - KH) P (I - KH)^T + K R K^T, which keeps P symmetric
 * positive-definite where the textbook (I - KH) P update loses it.
 */

#include "kalman/kf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize a filter: set the sizes and zero x, P, Q, R and nis.
 *
 * Set x, P, Q and R afterwards; P must be positive-definite before the first update.
 *
 * @param kf  Filter state to initialize.
 * @param n   State size, 1..KF_MAX_STATE.
 * @param m   Measurement size, 1..KF_MAX_MEAS.
 * @return KF_OK, or KF_ERR_INVALID_INPUT for a NULL kf or an out-of-range size
 *         (kf is then left unchanged).
 */
int kf_init(kf_state *kf, int n, int m);

/**
 * Predict: x = F x, P = F P F^T + Q.
 *
 * @param kf  Filter state.
 * @param F   State transition matrix (n x n).
 * @return KF_OK, or KF_ERR_INVALID_INPUT for NULL arguments, invalid sizes or
 *         non-finite values. On error the state is unchanged.
 */
int kf_predict(kf_state *kf, const kf_real *F);

/**
 * Update with a measurement: Joseph-form update, and kf->nis set to the
 * normalized innovation squared.
 *
 * @param kf  Filter state.
 * @param z   Measurement (m).
 * @param H   Measurement matrix (m x n).
 * @return KF_OK; KF_ERR_INVALID_INPUT for NULL arguments, invalid sizes or
 *         non-finite values (including a NaN measurement); or
 *         KF_ERR_NOT_POSITIVE_DEFINITE when the innovation covariance
 *         H P H^T + R is not positive-definite. On error the state is unchanged.
 */
int kf_update(kf_state *kf, const kf_real *z, const kf_real *H);

#ifdef __cplusplus
}
#endif

#endif

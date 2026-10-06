#ifndef KF_EKF_H
#define KF_EKF_H

/**
 * @file kf_ekf.h
 * @brief Extended Kalman filter with user-supplied models and Jacobians.
 *
 * Uses the same ::kf_state, Q and R as the linear filter, and the same
 * Joseph-form update. Each callback evaluates its model and Jacobian
 * together, so shared terms are computed once.
 *
 * Callbacks return 0 on success; any other value aborts the step with
 * KF_ERR_MODEL_FAILED and leaves the filter state unchanged. Output buffers
 * are zeroed before the call, and ctx is passed through untouched.
 */

#include "kalman/kf_types.h"

/**
 * Transition model: x_out (n) = f(x), F_out (n x n) = df/dx at x.
 * @return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_ekf_transition_fn)(kf_real *x_out, kf_real *F_out, const kf_real *x, int n,
                                    void *ctx);

/**
 * Measurement model: z_out (m) = h(x), H_out (m x n) = dh/dx at x.
 * @return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_ekf_measurement_fn)(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m,
                                     void *ctx);

/**
 * Predict: x = f(x), P = F P F^T + Q.
 *
 * @param kf   Filter state.
 * @param f    Transition model and Jacobian.
 * @param ctx  Passed to f unchanged (e.g. a time step or control input).
 * @return KF_OK, KF_ERR_INVALID_INPUT (NULL arguments, invalid sizes,
 *         non-finite model output) or KF_ERR_MODEL_FAILED. On error the
 *         state is unchanged.
 */
int kf_ekf_predict(kf_state *kf, kf_ekf_transition_fn f, void *ctx);

/**
 * Update: y = z - h(x), then the Joseph-form update with H = dh/dx.
 *
 * Angle-valued measurements must be wrapped by the caller (for example by
 * shifting z by 2*pi to be near h(x)) before calling.
 *
 * @param kf   Filter state.
 * @param z    Measurement (m).
 * @param h    Measurement model and Jacobian.
 * @param ctx  Passed to h unchanged.
 * @return KF_OK, KF_ERR_INVALID_INPUT, KF_ERR_NOT_POSITIVE_DEFINITE or
 *         KF_ERR_MODEL_FAILED. On error the state is unchanged.
 */
int kf_ekf_update(kf_state *kf, const kf_real *z, kf_ekf_measurement_fn h, void *ctx);

#endif

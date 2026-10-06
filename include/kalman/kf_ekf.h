#ifndef KF_EKF_H
#define KF_EKF_H

/*
 * Extended Kalman filter. The caller supplies the nonlinear models as callbacks
 * that evaluate the model and its Jacobian together, so shared terms are
 * computed once. Both use the same kf_state, Q and R as the linear filter.
 *
 * Callbacks return 0 on success; any other value aborts the step with
 * KF_ERR_MODEL_FAILED and leaves the filter state unchanged. Output buffers
 * are zeroed before the call. ctx is passed through untouched.
 */

#include "kalman/kf_types.h"

/* x_out (n) = f(x),  F_out (n x n) = df/dx at x */
typedef int (*kf_ekf_transition_fn)(kf_real *x_out, kf_real *F_out, const kf_real *x, int n,
                                    void *ctx);

/* z_out (m) = h(x),  H_out (m x n) = dh/dx at x */
typedef int (*kf_ekf_measurement_fn)(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m,
                                     void *ctx);

/* x = f(x),  P = F P F^T + Q */
int kf_ekf_predict(kf_state *kf, kf_ekf_transition_fn f, void *ctx);

/* y = z - h(x), then the Joseph-form update with H. Angle-valued measurements
 * should be wrapped by the caller (e.g. by shifting z near h(x)) before calling. */
int kf_ekf_update(kf_state *kf, const kf_real *z, kf_ekf_measurement_fn h, void *ctx);

#endif

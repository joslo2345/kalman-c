#ifndef KF_UKF_H
#define KF_UKF_H

/*
 * Unscented Kalman filter with additive process and measurement noise
 * (kf->Q and kf->R). Sigma points come from the Cholesky factor of P, so the
 * caller supplies only the model functions, without Jacobians.
 *
 * Callbacks return 0 on success; any other value aborts the step with
 * KF_ERR_MODEL_FAILED and leaves the filter state unchanged. ctx is passed
 * through untouched.
 */

#include "kalman/kf_types.h"

/* Scaled unscented transform parameters. Passing NULL uses the defaults
 * alpha = 1, beta = 2, kappa = 0. Small alpha (e.g. 1e-3) gives a large
 * negative centre weight, which loses precision in float builds.
 * n + lambda = alpha^2 (n + kappa) must be positive. */
typedef struct {
    kf_real alpha;
    kf_real beta;
    kf_real kappa;
} kf_ukf_params;

/* x_out (n) = f(x) */
typedef int (*kf_ukf_transition_fn)(kf_real *x_out, const kf_real *x, int n, void *ctx);

/* z_out (m) = h(x) */
typedef int (*kf_ukf_measurement_fn)(kf_real *z_out, const kf_real *x, int n, int m, void *ctx);

/* Propagate sigma points through f: x = mean, P = covariance + Q. */
int kf_ukf_predict(kf_state *kf, const kf_ukf_params *params, kf_ukf_transition_fn f, void *ctx);

/* Propagate sigma points through h, then x += K (z - z_hat), P -= K S K^T.
 * Angle-valued measurements should be wrapped by the caller before calling. */
int kf_ukf_update(kf_state *kf, const kf_real *z, const kf_ukf_params *params,
                  kf_ukf_measurement_fn h, void *ctx);

#endif

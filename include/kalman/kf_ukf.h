#ifndef KF_UKF_H
#define KF_UKF_H

/**
 * @file kf_ukf.h
 * @brief Unscented Kalman filter with additive process and measurement noise.
 *
 * Uses kf->Q and kf->R as additive noise. Sigma points come from the Cholesky
 * factor of P, so the caller supplies only the model functions, without
 * Jacobians.
 *
 * Callbacks return 0 on success; any other value aborts the step with
 * KF_ERR_MODEL_FAILED and leaves the filter state unchanged. ctx is passed
 * through untouched.
 */

#include "kalman/kf_types.h"

/**
 * Scaled unscented transform parameters.
 *
 * Passing NULL instead uses alpha = 1, beta = 2, kappa = 0. A small alpha
 * (the textbook 1e-3) gives a large negative centre weight, which loses
 * precision in float builds. alpha^2 (n + kappa) must be positive.
 */
typedef struct {
    kf_real alpha; /**< Sigma point spread. */
    kf_real beta;  /**< Prior knowledge of the distribution; 2 is optimal for Gaussians. */
    kf_real kappa; /**< Secondary scaling; 3 - n matches the Gaussian fourth moment. */
} kf_ukf_params;

/**
 * Transition model: x_out (n) = f(x).
 * @return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_ukf_transition_fn)(kf_real *x_out, const kf_real *x, int n, void *ctx);

/**
 * Measurement model: z_out (m) = h(x).
 * @return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_ukf_measurement_fn)(kf_real *z_out, const kf_real *x, int n, int m, void *ctx);

/**
 * Predict: propagate the 2n+1 sigma points through f; x and P become their
 * weighted mean and covariance, plus Q.
 *
 * @param kf      Filter state.
 * @param params  Unscented transform parameters, or NULL for the defaults.
 * @param f       Transition model.
 * @param ctx     Passed to f unchanged.
 * @return KF_OK, KF_ERR_INVALID_INPUT (NULL arguments, invalid sizes or
 *         parameters, non-finite model output), KF_ERR_NOT_POSITIVE_DEFINITE
 *         (P before or after the step) or KF_ERR_MODEL_FAILED. On error the
 *         state is unchanged.
 */
int kf_ukf_predict(kf_state *kf, const kf_ukf_params *params, kf_ukf_transition_fn f, void *ctx);

/**
 * Update: propagate sigma points through h, then x += K (z - z_hat) and
 * P -= K S K^T, and set kf->nis.
 *
 * Angle-valued measurements must be wrapped by the caller before calling.
 *
 * @param kf      Filter state.
 * @param z       Measurement (m).
 * @param params  Unscented transform parameters, or NULL for the defaults.
 * @param h       Measurement model.
 * @param ctx     Passed to h unchanged.
 * @return KF_OK, KF_ERR_INVALID_INPUT, KF_ERR_NOT_POSITIVE_DEFINITE or
 *         KF_ERR_MODEL_FAILED. On error the state is unchanged.
 */
int kf_ukf_update(kf_state *kf, const kf_real *z, const kf_ukf_params *params,
                  kf_ukf_measurement_fn h, void *ctx);

#endif

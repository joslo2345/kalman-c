#ifndef KF_FIXED_H
#define KF_FIXED_H

/**
 * @file kf_fixed.h
 * @brief Fixed-point linear and extended Kalman filter, for microcontrollers without an FPU.
 *
 * Every quantity is a signed 32-bit Q-format number with KF_FX_FRAC fractional
 * bits: the real value v is stored as round(v * 2^KF_FX_FRAC). The filter
 * mirrors kf_linear.h and kf_ekf.h (Joseph-form update, Cholesky solves) using
 * only integer arithmetic; the KF and EKF share one predict and update core. Each dot product
 * accumulates 64-bit products (each with 2 KF_FX_FRAC fractional bits, the
 * lowest few dropped as guard bits so the sum cannot overflow) and rounds once.
 * Rounding is symmetric (half away from zero).
 *
 * Choose KF_FX_FRAC for the problem, because one format serves every state,
 * covariance and noise value:
 * - the range is about +-2^(31 - KF_FX_FRAC): +-2048 for the default 20;
 * - the resolution is 2^-KF_FX_FRAC: about 9.5e-7 for the default 20.
 * Values below the resolution (for example a 1e-12 process noise) round to
 * zero. Scale the units of the state so values fit (for example kilometres
 * instead of metres) if they do not.
 *
 * Overflow is an error, never a silent wrap or saturation: any result that
 * leaves the range returns KF_ERR_OVERFLOW and leaves the filter unchanged.
 * Use kf_fx_from_double() to convert setup values (it range-checks them);
 * the filter itself uses no floating point.
 */

#include <stdint.h>

#include "kalman/kf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef KF_FX_FRAC
/** Fractional bits of the Q format, 8..28; default 20 (Q11.20). */
#define KF_FX_FRAC 20
#endif

#if (KF_FX_FRAC < 8) || (KF_FX_FRAC > 28)
#error "KF_FX_FRAC must be between 8 and 28"
#endif

/** A Q-format fixed-point number with KF_FX_FRAC fractional bits. */
typedef int32_t kf_fx;

/** The value 1.0 in the Q format. */
#define KF_FX_ONE ((kf_fx)((int32_t)1 << KF_FX_FRAC))

/** Fixed-point filter state, owned by the caller. Matrices are flat, row-major. */
typedef struct {
    int n;                                /**< State size, 1..KF_MAX_STATE. */
    int m;                                /**< Measurement size, 1..KF_MAX_MEAS. */
    kf_fx x[KF_MAX_STATE];                /**< State estimate (n). */
    kf_fx P[KF_MAX_STATE * KF_MAX_STATE]; /**< State covariance (n x n). */
    kf_fx Q[KF_MAX_STATE * KF_MAX_STATE]; /**< Process noise covariance (n x n). */
    kf_fx R[KF_MAX_MEAS * KF_MAX_MEAS];   /**< Measurement noise covariance (m x m). */
    kf_fx nis; /**< Normalized innovation squared from the last update. */
} kf_fx_state;

/**
 * Initialize a fixed-point filter: set the sizes and zero x, P, Q, R and nis.
 * @return KF_OK, or KF_ERR_INVALID_INPUT for a NULL kf or an out-of-range size.
 */
int kf_fx_init(kf_fx_state *kf, int n, int m);

/**
 * Predict: x = F x, P = F P F^T + Q.
 * @param kf Filter state.
 * @param F  State transition matrix (n x n), in the Q format.
 * @return KF_OK, KF_ERR_INVALID_INPUT (NULL arguments, invalid sizes) or
 *         KF_ERR_OVERFLOW. On error the state is unchanged.
 */
int kf_fx_predict(kf_fx_state *kf, const kf_fx *F);

/**
 * Update with a measurement: Joseph-form update, and kf->nis set.
 * @param kf Filter state.
 * @param z  Measurement (m), in the Q format.
 * @param H  Measurement matrix (m x n), in the Q format.
 * @return KF_OK, KF_ERR_INVALID_INPUT, KF_ERR_NOT_POSITIVE_DEFINITE (the
 *         innovation covariance is not positive-definite at this resolution)
 *         or KF_ERR_OVERFLOW. On error the state is unchanged.
 */
int kf_fx_update(kf_fx_state *kf, const kf_fx *z, const kf_fx *H);

/**
 * Fixed-point transition model for kf_fx_ekf_predict(): x_out (n) = f(x) and
 * F_out (n x n) = df/dx at x, all in the Q format. Output buffers are zeroed
 * before the call. Return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_fx_transition_fn)(kf_fx *x_out, kf_fx *F_out, const kf_fx *x, int n, void *ctx);

/**
 * Fixed-point measurement model for kf_fx_ekf_update(): z_out (m) = h(x) and
 * H_out (m x n) = dh/dx at x, all in the Q format. Output buffers are zeroed
 * before the call. Return 0 on success, non-zero to abort the step.
 */
typedef int (*kf_fx_measurement_fn)(kf_fx *z_out, kf_fx *H_out, const kf_fx *x, int n, int m,
                                    void *ctx);

/**
 * Extended predict: x = f(x), P = F P F^T + Q, with F the Jacobian from f.
 * @param kf  Filter state.
 * @param f   Transition model and Jacobian, in the Q format.
 * @param ctx Passed to f unchanged.
 * @return KF_OK, KF_ERR_INVALID_INPUT, KF_ERR_MODEL_FAILED or KF_ERR_OVERFLOW.
 *         On error the state is unchanged.
 */
int kf_fx_ekf_predict(kf_fx_state *kf, kf_fx_transition_fn f, void *ctx);

/**
 * Extended update: y = z - h(x), then the Joseph-form update with H = dh/dx.
 * Angle-valued measurements must be wrapped by the caller before calling.
 * @param kf  Filter state.
 * @param z   Measurement (m), in the Q format.
 * @param h   Measurement model and Jacobian, in the Q format.
 * @param ctx Passed to h unchanged.
 * @return KF_OK, KF_ERR_INVALID_INPUT, KF_ERR_NOT_POSITIVE_DEFINITE,
 *         KF_ERR_MODEL_FAILED or KF_ERR_OVERFLOW. On error the state is unchanged.
 */
int kf_fx_ekf_update(kf_fx_state *kf, const kf_fx *z, kf_fx_measurement_fn h, void *ctx);

/**
 * Convert a real value to the Q format, rounding to nearest.
 * @return KF_OK, KF_ERR_INVALID_INPUT for NaN or a NULL out, or
 *         KF_ERR_OVERFLOW if v is outside the representable range.
 */
int kf_fx_from_double(double v, kf_fx *out);

/** Convert a Q-format value to double (for setup, logging and tests). */
double kf_fx_to_double(kf_fx v);

#ifdef __cplusplus
}
#endif

#endif

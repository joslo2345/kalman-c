#include "kalman/kf_sqrt.h"
#include "kf_linalg_impl.h"

#include <string.h>

/*
 * Square-root filter in UD form: P = U diag(D) U^T with U unit upper
 * triangular. Predict is Thornton's modified weighted Gram-Schmidt; update is
 * Bierman's algorithm, one whitened scalar measurement at a time. As
 * elsewhere, every routine computes into local buffers and commits to *sr only
 * on success.
 */

static int sr_dims_ok(const kf_sr_state *sr) {
    return (sr->n >= 1) && (sr->n <= KF_MAX_STATE) && (sr->m >= 1) && (sr->m <= KF_MAX_MEAS);
}

int kf_sr_init(kf_sr_state *sr, int n, int m) {
    if ((sr == NULL) || (n < 1) || (n > KF_MAX_STATE) || (m < 1) || (m > KF_MAX_MEAS)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(sr, 0, sizeof *sr);
    sr->n = n;
    sr->m = m;
    kfi_mat_identity(sr->U, n);
    kfi_mat_identity(sr->Uq, n);
    kfi_mat_identity(sr->Ur, m);
    return KF_OK;
}

/* Factor a dim x dim covariance into U_out and D_out; psd allows semi-definite input. */
static int set_factors(kf_real *U_out, kf_real *D_out, const kf_real *A, int dim, int psd) {
    kf_real U[KF_MAX_DIM * KF_MAX_DIM];
    kf_real D[KF_MAX_DIM];
    int status;

    if (!kfi_all_finite(A, dim * dim)) {
        return KF_ERR_INVALID_INPUT;
    }
    status = kfi_ud_factor(U, D, A, dim, psd);
    if (status != (int)KF_OK) {
        return status;
    }
    (void)memcpy(U_out, U, (size_t)dim * (size_t)dim * sizeof *U);
    (void)memcpy(D_out, D, (size_t)dim * sizeof *D);
    return KF_OK;
}

int kf_sr_set_P(kf_sr_state *sr, const kf_real *P) {
    if ((sr == NULL) || (P == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    return set_factors(sr->U, sr->D, P, sr->n, 0);
}

int kf_sr_set_Q(kf_sr_state *sr, const kf_real *Q) {
    if ((sr == NULL) || (Q == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    return set_factors(sr->Uq, sr->Dq, Q, sr->n, 1);
}

int kf_sr_set_R(kf_sr_state *sr, const kf_real *R) {
    if ((sr == NULL) || (R == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    return set_factors(sr->Ur, sr->Dr, R, sr->m, 0);
}

int kf_sr_get_P(const kf_sr_state *sr, kf_real *P) {
    if ((sr == NULL) || (P == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = sr->n;
    for (int i = 0; i < n; ++i) { /* P[i][j] = sum_k U[i][k] D[k] U[j][k], k >= max(i, j) */
        for (int j = 0; j < n; ++j) {
            kf_real s = (kf_real)0;
            for (int k = (i > j) ? i : j; k < n; ++k) {
                s += sr->U[i * n + k] * sr->D[k] * sr->U[j * n + k];
            }
            P[i * n + j] = s;
        }
    }
    return KF_OK;
}

/*
 * Thornton's predict. With W = [F U, Uq] (n x 2n) and weights
 * Dw = [D, Dq], F P F^T + Q = W diag(Dw) W^T. Weighted Gram-Schmidt on the
 * rows of W, from the last up, gives the new U and D directly: each new
 * D[k] is a weighted sum of squares, so it cannot go negative.
 */
static int sr_core_predict(kf_sr_state *sr, const kf_real *x_pred, const kf_real *F) {
    kf_real W[KF_MAX_STATE * 2 * KF_MAX_STATE];
    kf_real Dw[2 * KF_MAX_STATE];
    kf_real c[2 * KF_MAX_STATE];
    kf_real U[KF_MAX_STATE * KF_MAX_STATE];
    kf_real D[KF_MAX_STATE];
    /* Re-checked here: a model callback receives ctx, which may alias *sr. */
    if (!sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = sr->n;
    const int w = 2 * n;

    if (!kfi_all_finite(x_pred, n) || !kfi_all_finite(F, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) { /* (F U)[i][j]; U[k][j] = 0 for k > j */
            kf_real s = (kf_real)0;
            for (int k = 0; k <= j; ++k) {
                s += F[i * n + k] * sr->U[k * n + j];
            }
            W[i * w + j] = s;
            W[i * w + n + j] = sr->Uq[i * n + j];
        }
        Dw[i] = sr->D[i];
        Dw[n + i] = sr->Dq[i];
    }
    kfi_mat_identity(U, n);

    for (int k = n - 1; k >= 0; --k) {
        kf_real dk = (kf_real)0;
        for (int j = 0; j < w; ++j) {
            c[j] = W[k * w + j] * Dw[j];
            dk += W[k * w + j] * c[j];
        }
        D[k] = dk;
        if (!(dk > (kf_real)0)) { /* zero variance: nothing to project out */
            D[k] = (kf_real)0;
            continue;
        }
        const kf_real inv = (kf_real)1 / dk;
        for (int i = 0; i < k; ++i) {
            kf_real s = (kf_real)0;
            for (int j = 0; j < w; ++j) {
                s += W[i * w + j] * c[j];
            }
            const kf_real u = s * inv;
            U[i * n + k] = u;
            for (int j = 0; j < w; ++j) {
                W[i * w + j] -= u * W[k * w + j];
            }
        }
    }

    if (!kfi_all_finite(U, n * n) || !kfi_all_finite(D, n)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memcpy(sr->x, x_pred, (size_t)n * sizeof *x_pred);
    (void)memcpy(sr->U, U, (size_t)n * (size_t)n * sizeof *U);
    (void)memcpy(sr->D, D, (size_t)n * sizeof *D);
    return KF_OK;
}

/* Whiten the measurement: solve Ur [yw, Hw] = [y, H] (Ur unit upper triangular). */
static void sr_whiten(const kf_sr_state *sr, const kf_real *y, const kf_real *H, kf_real *yw,
                      kf_real *Hw) {
    const int n = sr->n;
    const int m = sr->m;
    for (int i = m - 1; i >= 0; --i) {
        kf_real s = y[i];
        for (int p = i + 1; p < m; ++p) {
            s -= sr->Ur[i * m + p] * yw[p];
        }
        yw[i] = s;
        for (int j = 0; j < n; ++j) {
            kf_real t = H[i * n + j];
            for (int p = i + 1; p < m; ++p) {
                t -= sr->Ur[i * m + p] * Hw[p * n + j];
            }
            Hw[i * n + j] = t;
        }
    }
}

/*
 * One Bierman scalar update of U and D for measurement row h with variance r.
 * Writes the unnormalized gain to b (K = b / a) and returns a, the innovation
 * variance. Each new D[j] = D[j] * a_prev / a, with a a growing sum of
 * non-negative terms, so no subtraction produces a variance.
 */
static kf_real sr_bierman(kf_real *U, kf_real *D, kf_real *b, const kf_real *h, kf_real r, int n) {
    kf_real f[KF_MAX_STATE];
    kf_real v[KF_MAX_STATE];
    for (int j = 0; j < n; ++j) { /* f = U^T h, v = D f */
        kf_real s = h[j];
        for (int k = 0; k < j; ++k) {
            s += U[k * n + j] * h[k];
        }
        f[j] = s;
        v[j] = D[j] * s;
    }
    kf_real a = r;
    for (int j = 0; j < n; ++j) {
        const kf_real a_prev = a;
        a += f[j] * v[j];
        D[j] = D[j] * (a_prev / a);
        b[j] = v[j];
        const kf_real lambda = -f[j] / a_prev;
        for (int k = 0; k < j; ++k) {
            const kf_real u = U[k * n + j];
            U[k * n + j] = u + (b[k] * lambda);
            b[k] += u * v[j];
        }
    }
    return a;
}

/*
 * Update, given the innovation y = z - h(x0) at the prior x0. R's factors
 * whiten the measurement into m independent scalars with variances Dr, each
 * processed by sr_bierman in turn. Its innovation is y'_i - h'_i (x - x0),
 * which equals z'_i - h'_i x for a linear model. NIS is the sum of the scalar
 * ones.
 */
static int sr_core_update(kf_sr_state *sr, const kf_real *y, const kf_real *H) {
    kf_real yw[KF_MAX_MEAS];
    kf_real Hw[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real U[KF_MAX_STATE * KF_MAX_STATE];
    kf_real D[KF_MAX_STATE];
    kf_real x[KF_MAX_STATE];
    kf_real b[KF_MAX_STATE];
    kf_real nis = (kf_real)0;

    /* Re-checked here: a model callback receives ctx, which may alias *sr. */
    if (!sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = sr->n;
    const int m = sr->m;
    if (!kfi_all_finite(y, m) || !kfi_all_finite(H, m * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    sr_whiten(sr, y, H, yw, Hw);
    (void)memcpy(U, sr->U, (size_t)n * (size_t)n * sizeof *U);
    (void)memcpy(D, sr->D, (size_t)n * sizeof *D);
    (void)memcpy(x, sr->x, (size_t)n * sizeof *x);

    for (int i = 0; i < m; ++i) {
        const kf_real *h = &Hw[i * n];
        if (!(sr->Dr[i] > (kf_real)0)) {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        kf_real innov = yw[i];
        for (int j = 0; j < n; ++j) {
            innov -= h[j] * (x[j] - sr->x[j]);
        }
        const kf_real a = sr_bierman(U, D, b, h, sr->Dr[i], n);
        if (!(a > (kf_real)0) || !isfinite(a)) {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        const kf_real gain = innov / a;
        for (int j = 0; j < n; ++j) {
            x[j] += b[j] * gain;
        }
        nis += innov * gain;
    }

    if (!kfi_all_finite(x, n) || !kfi_all_finite(U, n * n) || !kfi_all_finite(D, n)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memcpy(sr->x, x, (size_t)n * sizeof *x);
    (void)memcpy(sr->U, U, (size_t)n * (size_t)n * sizeof *U);
    (void)memcpy(sr->D, D, (size_t)n * sizeof *D);
    sr->nis = nis;
    return KF_OK;
}

int kf_sr_predict(kf_sr_state *sr, const kf_real *F) {
    kf_real x[KF_MAX_STATE];

    if ((sr == NULL) || (F == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    kfi_mat_mul(x, F, sr->x, sr->n, sr->n, 1);
    return sr_core_predict(sr, x, F);
}

int kf_sr_update(kf_sr_state *sr, const kf_real *z, const kf_real *H) {
    kf_real Hx[KF_MAX_MEAS];
    kf_real y[KF_MAX_MEAS];

    if ((sr == NULL) || (z == NULL) || (H == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kfi_all_finite(z, sr->m)) {
        return KF_ERR_INVALID_INPUT;
    }
    kfi_mat_mul(Hx, H, sr->x, sr->m, sr->n, 1);
    kfi_mat_sub(y, z, Hx, sr->m, 1);
    return sr_core_update(sr, y, H);
}

int kf_sr_ekf_predict(kf_sr_state *sr, kf_ekf_transition_fn f, void *ctx) {
    kf_real x[KF_MAX_STATE];
    kf_real F[KF_MAX_STATE * KF_MAX_STATE];

    if ((sr == NULL) || (f == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(x, 0, sizeof x);
    (void)memset(F, 0, sizeof F);
    if (f(x, F, sr->x, sr->n, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    return sr_core_predict(sr, x, F);
}

int kf_sr_ekf_update(kf_sr_state *sr, const kf_real *z, kf_ekf_measurement_fn h, void *ctx) {
    kf_real hx[KF_MAX_MEAS];
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real y[KF_MAX_MEAS];

    if ((sr == NULL) || (z == NULL) || (h == NULL) || !sr_dims_ok(sr)) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kfi_all_finite(z, sr->m)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(hx, 0, sizeof hx);
    (void)memset(H, 0, sizeof H);
    if (h(hx, H, sr->x, sr->n, sr->m, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    kfi_mat_sub(y, z, hx, sr->m, 1);
    return sr_core_update(sr, y, H);
}

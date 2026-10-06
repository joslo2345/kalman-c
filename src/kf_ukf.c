#include "kalman/kf_ukf.h"
#include "kf_internal.h"
#include "kf_linalg_impl.h"

#include <math.h>
#include <string.h>

#define KF_MAX_SIGMA (2 * KF_MAX_STATE + 1)

/* Sigma points are stored one per row: X[j * n + i] is component i of point j. */
typedef struct {
    int count;
    /* Mean and covariance weights for point 0, and the weight shared by the rest. */
    kf_real wm0;
    kf_real wc0;
    kf_real wi;
    kf_real X[KF_MAX_SIGMA * KF_MAX_STATE];
} sigma_set;

static const kf_ukf_params default_params = {1, 2, 0};

/* Weights and sigma points X_0 = x, X_i = x + sqrt(c) L_i, X_{n+i} = x - sqrt(c) L_i,
 * where L_i is column i of the Cholesky factor of P and c = n + lambda. */
static int make_sigma_points(sigma_set *s, const kf_state *kf, const kf_ukf_params *p) {
    kf_real L[KF_MAX_STATE * KF_MAX_STATE];
    const int n = kf->n;
    const kf_real c = p->alpha * p->alpha * ((kf_real)n + p->kappa);
    int status;

    if (!(c > (kf_real)0) || !isfinite(c) || !isfinite(p->beta)) {
        return KF_ERR_INVALID_INPUT;
    }
    status = kfi_cholesky(L, kf->P, n);
    if (status != (int)KF_OK) {
        return status;
    }

    s->count = 2 * n + 1;
    s->wm0 = (c - (kf_real)n) / c;
    s->wc0 = s->wm0 + (((kf_real)1 - (p->alpha * p->alpha)) + p->beta);
    s->wi = (kf_real)0.5 / c;

    const kf_real scale = KF_SQRT(c);
    (void)memcpy(s->X, kf->x, (size_t)n * sizeof *kf->x);
    for (int i = 0; i < n; ++i) {
        kf_real *plus = &s->X[(1 + i) * n];
        kf_real *minus = &s->X[(1 + n + i) * n];
        for (int r = 0; r < n; ++r) {
            const kf_real d = scale * L[r * n + i];
            plus[r] = kf->x[r] + d;
            minus[r] = kf->x[r] - d;
        }
    }
    return KF_OK;
}

static kf_real weight_m(const sigma_set *s, int j) {
    return j == 0 ? s->wm0 : s->wi;
}

static kf_real weight_c(const sigma_set *s, int j) {
    return j == 0 ? s->wc0 : s->wi;
}

/* mean (dim) = sum_j Wm_j Y_j, for points stored one per row of width dim */
static void weighted_mean(kf_real *mean, const sigma_set *s, const kf_real *Y, int dim) {
    (void)memset(mean, 0, (size_t)dim * sizeof *mean);
    for (int j = 0; j < s->count; ++j) {
        const kf_real w = weight_m(s, j);
        for (int i = 0; i < dim; ++i) {
            mean[i] += w * Y[j * dim + i];
        }
    }
}

/* C (da x db) = sum_j Wc_j (A_j - a)(B_j - b)^T */
static void weighted_cross_cov(kf_real *C, const sigma_set *s, const kf_real *A, const kf_real *a,
                               int da, const kf_real *B, const kf_real *b, int db) {
    (void)memset(C, 0, (size_t)da * (size_t)db * sizeof *C);
    for (int j = 0; j < s->count; ++j) {
        const kf_real w = weight_c(s, j);
        for (int r = 0; r < da; ++r) {
            const kf_real dr = A[j * da + r] - a[r];
            for (int q = 0; q < db; ++q) {
                C[r * db + q] += w * dr * (B[j * db + q] - b[q]);
            }
        }
    }
}

int kf_ukf_predict(kf_state *kf, const kf_ukf_params *params, kf_ukf_transition_fn f, void *ctx) {
    sigma_set s;
    kf_real Y[KF_MAX_SIGMA * KF_MAX_STATE];
    kf_real x[KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];
    int status;

    if (kf == NULL || f == NULL || !kf_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    status = make_sigma_points(&s, kf, params ? params : &default_params);
    if (status != (int)KF_OK) {
        return status;
    }

    (void)memset(Y, 0, sizeof Y);
    for (int j = 0; j < s.count; ++j) {
        if (f(&Y[j * n], &s.X[j * n], n, ctx) != 0) {
            return KF_ERR_MODEL_FAILED;
        }
    }
    if (!kfi_all_finite(Y, s.count * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    weighted_mean(x, &s, Y, n);
    weighted_cross_cov(P, &s, Y, x, n, Y, x, n);
    kfi_mat_add(P, P, kf->Q, n, n);
    kfi_mat_symmetrize(P, n);

    /* A negative centre weight can push P out of the positive-definite cone. */
    if (!kfi_all_finite(P, n * n) || !kfi_cholesky_ok(P, n)) {
        return KF_ERR_NOT_POSITIVE_DEFINITE;
    }
    (void)memcpy(kf->x, x, (size_t)n * sizeof *x);
    (void)memcpy(kf->P, P, (size_t)n * (size_t)n * sizeof *P);
    return KF_OK;
}

int kf_ukf_update(kf_state *kf, const kf_real *z, const kf_ukf_params *params,
                  kf_ukf_measurement_fn h, void *ctx) {
    sigma_set s;
    kf_real Z[KF_MAX_SIGMA * KF_MAX_MEAS];
    kf_real z_hat[KF_MAX_MEAS];
    kf_real y[KF_MAX_MEAS];
    kf_real w[KF_MAX_MEAS];
    kf_real S[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real L[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real Pxz[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real Kt[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real K[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real KS[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real KSKt[KF_MAX_STATE * KF_MAX_STATE];
    kf_real x[KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];
    kf_real nis = (kf_real)0;
    int status;

    if (kf == NULL || z == NULL || h == NULL || !kf_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    const int m = kf->m;
    if (!kfi_all_finite(z, m)) {
        return KF_ERR_INVALID_INPUT;
    }
    status = make_sigma_points(&s, kf, params ? params : &default_params);
    if (status != (int)KF_OK) {
        return status;
    }

    (void)memset(Z, 0, sizeof Z);
    for (int j = 0; j < s.count; ++j) {
        if (h(&Z[j * m], &s.X[j * n], n, m, ctx) != 0) {
            return KF_ERR_MODEL_FAILED;
        }
    }
    if (!kfi_all_finite(Z, s.count * m)) {
        return KF_ERR_INVALID_INPUT;
    }

    /* Predicted measurement, innovation covariance S, and cross-covariance Pxz */
    weighted_mean(z_hat, &s, Z, m);
    weighted_cross_cov(S, &s, Z, z_hat, m, Z, z_hat, m);
    kfi_mat_add(S, S, kf->R, m, m);
    kfi_mat_symmetrize(S, m);
    weighted_cross_cov(Pxz, &s, s.X, kf->x, n, Z, z_hat, m);

    status = kfi_cholesky(L, S, m);
    if (status != (int)KF_OK) {
        return status;
    }

    /* Gain: S K^T = Pxz^T */
    kfi_mat_transpose(Kt, Pxz, n, m);
    kfi_cholesky_solve(Kt, L, Kt, m, n);
    kfi_mat_transpose(K, Kt, m, n);

    /* NIS = y^T S^-1 y = |L^-1 y|^2 */
    kfi_mat_sub(y, z, z_hat, m, 1);
    kfi_solve_lower(w, L, y, m, 1);
    for (int i = 0; i < m; ++i) {
        nis += w[i] * w[i];
    }

    /* x = x + K y,  P = P - K S K^T */
    kfi_mat_mul(x, K, y, n, m, 1);
    kfi_mat_add(x, kf->x, x, n, 1);
    kfi_mat_mul(KS, K, S, n, m, m);
    kfi_mat_mul(KSKt, KS, Kt, n, m, n);
    kfi_mat_sub(P, kf->P, KSKt, n, n);
    kfi_mat_symmetrize(P, n);

    if (!kfi_all_finite(x, n) || !kfi_all_finite(P, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kfi_cholesky_ok(P, n)) {
        return KF_ERR_NOT_POSITIVE_DEFINITE;
    }
    (void)memcpy(kf->x, x, (size_t)n * sizeof *x);
    (void)memcpy(kf->P, P, (size_t)n * (size_t)n * sizeof *P);
    kf->nis = nis;
    return KF_OK;
}

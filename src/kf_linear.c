#include "kalman/kf_linear.h"
#include "kalman/kf_linalg.h"

#include <string.h>

/*
 * Every routine computes into local buffers and only writes to *kf once the
 * whole step has succeeded, so an error leaves the filter state unchanged.
 */

int kf_init(kf_state *kf, int n, int m) {
    if (kf == NULL || n < 1 || n > KF_MAX_STATE || m < 1 || m > KF_MAX_MEAS) {
        return KF_ERR_INVALID_INPUT;
    }
    memset(kf, 0, sizeof *kf);
    kf->n = n;
    kf->m = m;
    return KF_OK;
}

/* x = F x,  P = F P F^T + Q */
int kf_predict(kf_state *kf, const kf_real *F) {
    kf_real x[KF_MAX_STATE];
    kf_real FP[KF_MAX_STATE * KF_MAX_STATE];
    kf_real Ft[KF_MAX_STATE * KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];

    if (kf == NULL || F == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    if (!kf_all_finite(F, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    kf_mat_mul(x, F, kf->x, n, n, 1);
    kf_mat_mul(FP, F, kf->P, n, n, n);
    kf_mat_transpose(Ft, F, n, n);
    kf_mat_mul(P, FP, Ft, n, n, n);
    kf_mat_add(P, P, kf->Q, n, n);
    kf_mat_symmetrize(P, n);

    if (!kf_all_finite(x, n) || !kf_all_finite(P, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }
    memcpy(kf->x, x, (size_t)n * sizeof *x);
    memcpy(kf->P, P, (size_t)(n * n) * sizeof *P);
    return KF_OK;
}

/*
 * Joseph-form update:
 *   y = z - H x,  S = H P H^T + R,  K = P H^T S^-1
 *   x = x + K y
 *   P = (I - K H) P (I - K H)^T + K R K^T
 * K is formed by solving S K^T = H P with the Cholesky factor of S,
 * so S is never inverted explicitly.
 */
int kf_update(kf_state *kf, const kf_real *z, const kf_real *H) {
    kf_real y[KF_MAX_MEAS];
    kf_real w[KF_MAX_MEAS];
    kf_real Hx[KF_MAX_MEAS];
    kf_real Ht[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real PHt[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real S[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real L[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real Kt[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real K[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real KR[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real A[KF_MAX_STATE * KF_MAX_STATE]; /* I - K H, then reused */
    kf_real T[KF_MAX_STATE * KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];
    kf_real x[KF_MAX_STATE];
    kf_real nis = 0;
    int status;

    if (kf == NULL || z == NULL || H == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    const int m = kf->m;
    if (!kf_all_finite(z, m) || !kf_all_finite(H, m * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    /* Innovation y and its covariance S */
    kf_mat_mul(Hx, H, kf->x, m, n, 1);
    kf_mat_sub(y, z, Hx, m, 1);
    kf_mat_transpose(Ht, H, m, n);
    kf_mat_mul(PHt, kf->P, Ht, n, n, m);
    kf_mat_mul(S, H, PHt, m, n, m);
    kf_mat_add(S, S, kf->R, m, m);
    kf_mat_symmetrize(S, m);

    status = kf_cholesky(L, S, m);
    if (status != KF_OK) {
        return status;
    }

    /* Gain: S K^T = (P H^T)^T */
    kf_mat_transpose(Kt, PHt, n, m);
    kf_cholesky_solve(Kt, L, Kt, m, n);
    kf_mat_transpose(K, Kt, m, n);

    /* NIS = y^T S^-1 y = |L^-1 y|^2 */
    kf_solve_lower(w, L, y, m, 1);
    for (int i = 0; i < m; ++i) {
        nis += w[i] * w[i];
    }

    /* State */
    kf_mat_mul(x, K, y, n, m, 1);
    kf_mat_add(x, kf->x, x, n, 1);

    /* Joseph-form covariance */
    kf_mat_mul(T, K, H, n, m, n);
    kf_mat_identity(A, n);
    kf_mat_sub(A, A, T, n, n);              /* A = I - K H */
    kf_mat_mul(T, A, kf->P, n, n, n);       /* T = A P */
    kf_mat_transpose(P, A, n, n);
    kf_mat_mul(A, T, P, n, n, n);           /* A = A P A^T */
    kf_mat_mul(KR, K, kf->R, n, m, m);
    kf_mat_mul(T, KR, Kt, n, m, n);         /* T = K R K^T */
    kf_mat_add(P, A, T, n, n);
    kf_mat_symmetrize(P, n);

    if (!kf_all_finite(x, n) || !kf_all_finite(P, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }
    memcpy(kf->x, x, (size_t)n * sizeof *x);
    memcpy(kf->P, P, (size_t)(n * n) * sizeof *P);
    kf->nis = nis;
    return KF_OK;
}

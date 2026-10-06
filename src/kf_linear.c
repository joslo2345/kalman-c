#include "kalman/kf_linear.h"
#include "kf_linalg_impl.h"
#include "kf_internal.h"

#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define KF_ALWAYS_INLINE static inline __attribute__((always_inline))
#else
#define KF_ALWAYS_INLINE static inline
#endif

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

/* P = F P F^T + Q. Only the upper triangle of the symmetric result is computed.
 * n is a compile-time constant when called from a KF_SPECIALIZE entry. */
KF_ALWAYS_INLINE int core_predict(kf_state *kf, const kf_real *x_pred, const kf_real *F,
                                  const int n) {
    kf_real FP[KF_MAX_STATE * KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];

    if (!kfi_all_finite(x_pred, n) || !kfi_all_finite(F, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    kfi_mat_mul(FP, F, kf->P, n, n, n);
    for (int i = 0; i < n; ++i) {
        const kf_real *fp = &FP[i * n];
        for (int j = i; j < n; ++j) {
            const kf_real *f = &F[j * n];
            kf_real s = 0;
            for (int k = 0; k < n; ++k) {
                s += fp[k] * f[k];
            }
            s += (kf->Q[i * n + j] + kf->Q[j * n + i]) * (kf_real)0.5;
            P[i * n + j] = s;
            P[j * n + i] = s;
        }
    }

    if (!kfi_all_finite(P, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }
    memcpy(kf->x, x_pred, (size_t)n * sizeof *x_pred);
    memcpy(kf->P, P, (size_t)(n * n) * sizeof *P);
    return KF_OK;
}

int kf_core_predict(kf_state *kf, const kf_real *x_pred, const kf_real *F) {
#ifdef KF_SPECIALIZE
#define KF_SIZE(N, M)                                                                              \
    if (kf->n == (N)) return core_predict(kf, x_pred, F, N);
    KF_SPECIALIZE
#undef KF_SIZE
#endif
    return core_predict(kf, x_pred, F, kf->n);
}

/* x = F x,  P = F P F^T + Q */
int kf_predict(kf_state *kf, const kf_real *F) {
    kf_real x[KF_MAX_STATE];

    if (kf == NULL || F == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    kfi_mat_mul(x, F, kf->x, kf->n, kf->n, 1);
    return kf_core_predict(kf, x, F);
}

/*
 * Joseph-form update, given the innovation y:
 *   S = H P H^T + R,  K = P H^T S^-1
 *   x = x + K y
 *   P = (I - K H) P (I - K H)^T + K R K^T
 *
 * K is formed by solving S K^T = H P with the Cholesky factor of S, so S is
 * never inverted explicitly. Because K H has rank m, the Joseph product is
 * evaluated in O(n^2 m) without forming the n x n matrix I - K H:
 *   B = (I - K H) P   = P - K (H P)
 *   B (I - K H)^T     = B - (B H^T) K^T
 * and K R K^T is added as its own term, keeping the Joseph structure.
 */
KF_ALWAYS_INLINE int core_update(kf_state *kf, const kf_real *y, const kf_real *H, const int n,
                                 const int m) {
    kf_real w[KF_MAX_MEAS];
    kf_real HP[KF_MAX_MEAS * KF_MAX_STATE]; /* H P = (P H^T)^T, as P is symmetric */
    kf_real S[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real L[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real Kt[KF_MAX_MEAS * KF_MAX_STATE];  /* K^T, m x n */
    kf_real RKt[KF_MAX_MEAS * KF_MAX_STATE]; /* R K^T, m x n */
    kf_real B[KF_MAX_STATE * KF_MAX_STATE];
    kf_real BHt[KF_MAX_STATE * KF_MAX_MEAS];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];
    kf_real x[KF_MAX_STATE];
    kf_real nis = 0;
    int status;

    if (!kfi_all_finite(y, m) || !kfi_all_finite(H, m * n)) {
        return KF_ERR_INVALID_INPUT;
    }

    /* Innovation covariance S = (H P) H^T + R */
    kfi_mat_mul(HP, H, kf->P, m, n, n);
    kfi_mat_mul_abt(S, HP, H, m, n, m);
    kfi_mat_add(S, S, kf->R, m, m);
    kfi_mat_symmetrize(S, m);

    status = kfi_cholesky(L, S, m);
    if (status != KF_OK) {
        return status;
    }

    /* Gain: S K^T = H P */
    kfi_cholesky_solve(Kt, L, HP, m, n);

    /* NIS = y^T S^-1 y = |L^-1 y|^2 */
    kfi_solve_lower(w, L, y, m, 1);
    for (int i = 0; i < m; ++i) {
        nis += w[i] * w[i];
    }

    /* State: x = x + K y = x + (K^T)^T y */
    for (int i = 0; i < n; ++i) {
        kf_real s = kf->x[i];
        for (int j = 0; j < m; ++j) {
            s += Kt[j * n + i] * y[j];
        }
        x[i] = s;
    }

    /* B = P - K (H P) */
    for (int i = 0; i < n; ++i) {
        for (int c = 0; c < n; ++c) {
            kf_real s = 0;
            for (int j = 0; j < m; ++j) {
                s += Kt[j * n + i] * HP[j * n + c];
            }
            B[i * n + c] = kf->P[i * n + c] - s;
        }
    }

    /* P = (B - (B H^T) K^T) + K R K^T, upper triangle mirrored */
    kfi_mat_mul_abt(BHt, B, H, n, n, m);
    kfi_mat_mul(RKt, kf->R, Kt, m, m, n);
    for (int i = 0; i < n; ++i) {
        for (int c = i; c < n; ++c) {
            kf_real apa_upper = B[i * n + c], apa_lower = B[c * n + i], krk = 0;
            for (int j = 0; j < m; ++j) {
                apa_upper -= BHt[i * m + j] * Kt[j * n + c];
                apa_lower -= BHt[c * m + j] * Kt[j * n + i];
                krk += Kt[j * n + i] * RKt[j * n + c];
            }
            const kf_real v = (apa_upper + apa_lower) * (kf_real)0.5 + krk;
            P[i * n + c] = v;
            P[c * n + i] = v;
        }
    }

    if (!kfi_all_finite(x, n) || !kfi_all_finite(P, n * n)) {
        return KF_ERR_INVALID_INPUT;
    }
    memcpy(kf->x, x, (size_t)n * sizeof *x);
    memcpy(kf->P, P, (size_t)(n * n) * sizeof *P);
    kf->nis = nis;
    return KF_OK;
}

int kf_core_update(kf_state *kf, const kf_real *y, const kf_real *H) {
#ifdef KF_SPECIALIZE
#define KF_SIZE(N, M)                                                                              \
    if (kf->n == (N) && kf->m == (M)) return core_update(kf, y, H, N, M);
    KF_SPECIALIZE
#undef KF_SIZE
#endif
    return core_update(kf, y, H, kf->n, kf->m);
}

/* y = z - H x, then the Joseph-form update */
int kf_update(kf_state *kf, const kf_real *z, const kf_real *H) {
    kf_real Hx[KF_MAX_MEAS];
    kf_real y[KF_MAX_MEAS];

    if (kf == NULL || z == NULL || H == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kfi_all_finite(z, kf->m)) {
        return KF_ERR_INVALID_INPUT;
    }
    kfi_mat_mul(Hx, H, kf->x, kf->m, kf->n, 1);
    kfi_mat_sub(y, z, Hx, kf->m, 1);
    return kf_core_update(kf, y, H);
}

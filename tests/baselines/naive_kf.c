#include "baselines/naive_kf.h"

#include <math.h>
#include <string.h>

static void mul(kf_real *C, const kf_real *A, const kf_real *B, int r, int k, int c) {
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) {
            kf_real s = 0;
            for (int p = 0; p < k; ++p) s += A[i * k + p] * B[p * c + j];
            C[i * c + j] = s;
        }
}

static void transpose(kf_real *At, const kf_real *A, int r, int c) {
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) At[j * r + i] = A[i * c + j];
}

/* Gauss-Jordan inverse with partial pivoting. Returns -1 if singular. */
static int invert(kf_real *Ainv, const kf_real *A, int n) {
    kf_real M[KF_MAX_MEAS * KF_MAX_MEAS];
    memcpy(M, A, (size_t)(n * n) * sizeof *M);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) Ainv[i * n + j] = (i == j);

    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r)
            if (fabs((double)M[r * n + col]) > fabs((double)M[piv * n + col])) piv = r;
        if (M[piv * n + col] == 0) return -1;
        for (int j = 0; j < n; ++j) {
            kf_real t = M[col * n + j]; M[col * n + j] = M[piv * n + j]; M[piv * n + j] = t;
            t = Ainv[col * n + j]; Ainv[col * n + j] = Ainv[piv * n + j]; Ainv[piv * n + j] = t;
        }
        const kf_real d = M[col * n + col];
        for (int j = 0; j < n; ++j) {
            M[col * n + j] /= d;
            Ainv[col * n + j] /= d;
        }
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const kf_real f = M[r * n + col];
            for (int j = 0; j < n; ++j) {
                M[r * n + j] -= f * M[col * n + j];
                Ainv[r * n + j] -= f * Ainv[col * n + j];
            }
        }
    }
    return 0;
}

int naive_kf_step(naive_kf *kf, const kf_real *z) {
    enum { N = KF_MAX_STATE, M = KF_MAX_MEAS };
    kf_real x[N], FP[N * N], Ft[N * N], Ht[N * M], PHt[N * M], S[M * M], Sinv[M * M];
    kf_real K[N * M], Hx[M], y[M], Ky[N], KH[N * N], IKH[N * N], P[N * N];
    const int n = kf->n, m = kf->m;

    /* Predict */
    mul(x, kf->F, kf->x, n, n, 1);
    mul(FP, kf->F, kf->P, n, n, n);
    transpose(Ft, kf->F, n, n);
    mul(kf->P, FP, Ft, n, n, n);
    for (int i = 0; i < n * n; ++i) kf->P[i] += kf->Q[i];
    memcpy(kf->x, x, (size_t)n * sizeof *x);

    /* Update */
    transpose(Ht, kf->H, m, n);
    mul(PHt, kf->P, Ht, n, n, m);
    mul(S, kf->H, PHt, m, n, m);
    for (int i = 0; i < m * m; ++i) S[i] += kf->R[i];
    if (invert(Sinv, S, m) != 0) return -1;
    mul(K, PHt, Sinv, n, m, m);

    mul(Hx, kf->H, kf->x, m, n, 1);
    for (int i = 0; i < m; ++i) y[i] = z[i] - Hx[i];
    mul(Ky, K, y, n, m, 1);
    for (int i = 0; i < n; ++i) kf->x[i] += Ky[i];

    mul(KH, K, kf->H, n, m, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) IKH[i * n + j] = (i == j) - KH[i * n + j];
    mul(P, IKH, kf->P, n, n, n);
    memcpy(kf->P, P, (size_t)(n * n) * sizeof *P);
    return 0;
}

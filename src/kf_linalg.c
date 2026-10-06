#include "kalman/kf_linalg.h"

#include <math.h>

#ifdef KF_USE_DOUBLE
#define KF_SQRT sqrt
#else
#define KF_SQRT sqrtf
#endif

void kf_mat_mul(kf_real *C, const kf_real *A, const kf_real *B, int r, int k, int c) {
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            kf_real sum = 0;
            for (int p = 0; p < k; ++p) {
                sum += A[i * k + p] * B[p * c + j];
            }
            C[i * c + j] = sum;
        }
    }
}

void kf_mat_transpose(kf_real *At, const kf_real *A, int r, int c) {
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            At[j * r + i] = A[i * c + j];
        }
    }
}

void kf_mat_add(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    for (int i = 0; i < r * c; ++i) {
        C[i] = A[i] + B[i];
    }
}

void kf_mat_sub(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    for (int i = 0; i < r * c; ++i) {
        C[i] = A[i] - B[i];
    }
}

void kf_mat_identity(kf_real *A, int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A[i * n + j] = (i == j) ? (kf_real)1 : (kf_real)0;
        }
    }
}

void kf_mat_symmetrize(kf_real *A, int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            kf_real avg = (A[i * n + j] + A[j * n + i]) * (kf_real)0.5;
            A[i * n + j] = avg;
            A[j * n + i] = avg;
        }
    }
}

int kf_cholesky(kf_real *L, const kf_real *A, int n) {
    for (int j = 0; j < n; ++j) {
        kf_real d = A[j * n + j];
        for (int p = 0; p < j; ++p) {
            d -= L[j * n + p] * L[j * n + p];
        }
        if (!(d > 0) || !isfinite(d)) { /* also catches NaN */
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        kf_real ljj = KF_SQRT(d);
        L[j * n + j] = ljj;

        for (int i = j + 1; i < n; ++i) {
            kf_real s = A[i * n + j];
            for (int p = 0; p < j; ++p) {
                s -= L[i * n + p] * L[j * n + p];
            }
            L[i * n + j] = s / ljj;
        }
        for (int i = 0; i < j; ++i) {
            L[i * n + j] = 0; /* zero the upper triangle */
        }
    }
    return KF_OK;
}

void kf_solve_lower(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    for (int col = 0; col < c; ++col) {
        for (int i = 0; i < n; ++i) {
            kf_real s = B[i * c + col];
            for (int p = 0; p < i; ++p) {
                s -= L[i * n + p] * X[p * c + col];
            }
            X[i * c + col] = s / L[i * n + i];
        }
    }
}

void kf_solve_lower_t(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    for (int col = 0; col < c; ++col) {
        for (int i = n - 1; i >= 0; --i) {
            kf_real s = B[i * c + col];
            for (int p = i + 1; p < n; ++p) {
                s -= L[p * n + i] * X[p * c + col];
            }
            X[i * c + col] = s / L[i * n + i];
        }
    }
}

void kf_cholesky_solve(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    kf_solve_lower(X, L, B, n, c);
    kf_solve_lower_t(X, L, X, n, c);
}

int kf_is_symmetric(const kf_real *A, int n, kf_real tol) {
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            kf_real diff = A[i * n + j] - A[j * n + i];
            if (!(diff <= tol && -diff <= tol)) {
                return 0;
            }
        }
    }
    return 1;
}

int kf_cholesky_ok(const kf_real *A, int n) {
    kf_real L[KF_MAX_DIM * KF_MAX_DIM];
    if (n < 1 || n > KF_MAX_DIM) {
        return 0;
    }
    return kf_cholesky(L, A, n) == KF_OK;
}

int kf_all_finite(const kf_real *v, int len) {
    for (int i = 0; i < len; ++i) {
        if (!isfinite(v[i])) {
            return 0;
        }
    }
    return 1;
}

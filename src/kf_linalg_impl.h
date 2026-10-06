#ifndef KF_LINALG_IMPL_H
#define KF_LINALG_IMPL_H

/*
 * Matrix kernels as static inline functions, so the filter code can inline
 * them. The public kf_mat_* / kf_cholesky* API in kf_linalg.c wraps these.
 * Not part of the public API.
 */

#include "kalman/kf_linalg.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define KFI_INLINE static inline __attribute__((always_inline))
#else
#define KFI_INLINE static inline
#endif

#ifdef KF_USE_DOUBLE
#define KFI_EPS DBL_EPSILON
#else
#define KFI_EPS FLT_EPSILON
#endif

#ifdef KF_USE_DOUBLE
#define KF_SQRT sqrt
#else
#define KF_SQRT sqrtf
#endif

/*
 * The products below use the row-axpy form C[i,:] += a * B[p,:], whose inner
 * loop runs over contiguous memory with independent elements, so compilers
 * vectorize it without -ffast-math. A dot-product inner loop would be a float
 * reduction, which they may not reorder.
 */

KFI_INLINE void kfi_mat_mul(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                            const kf_real *KF_RESTRICT B, int r, int k, int c) {
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            kf_real sum = (kf_real)0;
            for (int p = 0; p < k; ++p) {
                sum += A[i * k + p] * B[p * c + j];
            }
            C[i * c + j] = sum;
        }
    }
}

KFI_INLINE void kfi_mat_mul_abt(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                                const kf_real *KF_RESTRICT B, int r, int k, int c) {
    for (int i = 0; i < r; ++i) {
        const kf_real *a = &A[i * k];
        for (int j = 0; j < c; ++j) {
            const kf_real *b = &B[j * k];
            kf_real sum = (kf_real)0;
            for (int p = 0; p < k; ++p) {
                sum += a[p] * b[p];
            }
            C[i * c + j] = sum;
        }
    }
}

KFI_INLINE void kfi_mat_mul_atb(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                                const kf_real *KF_RESTRICT B, int r, int k, int c) {
    for (int i = 0; i < r * c; ++i) {
        C[i] = 0;
    }
    for (int p = 0; p < k; ++p) {
        const kf_real *ap = &A[p * r];
        const kf_real *bp = &B[p * c];
        for (int i = 0; i < r; ++i) {
            const kf_real a = ap[i];
            kf_real *ci = &C[i * c];
            for (int j = 0; j < c; ++j) {
                ci[j] += a * bp[j];
            }
        }
    }
}

KFI_INLINE void kfi_mat_transpose(kf_real *KF_RESTRICT At, const kf_real *KF_RESTRICT A, int r,
                                  int c) {
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            At[j * r + i] = A[i * c + j];
        }
    }
}

KFI_INLINE void kfi_mat_add(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    /* Separate row and column loops rather than one loop to r * c: the result
     * is identical, and the static analyzer can then see that every element
     * is written whenever r and c are positive. */
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            C[i * c + j] = A[i * c + j] + B[i * c + j];
        }
    }
}

KFI_INLINE void kfi_mat_sub(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    /* Separate row and column loops rather than one loop to r * c: the result
     * is identical, and the static analyzer can then see that every element
     * is written whenever r and c are positive. */
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            C[i * c + j] = A[i * c + j] - B[i * c + j];
        }
    }
}

KFI_INLINE void kfi_mat_identity(kf_real *A, int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A[i * n + j] = (i == j) ? (kf_real)1 : (kf_real)0;
        }
    }
}

KFI_INLINE void kfi_mat_symmetrize(kf_real *A, int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            kf_real avg = (A[i * n + j] + A[j * n + i]) * (kf_real)0.5;
            A[i * n + j] = avg;
            A[j * n + i] = avg;
        }
    }
}

KFI_INLINE int kfi_cholesky(kf_real *L, const kf_real *A, int n) {
    for (int j = 0; j < n; ++j) {
        kf_real d = A[j * n + j];
        for (int p = 0; p < j; ++p) {
            d -= L[j * n + p] * L[j * n + p];
        }
        if (!(d > (kf_real)0) || !isfinite(d)) { /* also catches NaN */
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

/* Triangular solves given the reciprocals rd[i] = 1 / L[i][i], so the filter
 * divides once per diagonal entry instead of once per solved element. */
KFI_INLINE void kfi_solve_lower_rd(kf_real *X, const kf_real *L, const kf_real *rd,
                                   const kf_real *B, int n, int c) {
    for (int col = 0; col < c; ++col) {
        for (int i = 0; i < n; ++i) {
            kf_real s = B[i * c + col];
            for (int p = 0; p < i; ++p) {
                s -= L[i * n + p] * X[p * c + col];
            }
            X[i * c + col] = s * rd[i];
        }
    }
}

KFI_INLINE void kfi_solve_lower_t_rd(kf_real *X, const kf_real *L, const kf_real *rd,
                                     const kf_real *B, int n, int c) {
    for (int col = 0; col < c; ++col) {
        for (int i = n - 1; i >= 0; --i) {
            kf_real s = B[i * c + col];
            for (int p = i + 1; p < n; ++p) {
                s -= L[p * n + i] * X[p * c + col];
            }
            X[i * c + col] = s * rd[i];
        }
    }
}

KFI_INLINE void kfi_solve_lower(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
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

KFI_INLINE void kfi_solve_lower_t(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
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

KFI_INLINE void kfi_cholesky_solve(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    kfi_solve_lower(X, L, B, n, c);
    kfi_solve_lower_t(X, L, X, n, c);
}

KFI_INLINE int kfi_is_symmetric(const kf_real *A, int n, kf_real tol) {
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

KFI_INLINE int kfi_cholesky_ok(const kf_real *A, int n) {
    kf_real L[KF_MAX_DIM * KF_MAX_DIM];
    if (n < 1 || n > KF_MAX_DIM) {
        return 0;
    }
    return kfi_cholesky(L, A, n) == (int)KF_OK;
}

/* For a zero UD pivot at column j: 1 if the entries above it are negligible. */
KFI_INLINE int kfi_ud_column_negligible(const kf_real *W, int n, int j, kf_real off_tol) {
    for (int k = 0; k < j; ++k) {
        if (!((W[k * n + j] <= off_tol) && (W[k * n + j] >= -off_tol))) {
            return 0;
        }
    }
    return 1;
}

/* Eliminate column j with pivot d: fill U's column j and update the upper
 * triangle of the leading j x j block of W. */
KFI_INLINE void kfi_ud_eliminate(kf_real *W, kf_real *U, int n, int j, kf_real d) {
    const kf_real inv = (kf_real)1 / d;
    for (int k = 0; k < j; ++k) {
        const kf_real beta = W[k * n + j];
        U[k * n + j] = beta * inv;
        for (int i = 0; i <= k; ++i) {
            W[i * n + k] -= beta * U[i * n + j];
        }
    }
}

/*
 * UD factorization A = U diag(D) U^T with U unit upper triangular, for a
 * symmetric A (n x n).
 *
 * With psd == 0 every pivot must be positive; tiny ones are fine, since a
 * well-conditioned A is not required (that is the point of the UD filter).
 * With psd != 0, a pivot within rounding of zero (|d| <= tol, relative to the
 * largest diagonal entry) is accepted when the entries above it are also
 * negligible: D[j] keeps a tiny positive value, or 0, and U's column is zero.
 *
 * Returns KF_ERR_NOT_POSITIVE_DEFINITE when A is indefinite, has a
 * non-positive pivot with psd == 0, or is not finite.
 */
KFI_INLINE int kfi_ud_factor(kf_real *U, kf_real *D, const kf_real *A, int n, int psd) {
    kf_real W[KF_MAX_DIM * KF_MAX_DIM];
    kf_real max_diag = (kf_real)0;
    for (int i = 0; i < n * n; ++i) {
        W[i] = A[i];
    }
    for (int i = 0; i < n; ++i) {
        if (A[i * n + i] > max_diag) {
            max_diag = A[i * n + i];
        }
    }
    const kf_real tol = (kf_real)n * KFI_EPS * max_diag;
    const kf_real off_tol = KF_SQRT((kf_real)n * KFI_EPS) * max_diag;

    kfi_mat_identity(U, n);
    for (int j = n - 1; j >= 0; --j) {
        const kf_real d = W[j * n + j];
        if (!isfinite(d) || (d < -tol) || ((psd == 0) && !(d > (kf_real)0))) {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        if ((d > tol) || (psd == 0)) {
            D[j] = d;
            kfi_ud_eliminate(W, U, n, j, d);
        } else if (kfi_ud_column_negligible(W, n, j, off_tol) != 0) {
            D[j] = (d > (kf_real)0) ? d : (kf_real)0;
        } else {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
    }
    return KF_OK;
}

/* A value is NaN or Inf exactly when its exponent bits are all ones, that is
 * when exp_mask & ~bits is zero. Tracking the minimum of that over the input
 * uses independent integer operations (no floating-point dependency chain) and
 * stays correct under -ffast-math.
 * MISRA C:2012 Rule 21.15 deviation: memcpy between kf_real and an unsigned
 * integer of the same size is the defined way to read the bit pattern. */
KFI_INLINE int kfi_all_finite(const kf_real *v, int len) {
#ifdef KF_USE_DOUBLE
    const uint64_t exp_mask = 0x7FF0000000000000ULL;
    uint64_t bits;
    uint64_t min_missing = exp_mask;
#else
    const uint32_t exp_mask = 0x7F800000U;
    uint32_t bits;
    uint32_t min_missing = exp_mask;
#endif
    for (int i = 0; i < len; ++i) {
        (void)memcpy(&bits, &v[i], sizeof bits); /* misra-c2012-21.15 deviation */
        if ((exp_mask & ~bits) < min_missing) {
            min_missing = exp_mask & ~bits;
        }
    }
    return min_missing != 0U;
}

#endif

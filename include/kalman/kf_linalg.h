#ifndef KF_LINALG_H
#define KF_LINALG_H

/**
 * @file kf_linalg.h
 * @brief Small matrix routines on flat, row-major ::kf_real arrays.
 *
 * Nothing allocates. Unless noted, outputs must not alias inputs. Square
 * matrices passed to the Cholesky routines are at most KF_MAX_DIM wide.
 */

#include "kalman/kf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** C (r x c) = A (r x k) * B (k x c). */
void kf_mat_mul(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A, const kf_real *KF_RESTRICT B,
                int r, int k, int c);

/** C (r x c) = A (r x k) * B^T, with B stored as (c x k). */
void kf_mat_mul_abt(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                    const kf_real *KF_RESTRICT B, int r, int k, int c);

/** C (r x c) = A^T * B, with A stored as (k x r) and B as (k x c). */
void kf_mat_mul_atb(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                    const kf_real *KF_RESTRICT B, int r, int k, int c);

/** At (c x r) = A (r x c)^T. */
void kf_mat_transpose(kf_real *KF_RESTRICT At, const kf_real *KF_RESTRICT A, int r, int c);

/** C = A + B on r x c matrices. C may alias A or B. */
void kf_mat_add(kf_real *C, const kf_real *A, const kf_real *B, int r, int c);
/** C = A - B on r x c matrices. C may alias A or B. */
void kf_mat_sub(kf_real *C, const kf_real *A, const kf_real *B, int r, int c);

/** A (n x n) = I. */
void kf_mat_identity(kf_real *A, int n);

/** A = (A + A^T) / 2, in place. */
void kf_mat_symmetrize(kf_real *A, int n);

/**
 * Cholesky factorization: lower-triangular L with A = L L^T. L may alias A.
 * @return KF_OK, or KF_ERR_NOT_POSITIVE_DEFINITE (also for NaN/Inf input).
 */
int kf_cholesky(kf_real *L, const kf_real *A, int n);

/** Solve L X = B by forward substitution, for X (n x c); L lower-triangular. X may alias B. */
void kf_solve_lower(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);
/** Solve L^T X = B by back substitution, for X (n x c); L lower-triangular. X may alias B. */
void kf_solve_lower_t(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);

/** Solve A X = B given the Cholesky factor L of A. X may alias B. */
void kf_cholesky_solve(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);

/** @return 1 if |A[i][j] - A[j][i]| <= tol for all i, j; otherwise 0. */
int kf_is_symmetric(const kf_real *A, int n, kf_real tol);
/** @return 1 if A is positive-definite (its Cholesky factorization succeeds); otherwise 0. */
int kf_cholesky_ok(const kf_real *A, int n);
/** @return 1 if all len values are finite (no NaN or Inf); otherwise 0. */
int kf_all_finite(const kf_real *v, int len);

#ifdef __cplusplus
}
#endif

#endif

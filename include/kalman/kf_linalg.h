#ifndef KF_LINALG_H
#define KF_LINALG_H

/*
 * Small fixed-size matrix routines on flat, row-major kf_real arrays.
 * Nothing allocates. Unless noted, outputs must not alias inputs.
 * Square matrices passed to the Cholesky routines are at most KF_MAX_DIM wide.
 */

#include "kalman/kf_types.h"

/* C (r x c) = A (r x k) * B (k x c) */
void kf_mat_mul(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A, const kf_real *KF_RESTRICT B,
                int r, int k, int c);

/* C (r x c) = A (r x k) * B^T, with B stored as (c x k) */
void kf_mat_mul_abt(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                    const kf_real *KF_RESTRICT B, int r, int k, int c);

/* C (r x c) = A^T * B, with A stored as (k x r) and B as (k x c) */
void kf_mat_mul_atb(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A,
                    const kf_real *KF_RESTRICT B, int r, int k, int c);

/* At (c x r) = A (r x c)^T */
void kf_mat_transpose(kf_real *KF_RESTRICT At, const kf_real *KF_RESTRICT A, int r, int c);

/* C = A + B and C = A - B on r x c matrices. C may alias A or B. */
void kf_mat_add(kf_real *C, const kf_real *A, const kf_real *B, int r, int c);
void kf_mat_sub(kf_real *C, const kf_real *A, const kf_real *B, int r, int c);

/* A (n x n) = I */
void kf_mat_identity(kf_real *A, int n);

/* A = (A + A^T) / 2, in place. */
void kf_mat_symmetrize(kf_real *A, int n);

/* Lower-triangular L with A = L * L^T. Returns KF_ERR_NOT_POSITIVE_DEFINITE on failure.
 * L may alias A. */
int kf_cholesky(kf_real *L, const kf_real *A, int n);

/* Solve L * X = B (forward substitution) and L^T * X = B (back substitution)
 * for X (n x c), with L lower-triangular. X may alias B. */
void kf_solve_lower(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);
void kf_solve_lower_t(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);

/* Solve A * X = B given the Cholesky factor L of A. X may alias B. */
void kf_cholesky_solve(kf_real *X, const kf_real *L, const kf_real *B, int n, int c);

/* Predicates: return 1 if true, 0 otherwise. */
int kf_is_symmetric(const kf_real *A, int n, kf_real tol);
int kf_cholesky_ok(const kf_real *A, int n);
int kf_all_finite(const kf_real *v, int len);

#endif

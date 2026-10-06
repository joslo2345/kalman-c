#include "kf_linalg_impl.h"

/* Public entry points; the kernels live in kf_linalg_impl.h. */

void kf_mat_mul(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A, const kf_real *KF_RESTRICT B, int r, int k, int c) {
    kfi_mat_mul(C, A, B, r, k, c);
}

void kf_mat_mul_abt(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A, const kf_real *KF_RESTRICT B, int r, int k, int c) {
    kfi_mat_mul_abt(C, A, B, r, k, c);
}

void kf_mat_mul_atb(kf_real *KF_RESTRICT C, const kf_real *KF_RESTRICT A, const kf_real *KF_RESTRICT B, int r, int k, int c) {
    kfi_mat_mul_atb(C, A, B, r, k, c);
}

void kf_mat_transpose(kf_real *KF_RESTRICT At, const kf_real *KF_RESTRICT A, int r, int c) {
    kfi_mat_transpose(At, A, r, c);
}

void kf_mat_add(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    kfi_mat_add(C, A, B, r, c);
}

void kf_mat_sub(kf_real *C, const kf_real *A, const kf_real *B, int r, int c) {
    kfi_mat_sub(C, A, B, r, c);
}

void kf_mat_identity(kf_real *A, int n) {
    kfi_mat_identity(A, n);
}

void kf_mat_symmetrize(kf_real *A, int n) {
    kfi_mat_symmetrize(A, n);
}

int kf_cholesky(kf_real *L, const kf_real *A, int n) {
    return kfi_cholesky(L, A, n);
}

void kf_solve_lower(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    kfi_solve_lower(X, L, B, n, c);
}

void kf_solve_lower_t(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    kfi_solve_lower_t(X, L, B, n, c);
}

void kf_cholesky_solve(kf_real *X, const kf_real *L, const kf_real *B, int n, int c) {
    kfi_cholesky_solve(X, L, B, n, c);
}

int kf_is_symmetric(const kf_real *A, int n, kf_real tol) {
    return kfi_is_symmetric(A, n, tol);
}

int kf_cholesky_ok(const kf_real *A, int n) {
    return kfi_cholesky_ok(A, n);
}

int kf_all_finite(const kf_real *v, int len) {
    return kfi_all_finite(v, len);
}

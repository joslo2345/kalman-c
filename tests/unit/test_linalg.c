#include "unity.h"
#include "kalman/kalman.h"

#include <float.h>
#include <math.h>

/* Smallest positive subnormal; *_TRUE_MIN is C11, so derive it for C99. */
#define FLT_TRUE_MIN_OR_DENORM (FLT_MIN / 4)
#define DBL_TRUE_MIN_OR_DENORM (DBL_MIN / 4)

#ifdef KF_USE_DOUBLE
#define TOL 1e-9
#else
#define TOL 1e-5f
#endif

static void assert_all_within(const kf_real *expected, const kf_real *actual, int len) {
    for (int i = 0; i < len; ++i) {
        TEST_ASSERT_DOUBLE_WITHIN(TOL * 10, (double)expected[i], (double)actual[i]);
    }
}

/* Classic SPD example with an integer Cholesky factor. */
static const kf_real A3[9] = {4, 12, -16, 12, 37, -43, -16, -43, 98};
static const kf_real L3[9] = {2, 0, 0, 6, 1, 0, -8, 5, 3};

void setUp(void) {}
void tearDown(void) {}

void test_mul_rectangular(void) {
    const kf_real A[6] = {1, 2, 3, 4, 5, 6};    /* 2x3 */
    const kf_real B[6] = {7, 8, 9, 10, 11, 12}; /* 3x2 */
    const kf_real expected[4] = {58, 64, 139, 154};
    kf_real C[4];
    kf_mat_mul(C, A, B, 2, 3, 2);
    assert_all_within(expected, C, 4);
}

void test_transpose_rectangular(void) {
    const kf_real A[6] = {1, 2, 3, 4, 5, 6};
    const kf_real expected[6] = {1, 4, 2, 5, 3, 6};
    kf_real At[6];
    kf_mat_transpose(At, A, 2, 3);
    assert_all_within(expected, At, 6);
}

void test_add_sub_identity(void) {
    const kf_real A[4] = {1, 2, 3, 4};
    const kf_real expected_sub[4] = {0, 2, 3, 3};
    kf_real I[4], C[4];
    kf_mat_identity(I, 2);
    kf_mat_sub(C, A, I, 2, 2);
    assert_all_within(expected_sub, C, 4);
    kf_mat_add(C, C, I, 2, 2); /* in place */
    assert_all_within(A, C, 4);
}

void test_symmetrize(void) {
    kf_real A[4] = {1, 2, 4, 5};
    kf_mat_symmetrize(A, 2);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 3.0, (double)A[1]);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 3.0, (double)A[2]);
    TEST_ASSERT_TRUE(kf_is_symmetric(A, 2, 0));
}

void test_cholesky_known_factor(void) {
    kf_real L[9];
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_cholesky(L, A3, 3));
    assert_all_within(L3, L, 9);
}

void test_cholesky_in_place(void) {
    kf_real A[9];
    for (int i = 0; i < 9; ++i) A[i] = A3[i];
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_cholesky(A, A, 3));
    assert_all_within(L3, A, 9);
}

void test_cholesky_rejects_indefinite_and_singular(void) {
    const kf_real indefinite[4] = {1, 2, 2, 1};
    const kf_real singular[4] = {1, 1, 1, 1};
    const kf_real zero[4] = {0, 0, 0, 0};
    kf_real L[4];
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_cholesky(L, indefinite, 2));
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_cholesky(L, singular, 2));
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_cholesky(L, zero, 2));
    TEST_ASSERT_FALSE(kf_cholesky_ok(indefinite, 2));
    TEST_ASSERT_TRUE(kf_cholesky_ok(A3, 3));
}

void test_cholesky_rejects_nan(void) {
    const kf_real A[4] = {NAN, 0, 0, 1};
    kf_real L[4];
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_cholesky(L, A, 2));
}

void test_cholesky_solve_recovers_x(void) {
    const kf_real x[6] = {1, -1, 2, 0, 3, 1}; /* 3x2 */
    kf_real B[6], L[9], X[6];
    kf_mat_mul(B, A3, x, 3, 3, 2);
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_cholesky(L, A3, 3));
    kf_cholesky_solve(X, L, B, 3, 2);
    assert_all_within(x, X, 6);
    kf_cholesky_solve(B, L, B, 3, 2); /* in place */
    assert_all_within(x, B, 6);
}

void test_is_symmetric_tolerance(void) {
    const kf_real A[4] = {1, 2, (kf_real)2.001, 1};
    TEST_ASSERT_TRUE(kf_is_symmetric(A, 2, (kf_real)0.01));
    TEST_ASSERT_FALSE(kf_is_symmetric(A, 2, (kf_real)0.0001));
}

void test_all_finite(void) {
    const kf_real ok[3] = {1, -2, 0};
    const kf_real bad_nan[3] = {1, NAN, 0};
    const kf_real bad_inf[3] = {1, 0, INFINITY};
    TEST_ASSERT_TRUE(kf_all_finite(ok, 3));
    TEST_ASSERT_FALSE(kf_all_finite(bad_nan, 3));
    TEST_ASSERT_FALSE(kf_all_finite(bad_inf, 3));
}

void test_all_finite_boundaries(void) {
#ifdef KF_USE_DOUBLE
    const kf_real largest = DBL_MAX, tiny = DBL_TRUE_MIN_OR_DENORM;
#else
    const kf_real largest = FLT_MAX, tiny = FLT_TRUE_MIN_OR_DENORM;
#endif
    const kf_real edge[5] = {largest, -largest, tiny, -tiny, (kf_real)-0.0};
    const kf_real neg_inf[2] = {1, -INFINITY};
    const kf_real neg_nan[2] = {-NAN, 1};
    TEST_ASSERT_TRUE(kf_all_finite(edge, 5));
    TEST_ASSERT_FALSE(kf_all_finite(neg_inf, 2));
    TEST_ASSERT_FALSE(kf_all_finite(neg_nan, 2));
    TEST_ASSERT_TRUE(kf_all_finite(edge, 0)); /* empty input is trivially finite */
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mul_rectangular);
    RUN_TEST(test_transpose_rectangular);
    RUN_TEST(test_add_sub_identity);
    RUN_TEST(test_symmetrize);
    RUN_TEST(test_cholesky_known_factor);
    RUN_TEST(test_cholesky_in_place);
    RUN_TEST(test_cholesky_rejects_indefinite_and_singular);
    RUN_TEST(test_cholesky_rejects_nan);
    RUN_TEST(test_cholesky_solve_recovers_x);
    RUN_TEST(test_is_symmetric_tolerance);
    RUN_TEST(test_all_finite);
    RUN_TEST(test_all_finite_boundaries);
    return UNITY_END();
}

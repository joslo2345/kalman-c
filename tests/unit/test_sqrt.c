#include "kalman/kalman.h"
#include "kf_linalg_impl.h"
#include "scenarios/constant_velocity.h"
#include "unity.h"

#include <math.h>
#include <string.h>

#ifdef KF_USE_DOUBLE
#define TOL 1e-9
#else
#define TOL 1e-5
#endif

static cv_scenario sc;

void setUp(void) {}
void tearDown(void) {}

static void assert_near(double expected, kf_real actual, double tol) {
    TEST_ASSERT_DOUBLE_WITHIN(tol * (1.0 + fabs(expected)), expected, (double)actual);
}

/* ---- Kernels ---- */

void test_ud_factor_reconstructs_and_handles_semidefinite(void) {
    const kf_real pd[9] = {4, 2, 0.6f, 2, 5, 1, 0.6f, 1, 3};
    const kf_real psd[9] = {1, 0, 0.5f, 0, 0, 0, 0.5f, 0, 4.25f};
    const kf_real indefinite[4] = {0, 1, 1, 0};
    const kf_real negative[4] = {1, 0, 0, -1};
    kf_real U[9], D[3], UD[9], P[9];

    for (int t = 0; t < 2; ++t) {
        const kf_real *A = t == 0 ? pd : psd;
        TEST_ASSERT_EQUAL_INT(KF_OK, kfi_ud_factor(U, D, A, 3, t));
        for (int i = 0; i < 3; ++i) {
            TEST_ASSERT_EQUAL_FLOAT(1, U[i * 3 + i]);
            TEST_ASSERT_TRUE(D[i] >= 0);
            for (int j = 0; j < i; ++j) {
                TEST_ASSERT_EQUAL_FLOAT(0, U[i * 3 + j]);
            }
            for (int j = 0; j < 3; ++j) {
                UD[i * 3 + j] = U[i * 3 + j] * D[j];
            }
        }
        kf_mat_mul_abt(P, UD, U, 3, 3, 3);
        for (int i = 0; i < 9; ++i) {
            assert_near((double)A[i], P[i], TOL);
        }
    }
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kfi_ud_factor(U, D, psd, 3, 0));

    /* Wide dynamic range: tiny variances must be kept, not rejected or zeroed. */
    const kf_real wide[9] = {1, 0, 0, 0, 1e-6f, 0, 0, 0, 1e-12f};
    for (int t = 0; t < 2; ++t) {
        TEST_ASSERT_EQUAL_INT(KF_OK, kfi_ud_factor(U, D, wide, 3, t));
        assert_near(1e-6, D[1], TOL);
        assert_near(1e-12, D[2], TOL);
    }
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kfi_ud_factor(U, D, indefinite, 2, 1));
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kfi_ud_factor(U, D, negative, 2, 1));
}

/* ---- Filter ---- */

static void setup_pair(kf_state *kf, kf_sr_state *sr) {
    cv_scenario_setup_kf(&sc, kf);
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_init(sr, CV_N, CV_M));
    memcpy(sr->x, sc.x0_hat, sizeof sc.x0_hat);
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_P(sr, sc.P0));
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_Q(sr, sc.Q));
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_R(sr, sc.R));
}

void test_square_root_filter_matches_joseph_filter(void) {
    kf_state kf;
    kf_sr_state sr;
    kf_real P[CV_N * CV_N];
    cv_scenario_init(&sc, 1000, 42);
    setup_pair(&kf, &sr);

    for (int k = 0; k < sc.steps; ++k) {
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, sc.z[k], sc.H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_predict(&sr, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_update(&sr, sc.z[k], sc.H));
        for (int i = 0; i < CV_N; ++i) {
            assert_near((double)kf.x[i], sr.x[i], TOL * 5);
        }
    }
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_get_P(&sr, P));
    for (int i = 0; i < CV_N * CV_N; ++i) {
        assert_near((double)kf.P[i], P[i], TOL * 5);
    }
    assert_near((double)kf.nis, sr.nis, TOL * 50);
    for (int i = 0; i < CV_N; ++i) { /* U stays unit upper triangular, D non-negative */
        TEST_ASSERT_TRUE(sr.D[i] >= 0);
        TEST_ASSERT_EQUAL_FLOAT(1, sr.U[i * CV_N + i]);
        for (int j = 0; j < i; ++j) {
            TEST_ASSERT_EQUAL_FLOAT(0, sr.U[i * CV_N + j]);
        }
    }
}

/* h(x) = |x|, as in test_ekf: x = [3, 4], P = I, R = 1, z = 6 */
static int range_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    const kf_real r = (kf_real)sqrt((double)(x[0] * x[0] + x[1] * x[1]));
    if (r == 0) {
        return 1;
    }
    z_out[0] = r;
    H_out[0] = x[0] / r;
    H_out[1] = x[1] / r;
    return 0;
}

void test_square_root_ekf_update_matches_hand_computation(void) {
    kf_sr_state sr;
    const kf_real I2[4] = {1, 0, 0, 1};
    const kf_real R[1] = {1};
    const kf_real z[1] = {6};
    kf_real P[4];
    kf_sr_init(&sr, 2, 1);
    sr.x[0] = 3;
    sr.x[1] = 4;
    kf_sr_set_P(&sr, I2);
    kf_sr_set_R(&sr, R);

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_ekf_update(&sr, z, range_h, NULL));
    kf_sr_get_P(&sr, P);
    assert_near(3.3, sr.x[0], TOL);
    assert_near(4.4, sr.x[1], TOL);
    assert_near(0.82, P[0], TOL);
    assert_near(-0.24, P[1], TOL);
    assert_near(0.68, P[3], TOL);
    assert_near(0.5, sr.nis, TOL);
}

/* ---- Errors ---- */

void test_setters_reject_bad_covariances_and_leave_state_unchanged(void) {
    kf_sr_state sr;
    const kf_real indefinite[4] = {1, 2, 2, 1};
    const kf_real nan_m[4] = {1, 0, 0, NAN};
    kf_sr_init(&sr, 2, 2);
    const kf_real I2[4] = {1, 0, 0, 1};
    kf_sr_set_P(&sr, I2);
    kf_sr_set_R(&sr, I2);
    kf_sr_state before = sr;

    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_sr_set_P(&sr, indefinite));
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_sr_set_Q(&sr, indefinite));
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_sr_set_R(&sr, indefinite));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_sr_set_P(&sr, nan_m));
    TEST_ASSERT_EQUAL_MEMORY(&before, &sr, sizeof sr);
}

void test_bad_inputs_are_rejected_and_state_unchanged(void) {
    kf_sr_state sr;
    const kf_real I2[4] = {1, 0, 0, 1};
    const kf_real F[4] = {1, 1, 0, 1};
    const kf_real nan_z[2] = {NAN, 1};
    const kf_real z[2] = {1, 1};
    kf_sr_init(&sr, 2, 2);
    kf_sr_set_P(&sr, I2);
    kf_sr_set_R(&sr, I2);
    kf_sr_state before = sr;

    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_sr_update(&sr, nan_z, I2));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_sr_predict(NULL, F));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_sr_update(&sr, z, NULL));
    sr.n = KF_MAX_STATE + 1; /* corrupted size */
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_sr_predict(&sr, F));
    sr.n = 2;
    TEST_ASSERT_EQUAL_MEMORY(&before, &sr, sizeof sr);
}

void test_singular_innovation_covariance_returns_error_and_state_unchanged(void) {
    kf_sr_state sr;
    const kf_real I2[4] = {1, 0, 0, 1};
    const kf_real z[2] = {1, 1};
    kf_sr_init(&sr, 2, 2); /* D = 0 and Dr = 0: no measurement variance at all */
    kf_sr_state before = sr;

    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_sr_update(&sr, z, I2));
    TEST_ASSERT_EQUAL_MEMORY(&before, &sr, sizeof sr);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ud_factor_reconstructs_and_handles_semidefinite);
    RUN_TEST(test_square_root_filter_matches_joseph_filter);
    RUN_TEST(test_square_root_ekf_update_matches_hand_computation);
    RUN_TEST(test_setters_reject_bad_covariances_and_leave_state_unchanged);
    RUN_TEST(test_bad_inputs_are_rejected_and_state_unchanged);
    RUN_TEST(test_singular_innovation_covariance_returns_error_and_state_unchanged);
    return UNITY_END();
}

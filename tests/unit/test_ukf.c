#include "kalman/kalman.h"
#include "unity.h"

#include <math.h>
#include <string.h>

#ifdef KF_USE_DOUBLE
#define TOL 1e-9
#else
#define TOL 1e-5
#endif

void setUp(void) {}
void tearDown(void) {}

static void assert_near(double expected, kf_real actual) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, expected, (double)actual);
}

/* ---- Linear models: the unscented transform is exact, so the UKF must match the KF ---- */

typedef struct {
    const kf_real *F;
    const kf_real *H;
} linear_ctx;

static int linear_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    const linear_ctx *c = ctx;
    kf_mat_mul(x_out, c->F, x, n, n, 1);
    return 0;
}

static int linear_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    const linear_ctx *c = ctx;
    kf_mat_mul(z_out, c->H, x, m, n, 1);
    return 0;
}

void test_ukf_with_linear_model_matches_linear_kf(void) {
    const kf_real dt = (kf_real)0.1;
    const kf_real F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
    const kf_real H[8] = {1, 0, 0, 0, 0, 1, 0, 0};
    linear_ctx ctx = {F, H};
    kf_state kf, ukf;

    kf_init(&kf, 4, 2);
    kf_mat_identity(kf.P, 4);
    for (int i = 0; i < 4; ++i)
        kf.Q[i * 4 + i] = (kf_real)1e-3;
    kf.R[0] = kf.R[3] = (kf_real)0.25;
    ukf = kf;

    for (int k = 0; k < 1000; ++k) {
        const kf_real t = dt * (kf_real)k;
        const kf_real z[2] = {t + (kf_real)0.3 * (kf_real)sin(k),
                              2 * t - (kf_real)0.2 * (kf_real)cos(k)};
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_predict(&ukf, NULL, linear_f, &ctx));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_update(&ukf, z, NULL, linear_h, &ctx));

        for (int i = 0; i < 4; ++i) {
            TEST_ASSERT_DOUBLE_WITHIN(TOL * 100, (double)kf.x[i], (double)ukf.x[i]);
        }
    }
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_DOUBLE_WITHIN(TOL * 100, (double)kf.P[i], (double)ukf.P[i]);
    }
    TEST_ASSERT_DOUBLE_WITHIN(TOL * 100, (double)kf.nis, (double)ukf.nis);
}

/* ---- Nonlinear: x ~ N(1, 1) through x^2 has mean 2 and variance 6 exactly ---- */

static int square_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    (void)n;
    (void)ctx;
    x_out[0] = x[0] * x[0];
    return 0;
}

static int square_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    z_out[0] = x[0] * x[0];
    return 0;
}

void test_predict_through_square_matches_exact_moments(void) {
    kf_state kf;
    kf_init(&kf, 1, 1);
    kf.x[0] = 1;
    kf.P[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_predict(&kf, NULL, square_f, NULL));
    assert_near(2.0, kf.x[0]);
    assert_near(6.0, kf.P[0]);
}

void test_update_through_square_matches_hand_computation(void) {
    kf_state kf;
    const kf_real z[1] = {3};
    kf_init(&kf, 1, 1);
    kf.x[0] = 1;
    kf.P[0] = 1;
    kf.R[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_update(&kf, z, NULL, square_h, NULL));
    /* z_hat = 2, S = 7, Pxz = 2, K = 2/7 */
    assert_near(9.0 / 7.0, kf.x[0]);
    assert_near(3.0 / 7.0, kf.P[0]);
    assert_near(1.0 / 7.0, kf.nis);
}

void test_custom_params_kappa_three_minus_n_gives_exact_moments(void) {
    /* alpha = 1, beta = 0, kappa = 2: c = 3, points 1, 1 +/- sqrt(3) */
    const kf_ukf_params p = {1, 0, 2};
    kf_state kf;
    kf_init(&kf, 1, 1);
    kf.x[0] = 1;
    kf.P[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_predict(&kf, &p, square_f, NULL));
    /* kappa = 3 - n matches the fourth moment of a Gaussian, so both moments stay exact. */
    assert_near(2.0, kf.x[0]);
    assert_near(6.0, kf.P[0]);
}

/* ---- Range-bearing tracking ---- */

static int cv_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    const kf_real dt = *(const kf_real *)ctx;
    (void)n;
    x_out[0] = x[0] + dt * x[2];
    x_out[1] = x[1] + dt * x[3];
    x_out[2] = x[2];
    x_out[3] = x[3];
    return 0;
}

static int range_bearing_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    const double px = x[0], py = x[1];
    z_out[0] = (kf_real)sqrt(px * px + py * py);
    z_out[1] = (kf_real)atan2(py, px);
    return 0;
}

static unsigned lcg_state = 2024u;
static double noise(void) { /* uniform in [-0.5, 0.5) */
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (double)(lcg_state >> 8) / 16777216.0 - 0.5;
}

void test_range_bearing_tracking_converges(void) {
    kf_real dt = (kf_real)0.1;
    kf_state kf;
    kf_init(&kf, 4, 2);
    kf.x[0] = 9; /* truth starts at (10, 5) moving at (1, -0.5) */
    kf.x[1] = 6;
    kf_mat_identity(kf.P, 4);
    kf.P[0] = kf.P[5] = 4;
    for (int i = 0; i < 4; ++i)
        kf.Q[i * 4 + i] = (kf_real)1e-4;
    kf.R[0] = (kf_real)0.01;
    kf.R[3] = (kf_real)1e-4;

    double px = 10, py = 5;
    for (int k = 0; k < 500; ++k) {
        px += dt * 1.0;
        py += dt * -0.5;
        kf_real z[2] = {(kf_real)(sqrt(px * px + py * py) + 0.2 * noise()),
                        (kf_real)(atan2(py, px) + 0.02 * noise())};
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_predict(&kf, NULL, cv_f, &dt));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_update(&kf, z, NULL, range_bearing_h, NULL));
    }
    TEST_ASSERT_DOUBLE_WITHIN(0.2, px, (double)kf.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(0.2, py, (double)kf.x[1]);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 1.0, (double)kf.x[2]);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, -0.5, (double)kf.x[3]);
    TEST_ASSERT_TRUE(kf_cholesky_ok(kf.P, 4));
}

/* ---- Errors ---- */

static int failing_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    (void)x_out;
    (void)x;
    (void)n;
    (void)ctx;
    return 1;
}

static int failing_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    (void)z_out;
    (void)x;
    (void)n;
    (void)m;
    (void)ctx;
    return 1;
}

static int nan_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    (void)x;
    (void)n;
    (void)m;
    (void)ctx;
    z_out[0] = NAN;
    return 0;
}

static void setup_1d(kf_state *kf) {
    kf_init(kf, 1, 1);
    kf->x[0] = 1;
    kf->P[0] = 1;
    kf->R[0] = 1;
}

void test_model_failure_returns_error_and_state_unchanged(void) {
    kf_state kf;
    const kf_real z[1] = {1};
    setup_1d(&kf);
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_MODEL_FAILED, kf_ukf_predict(&kf, NULL, failing_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_MODEL_FAILED, kf_ukf_update(&kf, z, NULL, failing_h, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_update(&kf, z, NULL, nan_h, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_non_positive_definite_covariance_returns_error_and_state_unchanged(void) {
    kf_state kf;
    setup_1d(&kf);
    kf.P[0] = 0; /* no Cholesky factor, so no sigma points */
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_ukf_predict(&kf, NULL, square_f, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_invalid_params_are_rejected_and_state_unchanged(void) {
    const kf_ukf_params zero_spread = {1, 2, -1}; /* n + kappa = 0 */
    const kf_ukf_params nan_alpha = {NAN, 2, 0};
    kf_state kf;
    const kf_real z[1] = {1};
    setup_1d(&kf);
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_predict(&kf, &zero_spread, square_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_update(&kf, z, &nan_alpha, square_h, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_null_arguments_and_nan_measurement_are_rejected(void) {
    kf_state kf;
    const kf_real z[1] = {1};
    const kf_real z_nan[1] = {NAN};
    setup_1d(&kf);
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_predict(NULL, NULL, square_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_predict(&kf, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_update(&kf, NULL, NULL, square_h, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_update(&kf, z, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ukf_update(&kf, z_nan, NULL, square_h, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ukf_with_linear_model_matches_linear_kf);
    RUN_TEST(test_predict_through_square_matches_exact_moments);
    RUN_TEST(test_update_through_square_matches_hand_computation);
    RUN_TEST(test_custom_params_kappa_three_minus_n_gives_exact_moments);
    RUN_TEST(test_range_bearing_tracking_converges);
    RUN_TEST(test_model_failure_returns_error_and_state_unchanged);
    RUN_TEST(test_non_positive_definite_covariance_returns_error_and_state_unchanged);
    RUN_TEST(test_invalid_params_are_rejected_and_state_unchanged);
    RUN_TEST(test_null_arguments_and_nan_measurement_are_rejected);
    return UNITY_END();
}

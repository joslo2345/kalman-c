#include "unity.h"
#include "kalman/kalman.h"

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

/* ---- Linear models expressed as EKF callbacks ---- */

typedef struct {
    const kf_real *F;
    const kf_real *H;
} linear_ctx;

static int linear_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    const linear_ctx *c = ctx;
    kf_mat_mul(x_out, c->F, x, n, n, 1);
    memcpy(F_out, c->F, (size_t)(n * n) * sizeof *F_out);
    return 0;
}

static int linear_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    const linear_ctx *c = ctx;
    kf_mat_mul(z_out, c->H, x, m, n, 1);
    memcpy(H_out, c->H, (size_t)(m * n) * sizeof *H_out);
    return 0;
}

void test_ekf_with_linear_model_matches_linear_kf_exactly(void) {
    const kf_real dt = (kf_real)0.1;
    const kf_real F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
    const kf_real H[8] = {1, 0, 0, 0, 0, 1, 0, 0};
    linear_ctx ctx = {F, H};
    kf_state kf, ekf;

    kf_init(&kf, 4, 2);
    kf_mat_identity(kf.P, 4);
    for (int i = 0; i < 4; ++i) kf.Q[i * 4 + i] = (kf_real)1e-3;
    kf.R[0] = kf.R[3] = (kf_real)0.25;
    ekf = kf;

    for (int k = 0; k < 1000; ++k) {
        const kf_real t = dt * (kf_real)k;
        const kf_real z[2] = {t + (kf_real)0.3 * (kf_real)sin(k), 2 * t - (kf_real)0.2 * (kf_real)cos(k)};
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_predict(&ekf, linear_f, &ctx));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&ekf, z, linear_h, &ctx));
    }
    /* Same arithmetic in the same order, so the results are bit-identical. */
    TEST_ASSERT_EQUAL_MEMORY(&kf, &ekf, sizeof kf);
}

/* ---- Nonlinear models ---- */

/* f(x) = [x0 * x1, x1] */
static int product_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)n;
    (void)ctx;
    x_out[0] = x[0] * x[1];
    x_out[1] = x[1];
    F_out[0] = x[1];
    F_out[1] = x[0];
    F_out[2] = 0;
    F_out[3] = 1;
    return 0;
}

/* h(x) = |x| (range to the origin) */
static int range_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    const kf_real r = (kf_real)sqrt((double)(x[0] * x[0] + x[1] * x[1]));
    if (r == 0) {
        return 1; /* Jacobian undefined at the origin */
    }
    z_out[0] = r;
    H_out[0] = x[0] / r;
    H_out[1] = x[1] / r;
    return 0;
}

void test_nonlinear_predict_matches_hand_computation(void) {
    kf_state kf;
    kf_init(&kf, 2, 1);
    kf.x[0] = 2;
    kf.x[1] = 3;
    kf_mat_identity(kf.P, 2);

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_predict(&kf, product_f, NULL));
    assert_near(6.0, kf.x[0]);
    assert_near(3.0, kf.x[1]);
    /* F = [[3, 2], [0, 1]],  F F^T = [[13, 2], [2, 1]] */
    assert_near(13.0, kf.P[0]);
    assert_near(2.0, kf.P[1]);
    assert_near(2.0, kf.P[2]);
    assert_near(1.0, kf.P[3]);
}

void test_range_update_matches_hand_computation(void) {
    kf_state kf;
    const kf_real z[1] = {6};
    kf_init(&kf, 2, 1);
    kf.x[0] = 3;
    kf.x[1] = 4;
    kf_mat_identity(kf.P, 2);
    kf.R[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&kf, z, range_h, NULL));
    /* h = 5, H = [0.6, 0.8], y = 1, S = 2, K = [0.3, 0.4] */
    assert_near(3.3, kf.x[0]);
    assert_near(4.4, kf.x[1]);
    assert_near(0.82, kf.P[0]);
    assert_near(-0.24, kf.P[1]);
    assert_near(-0.24, kf.P[2]);
    assert_near(0.68, kf.P[3]);
    assert_near(0.5, kf.nis);
}

/* ---- Range-bearing tracking from a sensor at the origin ---- */

static int cv_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    const kf_real dt = *(const kf_real *)ctx;
    (void)n;
    x_out[0] = x[0] + dt * x[2];
    x_out[1] = x[1] + dt * x[3];
    x_out[2] = x[2];
    x_out[3] = x[3];
    kf_mat_identity(F_out, 4);
    F_out[0 * 4 + 2] = dt;
    F_out[1 * 4 + 3] = dt;
    return 0;
}

static int range_bearing_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m,
                           void *ctx) {
    (void)m;
    (void)ctx;
    const double px = x[0], py = x[1];
    const double r2 = px * px + py * py;
    const double r = sqrt(r2);
    if (r2 == 0) {
        return 1;
    }
    z_out[0] = (kf_real)r;
    z_out[1] = (kf_real)atan2(py, px);
    H_out[0 * n + 0] = (kf_real)(px / r);
    H_out[0 * n + 1] = (kf_real)(py / r);
    H_out[1 * n + 0] = (kf_real)(-py / r2);
    H_out[1 * n + 1] = (kf_real)(px / r2);
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
    kf.x[0] = 9;  /* truth starts at (10, 5) moving at (1, -0.5) */
    kf.x[1] = 6;
    kf_mat_identity(kf.P, 4);
    kf.P[0] = kf.P[5] = 4;
    for (int i = 0; i < 4; ++i) kf.Q[i * 4 + i] = (kf_real)1e-4;
    kf.R[0] = (kf_real)0.01;
    kf.R[3] = (kf_real)1e-4;

    double px = 10, py = 5;
    for (int k = 0; k < 500; ++k) {
        px += dt * 1.0;
        py += dt * -0.5;
        kf_real z[2] = {(kf_real)(sqrt(px * px + py * py) + 0.2 * noise()),
                        (kf_real)(atan2(py, px) + 0.02 * noise())};
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_predict(&kf, cv_f, &dt));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&kf, z, range_bearing_h, NULL));
    }
    TEST_ASSERT_DOUBLE_WITHIN(0.2, px, (double)kf.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(0.2, py, (double)kf.x[1]);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 1.0, (double)kf.x[2]);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, -0.5, (double)kf.x[3]);
    TEST_ASSERT_TRUE(kf_cholesky_ok(kf.P, 4));
}

/* ---- Errors ---- */

static int failing_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)x_out; (void)F_out; (void)x; (void)n; (void)ctx;
    return 1;
}

static int nan_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)x; (void)ctx;
    kf_mat_identity(F_out, n);
    x_out[0] = NAN;
    return 0;
}

void test_model_failure_returns_error_and_state_unchanged(void) {
    kf_state kf;
    const kf_real z[1] = {1};
    kf_init(&kf, 2, 1); /* x at the origin, where range_h fails */
    kf_mat_identity(kf.P, 2);
    kf.R[0] = 1;
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_MODEL_FAILED, kf_ekf_predict(&kf, failing_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_MODEL_FAILED, kf_ekf_update(&kf, z, range_h, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_nan_from_model_returns_error_and_state_unchanged(void) {
    kf_state kf;
    kf_init(&kf, 2, 1);
    kf_mat_identity(kf.P, 2);
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_predict(&kf, nan_f, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_null_arguments_are_rejected(void) {
    kf_state kf;
    const kf_real z[1] = {1};
    kf_init(&kf, 2, 1);
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_predict(NULL, product_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_predict(&kf, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_update(&kf, NULL, range_h, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_update(&kf, z, NULL, NULL));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ekf_with_linear_model_matches_linear_kf_exactly);
    RUN_TEST(test_nonlinear_predict_matches_hand_computation);
    RUN_TEST(test_range_update_matches_hand_computation);
    RUN_TEST(test_range_bearing_tracking_converges);
    RUN_TEST(test_model_failure_returns_error_and_state_unchanged);
    RUN_TEST(test_nan_from_model_returns_error_and_state_unchanged);
    RUN_TEST(test_null_arguments_are_rejected);
    return UNITY_END();
}

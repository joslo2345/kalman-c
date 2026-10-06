#include "kalman/kalman.h"
#include "unity.h"

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

void test_scalar_update_matches_closed_form(void) {
    kf_state kf;
    const kf_real H[1] = {1};
    const kf_real z[1] = {2};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 1, 1));
    kf.P[0] = 1;
    kf.R[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
    assert_near(1.0, kf.x[0]); /* K = 0.5 */
    assert_near(0.5, kf.P[0]);
    assert_near(2.0, kf.nis); /* y^2 / S = 4 / 2 */
}

void test_scalar_predict_matches_closed_form(void) {
    kf_state kf;
    const kf_real F[1] = {2};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 1, 1));
    kf.x[0] = 1;
    kf.P[0] = (kf_real)0.5;
    kf.Q[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, F));
    assert_near(2.0, kf.x[0]);
    assert_near(3.0, kf.P[0]); /* 4 * 0.5 + 1 */
}

void test_constant_velocity_update_then_predict(void) {
    kf_state kf;
    const kf_real F[4] = {1, 1, 0, 1};
    const kf_real H[2] = {1, 0};
    const kf_real z[1] = {1};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 2, 1));
    kf_mat_identity(kf.P, 2);
    kf.R[0] = 1;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
    assert_near(0.5, kf.x[0]);
    assert_near(0.0, kf.x[1]);
    assert_near(0.5, kf.P[0]);
    assert_near(0.0, kf.P[1]);
    assert_near(1.0, kf.P[3]);

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, F));
    assert_near(0.5, kf.x[0]);
    assert_near(1.5, kf.P[0]);
    assert_near(1.0, kf.P[1]);
    assert_near(1.0, kf.P[2]);
    assert_near(1.0, kf.P[3]);
}

/* With the optimal gain, the Joseph form must equal the textbook (I - KH)P result. */
void test_correlated_update_matches_textbook_form(void) {
    kf_state kf;
    const kf_real H[2] = {1, 1};
    const kf_real z[1] = {4};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 2, 1));
    kf.x[0] = 1;
    kf.x[1] = 2;
    kf.P[0] = 2;
    kf.P[1] = kf.P[2] = (kf_real)0.5;
    kf.P[3] = 1;
    kf.R[0] = (kf_real)0.5;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
    /* y = 1, S = 4.5, K = [5/9, 1/3] */
    assert_near(1.0 + 5.0 / 9.0, kf.x[0]);
    assert_near(2.0 + 1.0 / 3.0, kf.x[1]);
    assert_near(11.0 / 18.0, kf.P[0]);
    assert_near(-1.0 / 3.0, kf.P[1]);
    assert_near(-1.0 / 3.0, kf.P[2]);
    assert_near(0.5, kf.P[3]);
    assert_near(2.0 / 9.0, kf.nis);
}

/* Deterministic pseudo-noise in [-0.5, 0.5) */
static unsigned lcg_state = 12345u;
static kf_real noise(void) {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (kf_real)((double)(lcg_state >> 8) / 16777216.0 - 0.5);
}

void test_covariance_stays_symmetric_positive_definite_over_long_run(void) {
    kf_state kf;
    const kf_real dt = (kf_real)0.1;
    /* 2D constant velocity: state [px, py, vx, vy], measure [px, py] */
    const kf_real F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
    const kf_real H[8] = {1, 0, 0, 0, 0, 1, 0, 0};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 4, 2));
    kf_mat_identity(kf.P, 4);
    for (int i = 0; i < 4; ++i)
        kf.Q[i * 4 + i] = (kf_real)1e-3;
    kf.R[0] = kf.R[3] = (kf_real)0.25;

    for (int k = 0; k < 10000; ++k) {
        const kf_real t = dt * (kf_real)k;
        const kf_real z[2] = {t + noise(), 2 * t + noise()};
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, H));
    }
    TEST_ASSERT_TRUE(kf_is_symmetric(kf.P, 4, (kf_real)1e-6));
    TEST_ASSERT_TRUE(kf_cholesky_ok(kf.P, 4));
    /* Velocities converge to the true [1, 2] */
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 1.0, (double)kf.x[2]);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 2.0, (double)kf.x[3]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_scalar_update_matches_closed_form);
    RUN_TEST(test_scalar_predict_matches_closed_form);
    RUN_TEST(test_constant_velocity_update_then_predict);
    RUN_TEST(test_correlated_update_matches_textbook_form);
    RUN_TEST(test_covariance_stays_symmetric_positive_definite_over_long_run);
    return UNITY_END();
}

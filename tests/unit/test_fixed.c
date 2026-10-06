#include "kalman/kalman.h"
#include "scenarios/constant_velocity.h"
#include "unity.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static cv_scenario sc;

/*
 * Tracking tolerances for Q11.20 (resolution 9.5e-7) on the CV scenario, where
 * positions reach about 100 m. Measured against the float64 filter: state
 * 3.1e-5, covariance 7.0e-6, i.e. a few tens of LSBs. The bounds leave about
 * 3x headroom; the float32 reference itself differs by up to 7e-5.
 */
#if KF_FX_FRAC == 20
#define FX_STATE_TOL 2e-4
#define FX_COV_TOL 2e-5
#else
#define FX_STATE_TOL (2e-4 * (double)(1 << (20 - KF_FX_FRAC)))
#define FX_COV_TOL (2e-5 * (double)(1 << (20 - KF_FX_FRAC)))
#endif

void setUp(void) {}
void tearDown(void) {}

static kf_fx fx(double v) {
    kf_fx out = 0;
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_from_double(v, &out));
    return out;
}

static void fx_matrix(kf_fx *dst, const kf_real *src, int len) {
    for (int i = 0; i < len; ++i)
        dst[i] = fx((double)src[i]);
}

/* ---- Conversions ---- */

void test_conversions_round_to_nearest_and_check_range(void) {
    const double lsb = 1.0 / (double)KF_FX_ONE;
    kf_fx v;
    TEST_ASSERT_EQUAL_INT32(KF_FX_ONE, fx(1.0));
    TEST_ASSERT_EQUAL_INT32(-KF_FX_ONE / 2, fx(-0.5));
    TEST_ASSERT_EQUAL_INT32(1, fx(0.5 * lsb)); /* tie rounds away from zero */
    TEST_ASSERT_EQUAL_INT32(-1, fx(-0.5 * lsb));
    TEST_ASSERT_EQUAL_INT32(0, fx(0.49 * lsb));
    TEST_ASSERT_EQUAL_DOUBLE(1.25, kf_fx_to_double(fx(1.25)));

    const double max_real = 2147483647.0 * lsb;
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_from_double(max_real, &v));
    TEST_ASSERT_EQUAL_INT32(2147483647, v);
    TEST_ASSERT_EQUAL_INT(KF_ERR_OVERFLOW, kf_fx_from_double(max_real + lsb, &v));
    TEST_ASSERT_EQUAL_INT(KF_ERR_OVERFLOW, kf_fx_from_double(-max_real - 2 * lsb, &v));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_from_double(NAN, &v));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_from_double(1.0, NULL));
}

/* ---- Closed forms (exact in the Q format) ---- */

void test_scalar_update_and_predict_match_closed_form(void) {
    kf_fx_state kf;
    const kf_fx H[1] = {KF_FX_ONE};
    const kf_fx F[1] = {2 * KF_FX_ONE};
    const kf_fx z[1] = {2 * KF_FX_ONE};
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_init(&kf, 1, 1));
    kf.P[0] = KF_FX_ONE;
    kf.R[0] = KF_FX_ONE;
    kf.Q[0] = KF_FX_ONE;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_update(&kf, z, H));
    TEST_ASSERT_EQUAL_INT32(KF_FX_ONE, kf.x[0]);     /* K = 0.5 */
    TEST_ASSERT_EQUAL_INT32(KF_FX_ONE / 2, kf.P[0]); /* P = 0.5 */
    /* NIS = 4 / 2 goes through sqrt(2), which the Q format can only round. */
    TEST_ASSERT_INT32_WITHIN(4, 2 * KF_FX_ONE, kf.nis);

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_predict(&kf, F));
    TEST_ASSERT_EQUAL_INT32(2 * KF_FX_ONE, kf.x[0]);
    TEST_ASSERT_EQUAL_INT32(3 * KF_FX_ONE, kf.P[0]); /* 4 * 0.5 + 1 */
}

/* ---- Against the floating-point filter ---- */

void test_fixed_point_filter_tracks_the_float_filter(void) {
    kf_state ref;
    kf_fx_state kf;
    kf_fx F[CV_N * CV_N], H[CV_M * CV_N], z[CV_M];
    double max_dx = 0, max_dp = 0;

    cv_scenario_init(&sc, 1000, 42);
    cv_scenario_setup_kf(&sc, &ref);
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_init(&kf, CV_N, CV_M));
    fx_matrix(kf.x, sc.x0_hat, CV_N);
    fx_matrix(kf.P, sc.P0, CV_N * CV_N);
    fx_matrix(kf.Q, sc.Q, CV_N * CV_N);
    fx_matrix(kf.R, sc.R, CV_M * CV_M);
    fx_matrix(F, sc.F, CV_N * CV_N);
    fx_matrix(H, sc.H, CV_M * CV_N);

    for (int k = 0; k < sc.steps; ++k) {
        fx_matrix(z, sc.z[k], CV_M);
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&ref, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&ref, sc.z[k], sc.H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_update(&kf, z, H));
        for (int i = 0; i < CV_N; ++i) {
            const double d = fabs(kf_fx_to_double(kf.x[i]) - (double)ref.x[i]);
            if (d > max_dx)
                max_dx = d;
        }
        for (int i = 0; i < CV_N * CV_N; ++i) {
            const double d = fabs(kf_fx_to_double(kf.P[i]) - (double)ref.P[i]);
            if (d > max_dp)
                max_dp = d;
        }
    }
    printf("[report] fixed-point q%d vs %s filter over %d CV steps: max |dx| %.3g, max |dP| %.3g\n",
           KF_FX_FRAC, KF_PRECISION_NAME, sc.steps, max_dx, max_dp);
    TEST_ASSERT_TRUE_MESSAGE(max_dx < FX_STATE_TOL,
                             "fixed-point state drifted from the float filter");
    TEST_ASSERT_TRUE_MESSAGE(max_dp < FX_COV_TOL,
                             "fixed-point covariance drifted from the float filter");
}

/* ---- Errors ---- */

void test_overflow_returns_error_and_state_unchanged(void) {
    kf_fx_state kf;
    const kf_fx F[1] = {4 * KF_FX_ONE};
    const kf_fx H[1] = {KF_FX_ONE};
    const kf_fx z[1] = {INT32_MIN};
    kf_fx_init(&kf, 1, 1);
    kf.x[0] = INT32_MAX / 2; /* F x leaves the range */
    kf.P[0] = KF_FX_ONE;
    kf.R[0] = KF_FX_ONE;
    kf_fx_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_OVERFLOW, kf_fx_predict(&kf, F));
    TEST_ASSERT_EQUAL_INT(KF_ERR_OVERFLOW, kf_fx_update(&kf, z, H)); /* z - H x leaves the range */
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_singular_innovation_and_bad_arguments_leave_state_unchanged(void) {
    kf_fx_state kf;
    const kf_fx H[2] = {KF_FX_ONE, 0};
    const kf_fx z[1] = {KF_FX_ONE};
    kf_fx_init(&kf, 2, 1); /* P = 0 and R = 0, so S = 0 */
    kf_fx_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_fx_update(&kf, z, H));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_update(&kf, NULL, H));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_predict(NULL, H));
    kf.n = KF_MAX_STATE + 1;
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_predict(&kf, H));
    kf.n = 2;
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_init(&kf, 0, 1));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_conversions_round_to_nearest_and_check_range);
    RUN_TEST(test_scalar_update_and_predict_match_closed_form);
    RUN_TEST(test_fixed_point_filter_tracks_the_float_filter);
    RUN_TEST(test_overflow_returns_error_and_state_unchanged);
    RUN_TEST(test_singular_innovation_and_bad_arguments_leave_state_unchanged);
    return UNITY_END();
}

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
/* Fixed-point EKF vs the float EKF on 300 range-bearing steps (positions of
 * 10-30 m): measured 4.9e-4 m against float64, about 4x headroom. */
#define FX_EKF_TOL 2e-3
#else
#define FX_STATE_TOL (2e-4 * (double)(1 << (20 - KF_FX_FRAC)))
#define FX_COV_TOL (2e-5 * (double)(1 << (20 - KF_FX_FRAC)))
#define FX_EKF_TOL (2e-3 * (double)(1 << (20 - KF_FX_FRAC)))
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

/* ---- Extended filter ---- */

/* F x exactly as the library computes it (wide_dot in kf_fixed.c): each product
 * divided by the guard scale (2^g >= KF_MAX_DIM, truncating), summed, scaled
 * back, then rounded to the Q format with ties away from zero. */
static kf_fx fx_dot(const kf_fx *a, const kf_fx *b, int len) {
    const int64_t guard = KF_MAX_DIM <= 4 ? 4 : KF_MAX_DIM <= 16 ? 16 : KF_MAX_DIM <= 64 ? 64 : 256;
    int64_t acc = 0;
    for (int k = 0; k < len; ++k)
        acc += ((int64_t)a[k] * (int64_t)b[k]) / guard;
    acc *= guard;
    const int64_t half = (int64_t)1 << (KF_FX_FRAC - 1);
    const int64_t r = acc >= 0 ? (acc + half) >> KF_FX_FRAC : -((-acc + half) >> KF_FX_FRAC);
    return (kf_fx)r;
}

typedef struct {
    const kf_fx *F;
    const kf_fx *H;
} fx_linear_ctx;

static int fx_linear_f(kf_fx *x_out, kf_fx *F_out, const kf_fx *x, int n, void *ctx) {
    const fx_linear_ctx *c = ctx;
    for (int i = 0; i < n; ++i)
        x_out[i] = fx_dot(&c->F[i * n], x, n);
    memcpy(F_out, c->F, (size_t)(n * n) * sizeof *F_out);
    return 0;
}

static int fx_linear_h(kf_fx *z_out, kf_fx *H_out, const kf_fx *x, int n, int m, void *ctx) {
    const fx_linear_ctx *c = ctx;
    for (int i = 0; i < m; ++i)
        z_out[i] = fx_dot(&c->H[i * n], x, n);
    memcpy(H_out, c->H, (size_t)(m * n) * sizeof *H_out);
    return 0;
}

void test_fixed_ekf_with_linear_model_matches_fixed_kf_exactly(void) {
    kf_fx_state kf, ekf;
    kf_fx F[CV_N * CV_N], H[CV_M * CV_N], z[CV_M];
    cv_scenario_init(&sc, 1000, 42);
    kf_fx_init(&kf, CV_N, CV_M);
    fx_matrix(kf.x, sc.x0_hat, CV_N);
    fx_matrix(kf.P, sc.P0, CV_N * CV_N);
    fx_matrix(kf.Q, sc.Q, CV_N * CV_N);
    fx_matrix(kf.R, sc.R, CV_M * CV_M);
    fx_matrix(F, sc.F, CV_N * CV_N);
    fx_matrix(H, sc.H, CV_M * CV_N);
    ekf = kf;
    fx_linear_ctx ctx = {F, H};

    for (int k = 0; k < sc.steps; ++k) {
        fx_matrix(z, sc.z[k], CV_M);
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_update(&kf, z, H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_ekf_predict(&ekf, fx_linear_f, &ctx));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_ekf_update(&ekf, z, fx_linear_h, &ctx));
    }
    /* One shared core, the same rounding in the callbacks: bit-identical. */
    TEST_ASSERT_EQUAL_MEMORY(&kf, &ekf, sizeof kf);
}

/* h(x) = |x| (range to the origin); computed in double, as a test convenience. */
static int fx_range_h(kf_fx *z_out, kf_fx *H_out, const kf_fx *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    const double px = kf_fx_to_double(x[0]), py = kf_fx_to_double(x[1]);
    const double r = sqrt(px * px + py * py);
    if (r == 0)
        return 1;
    return kf_fx_from_double(r, &z_out[0]) | kf_fx_from_double(px / r, &H_out[0]) |
           kf_fx_from_double(py / r, &H_out[1]);
}

void test_fixed_ekf_range_update_matches_hand_computation(void) {
    kf_fx_state kf;
    const kf_fx z[1] = {6 * KF_FX_ONE};
    kf_fx_init(&kf, 2, 1);
    kf.x[0] = 3 * KF_FX_ONE;
    kf.x[1] = 4 * KF_FX_ONE;
    kf.P[0] = kf.P[3] = KF_FX_ONE;
    kf.R[0] = KF_FX_ONE;

    TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_ekf_update(&kf, z, fx_range_h, NULL));
    /* h = 5, H = [0.6, 0.8], y = 1, S = 2, K = [0.3, 0.4]; within a few LSBs */
    TEST_ASSERT_INT32_WITHIN(8, fx(3.3), kf.x[0]);
    TEST_ASSERT_INT32_WITHIN(8, fx(4.4), kf.x[1]);
    TEST_ASSERT_INT32_WITHIN(8, fx(0.82), kf.P[0]);
    TEST_ASSERT_INT32_WITHIN(8, fx(-0.24), kf.P[1]);
    TEST_ASSERT_INT32_WITHIN(8, fx(0.68), kf.P[3]);
    TEST_ASSERT_INT32_WITHIN(8, fx(0.5), kf.nis);
}

/* Range-bearing from a sensor at (-10, -5), as in test_equivalence. */
static void range_bearing_d(double px, double py, double *z, double *H) {
    const double dx = px + 10, dy = py + 5, r2 = dx * dx + dy * dy, r = sqrt(r2);
    z[0] = r;
    z[1] = atan2(dy, dx);
    for (int i = 0; i < 8; ++i)
        H[i] = 0;
    H[0] = dx / r;
    H[1] = dy / r;
    H[4] = -dy / r2;
    H[5] = dx / r2;
}

static int fx_rb_h(kf_fx *z_out, kf_fx *H_out, const kf_fx *x, int n, int m, void *ctx) {
    double z[2], H[8];
    int rc = 0;
    (void)n;
    (void)m;
    (void)ctx;
    range_bearing_d(kf_fx_to_double(x[0]), kf_fx_to_double(x[1]), z, H);
    for (int i = 0; i < 2; ++i)
        rc |= kf_fx_from_double(z[i], &z_out[i]);
    for (int i = 0; i < 8; ++i)
        rc |= kf_fx_from_double(H[i], &H_out[i]);
    return rc;
}

static int fl_rb_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    double z[2], H[8];
    (void)n;
    (void)m;
    (void)ctx;
    range_bearing_d((double)x[0], (double)x[1], z, H);
    for (int i = 0; i < 2; ++i)
        z_out[i] = (kf_real)z[i];
    for (int i = 0; i < 8; ++i)
        H_out[i] = (kf_real)H[i];
    return 0;
}

void test_fixed_ekf_tracks_the_float_ekf_on_range_bearing(void) {
    kf_state ref;
    kf_fx_state kf;
    kf_fx F[CV_N * CV_N], zf[CV_M];
    double max_dx = 0;

    cv_scenario_init(&sc, 300, 42);
    cv_scenario_setup_kf(&sc, &ref);
    ref.R[0] = (kf_real)0.01;
    ref.R[3] = (kf_real)1e-4;
    kf_fx_init(&kf, CV_N, CV_M);
    fx_matrix(kf.x, ref.x, CV_N);
    fx_matrix(kf.P, ref.P, CV_N * CV_N);
    fx_matrix(kf.Q, ref.Q, CV_N * CV_N);
    fx_matrix(kf.R, ref.R, CV_M * CV_M);
    fx_matrix(F, sc.F, CV_N * CV_N);
    fx_linear_ctx ctx = {F, NULL};

    for (int k = 0; k < sc.steps; ++k) {
        double z[2], unused[8];
        range_bearing_d((double)sc.truth[k][0], (double)sc.truth[k][1], z, unused);
        const kf_real zr[2] = {(kf_real)z[0], (kf_real)z[1]};
        fx_matrix(zf, zr, CV_M);
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&ref, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&ref, zr, fl_rb_h, NULL));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_ekf_predict(&kf, fx_linear_f, &ctx));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_fx_ekf_update(&kf, zf, fx_rb_h, NULL));
        for (int i = 0; i < CV_N; ++i) {
            const double d = fabs(kf_fx_to_double(kf.x[i]) - (double)ref.x[i]);
            if (d > max_dx)
                max_dx = d;
        }
    }
    printf("[report] fixed-point q%d EKF vs %s EKF over %d range-bearing steps: max |dx| %.3g\n",
           KF_FX_FRAC, KF_PRECISION_NAME, sc.steps, max_dx);
    TEST_ASSERT_TRUE_MESSAGE(max_dx < FX_EKF_TOL, "fixed-point EKF drifted from the float EKF");
}

static int fx_failing_f(kf_fx *x_out, kf_fx *F_out, const kf_fx *x, int n, void *ctx) {
    (void)x_out;
    (void)F_out;
    (void)x;
    (void)n;
    (void)ctx;
    return 1;
}

static int fx_extreme_h(kf_fx *z_out, kf_fx *H_out, const kf_fx *x, int n, int m, void *ctx) {
    (void)x;
    (void)n;
    (void)m;
    (void)ctx;
    z_out[0] = INT32_MIN; /* z - h(x) with z = INT32_MAX overflows */
    H_out[0] = KF_FX_ONE;
    return 0;
}

void test_fixed_ekf_errors_leave_state_unchanged(void) {
    kf_fx_state kf;
    const kf_fx z[1] = {INT32_MAX};
    kf_fx_init(&kf, 1, 1);
    kf.P[0] = KF_FX_ONE;
    kf.R[0] = KF_FX_ONE;
    kf_fx_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_MODEL_FAILED, kf_fx_ekf_predict(&kf, fx_failing_f, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_OVERFLOW, kf_fx_ekf_update(&kf, z, fx_extreme_h, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_ekf_predict(&kf, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_fx_ekf_update(&kf, NULL, fx_extreme_h, NULL));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
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
    RUN_TEST(test_fixed_ekf_with_linear_model_matches_fixed_kf_exactly);
    RUN_TEST(test_fixed_ekf_range_update_matches_hand_computation);
    RUN_TEST(test_fixed_ekf_tracks_the_float_ekf_on_range_bearing);
    RUN_TEST(test_fixed_ekf_errors_leave_state_unchanged);
    RUN_TEST(test_overflow_returns_error_and_state_unchanged);
    RUN_TEST(test_singular_innovation_and_bad_arguments_leave_state_unchanged);
    return UNITY_END();
}

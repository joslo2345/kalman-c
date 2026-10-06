#include "unity.h"
#include "kalman/kalman.h"
#include "baselines/naive_kf.h"
#include "baselines/baseline_tinyekf.h"
#include "scenarios/constant_velocity.h"
#include "scenarios/rng.h"

#include <math.h>
#include <stdio.h>

/* On a well-conditioned problem every correct filter computes the same estimate,
 * so the goal is to match the baselines, not beat them. Tolerances are relative.
 *
 * The float tolerance is 5e-5 rather than the guide's 1e-5: over 1000 steps,
 * each float implementation accumulates its own rounding error. In the CV
 * scenario at step 792, the velocity estimate's relative error against a
 * float64 run was 3.6e-6 for kalman-c, 3.3e-7 for naive and 1.4e-5 for TinyEKF,
 * so kalman-c and TinyEKF differ by 1.7e-5 while all three agree to 9 digits
 * in double. */
#ifdef KF_USE_DOUBLE
#define TOL 1e-9
#else
#define TOL 5e-5
#endif

static cv_scenario sc; /* too large for the stack */

void setUp(void) {
    cv_scenario_init(&sc, 1000, 42);
}
void tearDown(void) {}

static void assert_states_match(const kf_real *expected, const kf_real *actual, int n, int step) {
    for (int i = 0; i < n; ++i) {
        const double e = (double)expected[i];
        char msg[64];
        snprintf(msg, sizeof msg, "step %d, state %d", step, i);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL * (1.0 + fabs(e)), e, (double)actual[i], msg);
    }
}

void test_linear_kf_matches_naive_on_well_conditioned_problem(void) {
    kf_state ours;
    naive_kf theirs;
    cv_scenario_setup_kf(&sc, &ours);
    cv_scenario_setup_naive(&sc, &theirs);

    for (int k = 0; k < sc.steps; ++k) {
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&ours, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&ours, sc.z[k], sc.H));
        TEST_ASSERT_EQUAL_INT(0, naive_kf_step(&theirs, sc.z[k]));
        assert_states_match(theirs.x, ours.x, ours.n, k);
    }
}

void test_linear_kf_matches_tinyekf_on_well_conditioned_problem(void) {
    kf_state ours;
    tinyekf_4x2 theirs;
    kf_real fx[CV_N], hx[CV_M];
    cv_scenario_setup_kf(&sc, &ours);
    cv_scenario_setup_tinyekf(&sc, &theirs);

    for (int k = 0; k < sc.steps; ++k) {
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&ours, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&ours, sc.z[k], sc.H));

        kf_mat_mul(fx, sc.F, theirs.x, CV_N, CV_N, 1);
        tinyekf_4x2_predict(&theirs, fx, sc.F, sc.Q);
        kf_mat_mul(hx, sc.H, theirs.x, CV_M, CV_N, 1);
        TEST_ASSERT_EQUAL_INT(0, tinyekf_4x2_update(&theirs, sc.z[k], hx, sc.H, sc.R));
        assert_states_match(theirs.x, ours.x, ours.n, k);
    }
}

/* ---- EKF vs TinyEKF on range-bearing measurements of the same trajectory ---- */

static const double sensor_offset[2] = {-10, -5}; /* sensor sits away from the track */

static void range_bearing(const kf_real *x, kf_real *z, kf_real *H) {
    const double px = (double)x[0] - sensor_offset[0], py = (double)x[1] - sensor_offset[1];
    const double r2 = px * px + py * py, r = sqrt(r2);
    z[0] = (kf_real)r;
    z[1] = (kf_real)atan2(py, px);
    for (int i = 0; i < CV_M * CV_N; ++i) H[i] = 0;
    H[0 * CV_N + 0] = (kf_real)(px / r);
    H[0 * CV_N + 1] = (kf_real)(py / r);
    H[1 * CV_N + 0] = (kf_real)(-py / r2);
    H[1 * CV_N + 1] = (kf_real)(px / r2);
}

static int ekf_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    range_bearing(x, z_out, H_out);
    return 0;
}

static int ekf_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)ctx;
    kf_mat_mul(x_out, sc.F, x, n, n, 1);
    for (int i = 0; i < n * n; ++i) F_out[i] = sc.F[i];
    return 0;
}

void test_ekf_matches_tinyekf_on_range_bearing(void) {
    kf_state ours;
    tinyekf_4x2 theirs;
    kf_real R[CV_M * CV_M] = {(kf_real)0.01, 0, 0, (kf_real)1e-4};
    kf_real fx[CV_N], hx[CV_M], H[CV_M * CV_N], z[CV_M], unused[CV_M * CV_N];
    rng g;
    rng_seed(&g, 7);

    cv_scenario_setup_kf(&sc, &ours);
    for (int i = 0; i < CV_M * CV_M; ++i) ours.R[i] = R[i];
    cv_scenario_setup_tinyekf(&sc, &theirs);

    for (int k = 0; k < 500; ++k) {
        range_bearing(sc.truth[k], z, unused);
        z[0] += (kf_real)(0.1 * rng_gauss(&g));
        z[1] += (kf_real)(0.01 * rng_gauss(&g));

        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_predict(&ours, ekf_f, NULL));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&ours, z, ekf_h, NULL));

        kf_mat_mul(fx, sc.F, theirs.x, CV_N, CV_N, 1);
        tinyekf_4x2_predict(&theirs, fx, sc.F, sc.Q);
        range_bearing(theirs.x, hx, H);
        TEST_ASSERT_EQUAL_INT(0, tinyekf_4x2_update(&theirs, z, hx, H, R));
        assert_states_match(theirs.x, ours.x, ours.n, k);
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_linear_kf_matches_naive_on_well_conditioned_problem);
    RUN_TEST(test_linear_kf_matches_tinyekf_on_well_conditioned_problem);
    RUN_TEST(test_ekf_matches_tinyekf_on_range_bearing);
    return UNITY_END();
}

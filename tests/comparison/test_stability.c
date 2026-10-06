#include "unity.h"
#include "kalman/kalman.h"
#include "scenarios/ill_conditioned.h"

#include <stdio.h>

/*
 * Long-run numerical stability on an ill-conditioned problem (tiny R, large Q).
 * The test only requires our covariance to stay symmetric and positive-definite.
 * Where the baselines fail is printed as [report] lines for the comparison
 * report, because that isn't guaranteed on every platform or precision.
 */

#define STEPS 1000000L

void setUp(void) {}
void tearDown(void) {}

static void report_baseline(const char *name, double r, double q, long failed_at) {
    if (failed_at < 0) {
        printf("[report] stability %s r=%g q=%g: %s stayed positive-definite for %ld steps\n",
               KF_PRECISION_NAME, r, q, name, STEPS);
    } else {
        printf("[report] stability %s r=%g q=%g: %s lost positive-definiteness at step %ld\n",
               KF_PRECISION_NAME, r, q, name, failed_at);
    }
}

static void run(double r) {
    const double q = 100, dt = 0.1;
    ic_scenario sc;
    kf_state ours;
    naive_kf naive;
    tinyekf_4x2 tiny;
    kf_real z[CV_M], fx[CV_N], hx[CV_M];
    long naive_failed_at = -1, tiny_failed_at = -1;

    ic_scenario_init(&sc, r, q, dt, 7);
    ic_scenario_setup_kf(&sc, &ours);
    ic_scenario_setup_naive(&sc, &naive);
    ic_scenario_setup_tinyekf(&sc, &tiny);

    for (long k = 0; k < STEPS; ++k) {
        ic_scenario_measure(&sc, z);

        TEST_ASSERT_EQUAL_INT_MESSAGE(KF_OK, kf_predict(&ours, sc.F), "our predict failed");
        TEST_ASSERT_EQUAL_INT_MESSAGE(KF_OK, kf_update(&ours, z, sc.H), "our update failed");
        TEST_ASSERT_TRUE_MESSAGE(kf_is_symmetric(ours.P, CV_N, (kf_real)1e-4),
                                 "our covariance lost symmetry");
        TEST_ASSERT_TRUE_MESSAGE(kf_cholesky_ok(ours.P, CV_N),
                                 "our covariance lost positive-definiteness");

        if (naive_failed_at < 0) {
            if (naive_kf_step(&naive, z) != 0 || !kf_cholesky_ok(naive.P, CV_N)) {
                naive_failed_at = k;
            }
        }
        if (tiny_failed_at < 0) {
            kf_mat_mul(fx, sc.F, tiny.x, CV_N, CV_N, 1);
            tinyekf_4x2_predict(&tiny, fx, sc.F, sc.Q);
            kf_mat_mul(hx, sc.H, tiny.x, CV_M, CV_N, 1);
            if (tinyekf_4x2_update(&tiny, z, hx, sc.H, sc.R) != 0 ||
                !kf_cholesky_ok(tiny.P, CV_N)) {
                tiny_failed_at = k;
            }
        }
    }
    printf("[report] stability %s r=%g q=%g: kalman-c stayed positive-definite for %ld steps\n",
           KF_PRECISION_NAME, r, q, STEPS);
    report_baseline("naive", r, q, naive_failed_at);
    report_baseline("tinyekf", r, q, tiny_failed_at);
}

void test_covariance_stays_positive_definite_with_tiny_r(void) {
    run(1e-8);
}

void test_covariance_stays_positive_definite_with_tinier_r(void) {
    run(1e-9);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_covariance_stays_positive_definite_with_tiny_r);
    RUN_TEST(test_covariance_stays_positive_definite_with_tinier_r);
    return UNITY_END();
}

#include "kalman/kalman.h"
#include "scenarios/constant_velocity.h"
#include "unity.h"

#include <math.h>
#include <stdio.h>

/*
 * Monte Carlo consistency: when the filter's model matches the truth, the
 * average NIS and NEES must fall inside their 95% chi-squared bounds. A filter
 * that passes is neither overconfident nor underconfident.
 *
 * - NIS: innovations of a consistent filter are white, so the sum over all
 *   trials and steps is chi-squared with TRIALS * STEPS * m degrees of freedom.
 * - NEES: estimation errors are correlated over time, so it is checked at the
 *   final step across independent trials, with TRIALS * n degrees of freedom.
 */

#define TRIALS 500
#define STEPS 100

static cv_scenario sc;

void setUp(void) {}
void tearDown(void) {}

/* Wilson-Hilferty approximation to the chi-squared quantile, divided by dof:
 * the bounds for an average of chi-squared variables. */
static double chi2_mean_bound(double dof, double z) {
    const double a = 2.0 / (9.0 * dof);
    const double t = 1.0 - a + z * sqrt(a);
    return t * t * t;
}

/* e^T P^-1 e via Cholesky */
static double nees(const kf_state *kf, const kf_real *truth) {
    kf_real L[CV_N * CV_N], e[CV_N], w[CV_N];
    double s = 0;
    for (int i = 0; i < CV_N; ++i)
        e[i] = kf->x[i] - truth[i];
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_cholesky(L, kf->P, CV_N));
    kf_solve_lower(w, L, e, CV_N, 1);
    for (int i = 0; i < CV_N; ++i)
        s += (double)w[i] * (double)w[i];
    return s;
}

void test_average_nis_and_nees_are_inside_95_percent_bounds(void) {
    const double z95 = 1.959963984540054;
    double nis_sum = 0, nees_sum = 0;

    for (int trial = 0; trial < TRIALS; ++trial) {
        kf_state kf;
        cv_scenario_init(&sc, STEPS, 1000u + (unsigned)trial);
        cv_scenario_setup_kf(&sc, &kf);
        for (int k = 0; k < STEPS; ++k) {
            TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, sc.F));
            TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, sc.z[k], sc.H));
            nis_sum += (double)kf.nis;
        }
        nees_sum += nees(&kf, sc.truth[STEPS - 1]);
    }

    const double nis_dof = (double)TRIALS * STEPS * CV_M;
    const double nis_avg = nis_sum / ((double)TRIALS * STEPS); /* expected: m */
    const double nis_lo = CV_M * chi2_mean_bound(nis_dof, -z95);
    const double nis_hi = CV_M * chi2_mean_bound(nis_dof, z95);

    const double nees_dof = (double)TRIALS * CV_N;
    const double nees_avg = nees_sum / TRIALS; /* expected: n */
    const double nees_lo = CV_N * chi2_mean_bound(nees_dof, -z95);
    const double nees_hi = CV_N * chi2_mean_bound(nees_dof, z95);

    printf("[report] consistency %s KF: average NIS %.4f, 95%% bounds [%.4f, %.4f]\n",
           KF_PRECISION_NAME, nis_avg, nis_lo, nis_hi);
    printf("[report] consistency %s KF: average NEES %.4f, 95%% bounds [%.4f, %.4f]\n",
           KF_PRECISION_NAME, nees_avg, nees_lo, nees_hi);

    TEST_ASSERT_TRUE_MESSAGE(nis_avg >= nis_lo && nis_avg <= nis_hi, "NIS outside 95% bounds");
    TEST_ASSERT_TRUE_MESSAGE(nees_avg >= nees_lo && nees_avg <= nees_hi, "NEES outside 95% bounds");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_average_nis_and_nees_are_inside_95_percent_bounds);
    return UNITY_END();
}

#include "kalman/kalman.h"
#include "scenarios/ill_conditioned.h"
#include "unity.h"

#include <math.h>
#include <stdio.h>

/*
 * Covariance accuracy on the ill-conditioned model (tiny R, large Q), against
 * a float64 reference computed here, independent of kf_real. A linear filter's
 * covariance does not depend on the measurements, so only P is compared.
 *
 * The UD square-root filter must keep the small position variance accurate.
 * The Joseph-form filter's error is reported but not asserted: in float32 it
 * loses about 3 digits at R = 1e-10 (see kf_sqrt.h).
 */

#define STEPS 100000L

void setUp(void) {}
void tearDown(void) {}

/* float64 Joseph recursion for the 4-state / 2-measurement model with H = [I 0]. */
static void reference_step(double P[16], const ic_scenario *sc) {
    double FP[16], Pp[16], S[4], Si[4], K[8], A[16], AP[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += (double)sc->F[i * 4 + k] * P[k * 4 + j];
            FP[i * 4 + j] = s;
        }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += FP[i * 4 + k] * (double)sc->F[j * 4 + k];
            Pp[i * 4 + j] = s + (double)sc->Q[i * 4 + j];
        }
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            S[i * 2 + j] = Pp[i * 4 + j] + (double)sc->R[i * 2 + j];
    const double det = S[0] * S[3] - S[1] * S[2];
    Si[0] = S[3] / det;
    Si[1] = -S[1] / det;
    Si[2] = -S[2] / det;
    Si[3] = S[0] / det;
    for (int i = 0; i < 4; ++i) /* K = P H^T S^-1, P H^T = first two columns of P */
        for (int j = 0; j < 2; ++j)
            K[i * 2 + j] = Pp[i * 4 + 0] * Si[0 * 2 + j] + Pp[i * 4 + 1] * Si[1 * 2 + j];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            A[i * 4 + j] = (i == j) - (j < 2 ? K[i * 2 + j] : 0);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += A[i * 4 + k] * Pp[k * 4 + j];
            AP[i * 4 + j] = s;
        }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += AP[i * 4 + k] * A[j * 4 + k];
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b)
                    s += K[i * 2 + a] * (double)sc->R[a * 2 + b] * K[j * 2 + b];
            P[i * 4 + j] = s;
        }
}

void test_ud_filter_keeps_small_variances_accurate(void) {
    const double r = 1e-10;
    ic_scenario sc;
    kf_state kf;
    kf_sr_state sr;
    kf_real z[CV_M] = {0, 0}, P_ud[16];
    double P_ref[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    double err_ud = 0, err_joseph = 0;

    ic_scenario_init(&sc, r, 100, 0.1, 7);
    ic_scenario_setup_kf(&sc, &kf);
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_init(&sr, CV_N, CV_M));
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_P(&sr, kf.P));
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_Q(&sr, sc.Q));
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_set_R(&sr, sc.R));

    for (long k = 0; k < STEPS; ++k) {
        reference_step(P_ref, &sc);
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, z, sc.H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_predict(&sr, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_sr_update(&sr, z, sc.H));
        kf_sr_get_P(&sr, P_ud);
        const double e_ud = fabs((double)P_ud[0] - P_ref[0]) / P_ref[0];
        const double e_j = fabs((double)kf.P[0] - P_ref[0]) / P_ref[0];
        if (e_ud > err_ud)
            err_ud = e_ud;
        if (e_j > err_joseph)
            err_joseph = e_j;
    }
    printf("[report] sqrt accuracy %s r=%g: worst relative error of the position variance over "
           "%ld steps: UD %.2e, Joseph %.2e\n",
           KF_PRECISION_NAME, r, STEPS, err_ud, err_joseph);
    TEST_ASSERT_TRUE_MESSAGE(err_ud < 1e-5, "UD position variance lost accuracy");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ud_filter_keeps_small_variances_accurate);
    return UNITY_END();
}

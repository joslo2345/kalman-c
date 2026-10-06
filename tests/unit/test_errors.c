#include "kalman/kalman.h"
#include "unity.h"

#include <math.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

void test_init_accepts_valid_dimensions(void) {
    kf_state kf;
    TEST_ASSERT_EQUAL_INT(KF_OK, kf_init(&kf, 4, 2));
    TEST_ASSERT_EQUAL_INT(4, kf.n);
    TEST_ASSERT_EQUAL_INT(2, kf.m);
}

void test_init_rejects_null(void) {
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_init(NULL, 4, 2));
}

void test_init_rejects_out_of_range_dimensions_and_leaves_state_unchanged(void) {
    kf_state kf;
    memset(&kf, 0xA5, sizeof kf);
    kf_state before = kf;

    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_init(&kf, 0, 1));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_init(&kf, KF_MAX_STATE + 1, 1));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_init(&kf, 2, 0));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_init(&kf, 2, KF_MAX_MEAS + 1));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

static void cv_setup_default(kf_state *kf) {
    kf_init(kf, 2, 2);
    kf_mat_identity(kf->P, 2);
    kf_mat_identity(kf->R, 2);
}

static const kf_real cv_H[4] = {1, 0, 0, 1};

void test_nan_measurement_is_rejected_and_state_unchanged(void) {
    kf_state kf;
    cv_setup_default(&kf);
    kf_state before = kf;

    kf_real z[2] = {NAN, 1.0f};
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_update(&kf, z, cv_H));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_inf_in_transition_matrix_is_rejected_and_state_unchanged(void) {
    kf_state kf;
    cv_setup_default(&kf);
    kf_state before = kf;

    const kf_real F[4] = {1, INFINITY, 0, 1};
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_predict(&kf, F));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_singular_innovation_covariance_returns_error_and_state_unchanged(void) {
    kf_state kf;
    kf_init(&kf, 2, 2); /* P = 0 and R = 0, so S = 0 */
    kf_state before = kf;

    kf_real z[2] = {1.0f, 1.0f};
    TEST_ASSERT_EQUAL_INT(KF_ERR_NOT_POSITIVE_DEFINITE, kf_update(&kf, z, cv_H));
    TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
}

void test_null_arguments_are_rejected(void) {
    kf_state kf;
    cv_setup_default(&kf);
    kf_real z[2] = {0, 0};
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_predict(NULL, cv_H));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_predict(&kf, NULL));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_update(&kf, NULL, cv_H));
    TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_update(&kf, z, NULL));
}

static int identity_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)ctx;
    for (int i = 0; i < n; ++i)
        x_out[i] = x[i];
    kf_mat_identity(F_out, n);
    return 0;
}

static int identity_ukf_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    (void)ctx;
    for (int i = 0; i < n; ++i)
        x_out[i] = x[i];
    return 0;
}

void test_corrupted_dimensions_are_rejected_and_state_unchanged(void) {
    const int bad_dims[4][2] = {{0, 1}, {KF_MAX_STATE + 1, 1}, {2, 0}, {2, KF_MAX_MEAS + 1}};
    kf_real F[KF_MAX_STATE * KF_MAX_STATE] = {0};
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE] = {0};
    kf_real z[KF_MAX_MEAS] = {0};

    for (int i = 0; i < 4; ++i) {
        kf_state kf;
        cv_setup_default(&kf);
        kf.n = bad_dims[i][0];
        kf.m = bad_dims[i][1];
        kf_state before = kf;
        TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_predict(&kf, F));
        TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_update(&kf, z, H));
        TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT, kf_ekf_predict(&kf, identity_f, NULL));
        TEST_ASSERT_EQUAL_INT(KF_ERR_INVALID_INPUT,
                              kf_ukf_predict(&kf, NULL, identity_ukf_f, NULL));
        TEST_ASSERT_EQUAL_MEMORY(&before, &kf, sizeof kf);
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_accepts_valid_dimensions);
    RUN_TEST(test_init_rejects_null);
    RUN_TEST(test_init_rejects_out_of_range_dimensions_and_leaves_state_unchanged);
    RUN_TEST(test_nan_measurement_is_rejected_and_state_unchanged);
    RUN_TEST(test_inf_in_transition_matrix_is_rejected_and_state_unchanged);
    RUN_TEST(test_singular_innovation_covariance_returns_error_and_state_unchanged);
    RUN_TEST(test_null_arguments_are_rejected);
    RUN_TEST(test_corrupted_dimensions_are_rejected_and_state_unchanged);
    return UNITY_END();
}

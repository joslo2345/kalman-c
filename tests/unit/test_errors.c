#include "unity.h"
#include "kalman/kalman.h"

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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_accepts_valid_dimensions);
    RUN_TEST(test_init_rejects_null);
    RUN_TEST(test_init_rejects_out_of_range_dimensions_and_leaves_state_unchanged);
    return UNITY_END();
}

#include "unity.h"
#include "kalman/kalman.h"

#include <stddef.h>

static int alloc_count = 0;

void *__wrap_malloc(size_t n)           { (void)n; alloc_count++; return NULL; }
void *__wrap_calloc(size_t a, size_t b) { (void)a; (void)b; alloc_count++; return NULL; }
void *__wrap_realloc(void *p, size_t n) { (void)p; (void)n; alloc_count++; return NULL; }

void setUp(void) {}
void tearDown(void) {}

/* TODO(Step 7): run a full EKF and UKF scenario once those filters exist. */
void test_full_filter_run_never_allocates(void) {
    kf_state kf;
    kf_real F[KF_MAX_STATE * KF_MAX_STATE] = {0};
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE] = {0};
    kf_real z[KF_MAX_MEAS] = {0};

    alloc_count = 0;
    kf_init(&kf, 4, 2);
    kf_predict(&kf, F);
    kf_update(&kf, z, H);
    TEST_ASSERT_EQUAL_INT(0, alloc_count);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_full_filter_run_never_allocates);
    return UNITY_END();
}

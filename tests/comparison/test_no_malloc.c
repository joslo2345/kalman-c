#include "kalman/kalman.h"
#include "scenarios/constant_velocity.h"
#include "unity.h"

#include <stddef.h>
#include <string.h>

/* Linked with -Wl,--wrap=malloc,... so every allocation in this binary is counted. */
static int alloc_count = 0;

void *__wrap_malloc(size_t n) {
    (void)n;
    alloc_count++;
    return NULL;
}
void *__wrap_calloc(size_t a, size_t b) {
    (void)a;
    (void)b;
    alloc_count++;
    return NULL;
}
void *__wrap_realloc(void *p, size_t n) {
    (void)p;
    (void)n;
    alloc_count++;
    return NULL;
}

static cv_scenario sc;

void setUp(void) {}
void tearDown(void) {}

static int ekf_f(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    (void)ctx;
    kf_mat_mul(x_out, sc.F, x, n, n, 1);
    memcpy(F_out, sc.F, sizeof sc.F);
    return 0;
}

static int ekf_h(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    (void)ctx;
    kf_mat_mul(z_out, sc.H, x, m, n, 1);
    memcpy(H_out, sc.H, sizeof sc.H);
    return 0;
}

static int ukf_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    (void)ctx;
    kf_mat_mul(x_out, sc.F, x, n, n, 1);
    return 0;
}

static int ukf_h(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    (void)ctx;
    kf_mat_mul(z_out, sc.H, x, m, n, 1);
    return 0;
}

void test_full_kf_ekf_and_ukf_runs_never_allocate(void) {
    kf_state kf, ekf, ukf;
    cv_scenario_init(&sc, 200, 3);
    cv_scenario_setup_kf(&sc, &kf);
    ekf = kf;
    ukf = kf;

    alloc_count = 0;
    for (int k = 0; k < sc.steps; ++k) {
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_predict(&kf, sc.F));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_update(&kf, sc.z[k], sc.H));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_predict(&ekf, ekf_f, NULL));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ekf_update(&ekf, sc.z[k], ekf_h, NULL));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_predict(&ukf, NULL, ukf_f, NULL));
        TEST_ASSERT_EQUAL_INT(KF_OK, kf_ukf_update(&ukf, sc.z[k], NULL, ukf_h, NULL));
    }
    TEST_ASSERT_EQUAL_INT(0, alloc_count);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_full_kf_ekf_and_ukf_runs_never_allocate);
    return UNITY_END();
}

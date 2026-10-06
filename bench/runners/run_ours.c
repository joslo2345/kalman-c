#include "runners.h"

#include <string.h>

static void setup(kf_state *kf, const scenario *sc) {
    kf_init(kf, sc->n, sc->m);
    memcpy(kf->x, sc->x0, (size_t)sc->n * sizeof *kf->x);
    memcpy(kf->P, sc->P0, (size_t)(sc->n * sc->n) * sizeof *kf->P);
    memcpy(kf->Q, sc->Q, (size_t)(sc->n * sc->n) * sizeof *kf->Q);
    memcpy(kf->R, sc->R, (size_t)(sc->m * sc->m) * sizeof *kf->R);
}

/* Runs fn(kf, sc, trial, k) for every step, with metric collection when asked. */
#define RUN_LOOP(step_expr)                                                                        \
    do {                                                                                           \
        metrics mt;                                                                                \
        if (out->measure)                                                                          \
            metrics_begin(&mt, out);                                                               \
        for (int trial = 0; trial < sc->trials; ++trial) {                                         \
            kf_state kf;                                                                           \
            setup(&kf, sc);                                                                        \
            for (int k = 0; k < sc->steps; ++k) {                                                  \
                const kf_real *z = scenario_z(sc, trial, k);                                       \
                const int ok = (step_expr);                                                        \
                if (out->measure) {                                                                \
                    if (metrics_step(&mt, sc, out, trial, k, kf.x, kf.P, ok))                      \
                        goto done;                                                                 \
                } else if (!ok) {                                                                  \
                    return RUN_FAILED;                                                             \
                }                                                                                  \
            }                                                                                      \
        }                                                                                          \
    done:                                                                                          \
        if (out->measure)                                                                          \
            metrics_end(&mt, sc, out);                                                             \
        return RUN_OK;                                                                             \
    } while (0)

int run_ours_kf(const scenario *sc, run_result *out) {
    if (sc->meas != MEAS_LINEAR)
        return RUN_UNSUPPORTED;
    RUN_LOOP(kf_predict(&kf, sc->F) == KF_OK && kf_update(&kf, z, sc->H) == KF_OK);
}

/* ---- Model callbacks shared by the EKF and UKF; ctx is the scenario ---- */

static int f_ekf(kf_real *x_out, kf_real *F_out, const kf_real *x, int n, void *ctx) {
    const scenario *sc = ctx;
    kf_mat_mul(x_out, sc->F, x, n, n, 1);
    memcpy(F_out, sc->F, (size_t)(n * n) * sizeof *F_out);
    return 0;
}

static int h_ekf(kf_real *z_out, kf_real *H_out, const kf_real *x, int n, int m, void *ctx) {
    const scenario *sc = ctx;
    if (sc->meas == MEAS_RANGE_BEARING) {
        scenario_range_bearing(sc, x, z_out, H_out);
    } else {
        kf_mat_mul(z_out, sc->H, x, m, n, 1);
        memcpy(H_out, sc->H, (size_t)(m * n) * sizeof *H_out);
    }
    return 0;
}

static int f_ukf(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    const scenario *sc = ctx;
    kf_mat_mul(x_out, sc->F, x, n, n, 1);
    return 0;
}

static int h_ukf(kf_real *z_out, const kf_real *x, int n, int m, void *ctx) {
    const scenario *sc = ctx;
    if (sc->meas == MEAS_RANGE_BEARING) {
        scenario_range_bearing(sc, x, z_out, NULL);
    } else {
        kf_mat_mul(z_out, sc->H, x, m, n, 1);
    }
    return 0;
}

int run_ours_ekf(const scenario *sc, run_result *out) {
    void *ctx = (void *)sc;
    RUN_LOOP(kf_ekf_predict(&kf, f_ekf, ctx) == KF_OK &&
             kf_ekf_update(&kf, z, h_ekf, ctx) == KF_OK);
}

int run_ours_ukf(const scenario *sc, run_result *out) {
    void *ctx = (void *)sc;
    RUN_LOOP(kf_ukf_predict(&kf, NULL, f_ukf, ctx) == KF_OK &&
             kf_ukf_update(&kf, z, NULL, h_ukf, ctx) == KF_OK);
}

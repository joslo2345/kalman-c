#include "scenarios/constant_velocity.h"
#include "scenarios/rng.h"

#include <string.h>

static void cv_q_double(double *Q, double dt, double q) {
    memset(Q, 0, CV_N * CV_N * sizeof *Q);
    /* Per axis: q * [[dt^3/3, dt^2/2], [dt^2/2, dt]] */
    for (int a = 0; a < 2; ++a) {
        const int p = a, vel = a + 2;
        Q[p * CV_N + p] = q * dt * dt * dt / 3;
        Q[p * CV_N + vel] = Q[vel * CV_N + p] = q * dt * dt / 2;
        Q[vel * CV_N + vel] = q * dt;
    }
}

void cv_build_model(double dt, double q, double r, kf_real *F, kf_real *H, kf_real *Q, kf_real *R) {
    double Qd[CV_N * CV_N];
    cv_q_double(Qd, dt, q);
    kf_mat_identity(F, CV_N);
    F[0 * CV_N + 2] = (kf_real)dt;
    F[1 * CV_N + 3] = (kf_real)dt;
    memset(H, 0, CV_M * CV_N * sizeof *H);
    H[0 * CV_N + 0] = 1;
    H[1 * CV_N + 1] = 1;
    for (int i = 0; i < CV_N * CV_N; ++i)
        Q[i] = (kf_real)Qd[i];
    R[0] = R[3] = (kf_real)r;
    R[1] = R[2] = 0;
}

void cv_scenario_init(cv_scenario *sc, int steps, unsigned long long seed) {
    const double dt = 0.1, q = 0.1, sigma = 0.5;
    const double x0_hat[CV_N] = {0, 0, 1, 0.5};
    const double P0_diag[CV_N] = {1, 1, 0.25, 0.25};
    double Q[CV_N * CV_N], P0[CV_N * CV_N] = {0};
    double LQ[CV_N * CV_N], LP0[CV_N * CV_N], LR[CV_M * CV_M] = {sigma, 0, 0, sigma};
    double x[CV_N], w[CV_N], v[CV_M];
    rng r;

    if (steps > CV_MAX_STEPS)
        steps = CV_MAX_STEPS;
    memset(sc, 0, sizeof *sc);
    sc->steps = steps;

    cv_q_double(Q, dt, q);
    for (int i = 0; i < CV_N; ++i)
        P0[i * CV_N + i] = P0_diag[i];
    rng_cholesky(LQ, Q, CV_N);
    rng_cholesky(LP0, P0, CV_N);

    cv_build_model(dt, q, sigma * sigma, sc->F, sc->H, sc->Q, sc->R);
    for (int i = 0; i < CV_N * CV_N; ++i)
        sc->P0[i] = (kf_real)P0[i];
    for (int i = 0; i < CV_N; ++i)
        sc->x0_hat[i] = (kf_real)x0_hat[i];

    rng_seed(&r, seed);
    rng_gauss_vec(&r, x, LP0, CV_N);
    for (int i = 0; i < CV_N; ++i)
        x[i] += x0_hat[i];

    for (int k = 0; k < steps; ++k) {
        rng_gauss_vec(&r, w, LQ, CV_N);
        x[0] += dt * x[2];
        x[1] += dt * x[3];
        for (int i = 0; i < CV_N; ++i)
            x[i] += w[i];
        rng_gauss_vec(&r, v, LR, CV_M);
        for (int i = 0; i < CV_N; ++i)
            sc->truth[k][i] = (kf_real)x[i];
        sc->z[k][0] = (kf_real)(x[0] + v[0]);
        sc->z[k][1] = (kf_real)(x[1] + v[1]);
    }
}

void cv_scenario_setup_kf(const cv_scenario *sc, kf_state *kf) {
    kf_init(kf, CV_N, CV_M);
    memcpy(kf->x, sc->x0_hat, sizeof sc->x0_hat);
    memcpy(kf->P, sc->P0, sizeof sc->P0);
    memcpy(kf->Q, sc->Q, sizeof sc->Q);
    memcpy(kf->R, sc->R, sizeof sc->R);
}

void cv_scenario_setup_naive(const cv_scenario *sc, naive_kf *kf) {
    memset(kf, 0, sizeof *kf);
    kf->n = CV_N;
    kf->m = CV_M;
    memcpy(kf->x, sc->x0_hat, sizeof sc->x0_hat);
    memcpy(kf->P, sc->P0, sizeof sc->P0);
    memcpy(kf->F, sc->F, sizeof sc->F);
    memcpy(kf->H, sc->H, sizeof sc->H);
    memcpy(kf->Q, sc->Q, sizeof sc->Q);
    memcpy(kf->R, sc->R, sizeof sc->R);
}

void cv_scenario_setup_tinyekf(const cv_scenario *sc, tinyekf_4x2 *t) {
    memcpy(t->x, sc->x0_hat, sizeof sc->x0_hat);
    memcpy(t->P, sc->P0, sizeof sc->P0);
}

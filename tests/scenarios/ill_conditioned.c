#include "scenarios/ill_conditioned.h"

#include <math.h>
#include <string.h>

void ic_scenario_init(ic_scenario *sc, double r, double q, double dt, unsigned long long seed) {
    memset(sc, 0, sizeof *sc);
    sc->sigma = sqrt(r);
    cv_build_model(dt, q, r, sc->F, sc->H, sc->Q, sc->R);
    rng_seed(&sc->g, seed);
}

void ic_scenario_measure(ic_scenario *sc, kf_real *z) {
    z[0] = (kf_real)(sc->sigma * rng_gauss(&sc->g));
    z[1] = (kf_real)(sc->sigma * rng_gauss(&sc->g));
}

void ic_scenario_setup_kf(const ic_scenario *sc, kf_state *kf) {
    kf_init(kf, CV_N, CV_M);
    kf_mat_identity(kf->P, CV_N);
    memcpy(kf->Q, sc->Q, sizeof sc->Q);
    memcpy(kf->R, sc->R, sizeof sc->R);
}

void ic_scenario_setup_naive(const ic_scenario *sc, naive_kf *kf) {
    memset(kf, 0, sizeof *kf);
    kf->n = CV_N;
    kf->m = CV_M;
    kf_mat_identity(kf->P, CV_N);
    memcpy(kf->F, sc->F, sizeof sc->F);
    memcpy(kf->H, sc->H, sizeof sc->H);
    memcpy(kf->Q, sc->Q, sizeof sc->Q);
    memcpy(kf->R, sc->R, sizeof sc->R);
}

void ic_scenario_setup_tinyekf(const ic_scenario *sc, tinyekf_4x2 *t) {
    (void)sc;
    memset(t, 0, sizeof *t);
    kf_mat_identity(t->P, CV_N);
}

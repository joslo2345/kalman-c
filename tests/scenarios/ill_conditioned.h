#ifndef SCENARIO_ILL_CONDITIONED_H
#define SCENARIO_ILL_CONDITIONED_H

/*
 * Ill-conditioned tracking: the 2D constant-velocity model with a tiny
 * measurement variance r and a large acceleration noise q, observing a
 * stationary target at the origin. Measurements are generated on the fly, so
 * the scenario can run for millions of steps. P0 = I, x0 = 0.
 */

#include "scenarios/constant_velocity.h"
#include "scenarios/rng.h"

typedef struct {
    double sigma;
    kf_real F[CV_N * CV_N];
    kf_real H[CV_M * CV_N];
    kf_real Q[CV_N * CV_N];
    kf_real R[CV_M * CV_M];
    rng g;
} ic_scenario;

void ic_scenario_init(ic_scenario *sc, double r, double q, double dt, unsigned long long seed);
void ic_scenario_measure(ic_scenario *sc, kf_real *z);

void ic_scenario_setup_kf(const ic_scenario *sc, kf_state *kf);
void ic_scenario_setup_naive(const ic_scenario *sc, naive_kf *kf);
void ic_scenario_setup_tinyekf(const ic_scenario *sc, tinyekf_4x2 *t);

#endif

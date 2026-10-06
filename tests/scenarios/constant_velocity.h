#ifndef SCENARIO_CONSTANT_VELOCITY_H
#define SCENARIO_CONSTANT_VELOCITY_H

/*
 * 2D constant-velocity tracking (state [px, py, vx, vy], measurement [px, py])
 * with continuous white-noise acceleration. The truth, including x0 drawn
 * from N(x0_hat, P0), is simulated in double from a fixed seed, so every filter
 * and every precision sees the same data.
 */

#include "baselines/baseline_tinyekf.h"
#include "baselines/naive_kf.h"
#include "kalman/kalman.h"

#define CV_N 4
#define CV_M 2
#define CV_MAX_STEPS 1000

typedef struct {
    int steps;
    kf_real F[CV_N * CV_N];
    kf_real H[CV_M * CV_N];
    kf_real Q[CV_N * CV_N];
    kf_real R[CV_M * CV_M];
    kf_real x0_hat[CV_N];
    kf_real P0[CV_N * CV_N];
    kf_real truth[CV_MAX_STEPS][CV_N]; /* true state after step k's prediction */
    kf_real z[CV_MAX_STEPS][CV_M];     /* measurement of truth[k] */
} cv_scenario;

/* F, H, Q (continuous white-noise acceleration with intensity q) and R = r I
 * for the 4-state / 2-measurement model, all row-major. */
void cv_build_model(double dt, double q, double r, kf_real *F, kf_real *H, kf_real *Q, kf_real *R);

/* dt = 0.1, accel noise q = 0.1, measurement sigma = 0.5 */
void cv_scenario_init(cv_scenario *sc, int steps, unsigned long long seed);

void cv_scenario_setup_kf(const cv_scenario *sc, kf_state *kf);
void cv_scenario_setup_naive(const cv_scenario *sc, naive_kf *kf);
void cv_scenario_setup_tinyekf(const cv_scenario *sc, tinyekf_4x2 *t);

#endif

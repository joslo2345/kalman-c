#ifndef BASELINE_TINYEKF_H
#define BASELINE_TINYEKF_H

/*
 * Thin adapter over TinyEKF (tests/baselines/tinyekf, pinned submodule).
 * TinyEKF fixes its dimensions at compile time, so this adapter is built for
 * the 4-state / 2-measurement scenarios. The calls mirror TinyEKF's own
 * ekf_predict / ekf_update, which take precomputed f(x), h(x) and Jacobians.
 */

#include "kalman/kf_config.h"

#define TINYEKF_N 4
#define TINYEKF_M 2

typedef struct {
    kf_real x[TINYEKF_N];
    kf_real P[TINYEKF_N * TINYEKF_N];
} tinyekf_4x2;

void tinyekf_4x2_predict(tinyekf_4x2 *t, const kf_real *fx, const kf_real *F, const kf_real *Q);

/* Returns 0, or -1 if TinyEKF could not invert S. */
int tinyekf_4x2_update(tinyekf_4x2 *t, const kf_real *z, const kf_real *hx, const kf_real *H,
                       const kf_real *R);

#endif

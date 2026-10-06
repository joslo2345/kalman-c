#ifndef NAIVE_KF_H
#define NAIVE_KF_H

/*
 * Textbook linear Kalman filter, representing the typical hand-written filter:
 * explicit inverse of S and the P = (I - K H) P update, with no symmetrization
 * or other stabilization. It's a comparison baseline only; do not "fix" it.
 */

#include "kalman/kf_config.h"

typedef struct {
    int n, m;
    kf_real x[KF_MAX_STATE];
    kf_real P[KF_MAX_STATE * KF_MAX_STATE];
    kf_real F[KF_MAX_STATE * KF_MAX_STATE];
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real Q[KF_MAX_STATE * KF_MAX_STATE];
    kf_real R[KF_MAX_MEAS * KF_MAX_MEAS];
} naive_kf;

/* One predict + update. Returns 0, or -1 if S is singular. */
int naive_kf_step(naive_kf *kf, const kf_real *z);

#endif

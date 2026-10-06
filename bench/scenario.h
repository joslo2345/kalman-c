#ifndef BENCH_SCENARIO_H
#define BENCH_SCENARIO_H

/* Loader for the frozen scenario files in tests/vectors/ (format: scripts/gen_vectors.py). */

#include "kalman/kalman.h"

typedef enum { MEAS_LINEAR, MEAS_RANGE_BEARING } meas_model;

typedef struct {
    char id[16];
    int n, m, steps, trials;
    double dt;
    meas_model meas;
    kf_real sensor[2]; /* range_bearing only */
    kf_real F[KF_MAX_STATE * KF_MAX_STATE];
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real Q[KF_MAX_STATE * KF_MAX_STATE];
    kf_real R[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_real x0[KF_MAX_STATE];
    kf_real P0[KF_MAX_STATE * KF_MAX_STATE];
    kf_real *truth; /* trials * steps * n; NULL when the data is all zero */
    kf_real *z;     /* trials * steps * m; NULL when the data is all zero */
} scenario;

/* Loads <dir>/<id>.txt. Returns 0, or -1 with a message on stderr. */
int scenario_load(scenario *sc, const char *dir, const char *id);
void scenario_free(scenario *sc);

/* Row accessors; return a shared zero vector for "data zero" scenarios. */
const kf_real *scenario_truth(const scenario *sc, int trial, int k);
const kf_real *scenario_z(const scenario *sc, int trial, int k);

/* Range-bearing from sc->sensor: z (2) and, if H is non-NULL, the Jacobian (2 x n). */
void scenario_range_bearing(const scenario *sc, const kf_real *x, kf_real *z, kf_real *H);

#endif

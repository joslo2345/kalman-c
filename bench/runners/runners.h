#ifndef BENCH_RUNNERS_H
#define BENCH_RUNNERS_H

/*
 * Every library gets a runner with the same signature, so the harness treats
 * them identically. A runner sets up its filter from the scenario and runs
 * predict + update over every trial and step.
 *
 * With measure = 0 (timed runs) a runner does nothing but predict + update.
 * With measure = 1 it also collects the metrics below, outside any timing.
 */

#include "scenario.h"

enum { RUN_OK = 0, RUN_UNSUPPORTED = 1, RUN_FAILED = 2 };

typedef struct {
    /* in */
    int measure;
    kf_real *estimates; /* optional, trials * steps * n; filled when measure is set */
    /* out, when measure is set */
    double rmse;           /* over all trials, steps and state components */
    double nees;           /* average over all trials and steps */
    long steps_to_failure; /* first step where a step failed or P lost positive-
                              definiteness; -1 if never. The run stops there. */
} run_result;

typedef int (*runner_fn)(const scenario *sc, run_result *out);

int run_ours_kf(const scenario *sc, run_result *out);
int run_ours_ekf(const scenario *sc, run_result *out);
int run_ours_ukf(const scenario *sc, run_result *out);
int run_ours_sr(const scenario *sc, run_result *out); /* square-root (UD) KF, linear only */
int run_ours_fx(const scenario *sc, run_result *out); /* fixed-point KF, linear only */
int run_naive(const scenario *sc, run_result *out);
int run_tinyekf(const scenario *sc, run_result *out); /* EKF; dims 2x1, 4x2 and 15x6 */

/* ---- Shared metric collection (metrics.c) ---- */

typedef struct {
    double sse, nees_sum;
    long count;
} metrics;

void metrics_begin(metrics *mt, run_result *out);

/* Records step k of a trial. Returns 1 if the run must stop: the step failed
 * (step_ok == 0) or P is no longer positive-definite. */
int metrics_step(metrics *mt, const scenario *sc, run_result *out, int trial, int k,
                 const kf_real *x, const kf_real *P, int step_ok);

void metrics_end(metrics *mt, const scenario *sc, run_result *out);

#endif

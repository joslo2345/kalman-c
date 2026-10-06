#include "runners.h"

#include <math.h>
#include <string.h>

void metrics_begin(metrics *mt, run_result *out) {
    memset(mt, 0, sizeof *mt);
    out->rmse = NAN;
    out->nees = NAN;
    out->steps_to_failure = -1;
}

int metrics_step(metrics *mt, const scenario *sc, run_result *out, int trial, int k,
                 const kf_real *x, const kf_real *P, int step_ok) {
    kf_real L[KF_MAX_STATE * KF_MAX_STATE], e[KF_MAX_STATE], w[KF_MAX_STATE];
    const int n = sc->n;
    const kf_real *truth = scenario_truth(sc, trial, k);

    if (!step_ok || kf_cholesky(L, P, n) != KF_OK) {
        out->steps_to_failure = k;
        return 1;
    }
    for (int i = 0; i < n; ++i) {
        e[i] = x[i] - truth[i];
        mt->sse += (double)e[i] * (double)e[i];
    }
    kf_solve_lower(w, L, e, n, 1);
    for (int i = 0; i < n; ++i) {
        mt->nees_sum += (double)w[i] * (double)w[i];
    }
    mt->count++;
    if (out->estimates != NULL) {
        memcpy(&out->estimates[((size_t)trial * sc->steps + k) * n], x, (size_t)n * sizeof *x);
    }
    return 0;
}

void metrics_end(metrics *mt, const scenario *sc, run_result *out) {
    if (mt->count > 0) {
        out->rmse = sqrt(mt->sse / ((double)mt->count * sc->n));
        out->nees = mt->nees_sum / (double)mt->count;
    }
}

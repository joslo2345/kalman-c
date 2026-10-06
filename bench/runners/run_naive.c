#include "runners.h"
#include "baselines/naive_kf.h"

#include <string.h>

int run_naive(const scenario *sc, run_result *out) {
    metrics mt;
    if (sc->meas != MEAS_LINEAR) return RUN_UNSUPPORTED;
    if (out->measure) metrics_begin(&mt, out);

    for (int trial = 0; trial < sc->trials; ++trial) {
        naive_kf kf;
        memset(&kf, 0, sizeof kf);
        kf.n = sc->n;
        kf.m = sc->m;
        memcpy(kf.x, sc->x0, (size_t)sc->n * sizeof *kf.x);
        memcpy(kf.P, sc->P0, (size_t)(sc->n * sc->n) * sizeof *kf.P);
        memcpy(kf.F, sc->F, (size_t)(sc->n * sc->n) * sizeof *kf.F);
        memcpy(kf.H, sc->H, (size_t)(sc->m * sc->n) * sizeof *kf.H);
        memcpy(kf.Q, sc->Q, (size_t)(sc->n * sc->n) * sizeof *kf.Q);
        memcpy(kf.R, sc->R, (size_t)(sc->m * sc->m) * sizeof *kf.R);

        for (int k = 0; k < sc->steps; ++k) {
            const int ok = naive_kf_step(&kf, scenario_z(sc, trial, k)) == 0;
            if (out->measure) {
                if (metrics_step(&mt, sc, out, trial, k, kf.x, kf.P, ok)) goto done;
            } else if (!ok) {
                return RUN_FAILED;
            }
        }
    }
done:
    if (out->measure) metrics_end(&mt, sc, out);
    return RUN_OK;
}

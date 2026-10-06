/*
 * One TinyEKF runner per dimension. TinyEKF fixes its sizes at compile time
 * and has no include guard, so each size is its own translation unit that
 * defines EKF_N, EKF_M and TINYEKF_RUN, then includes this file.
 * The filter state (ekf_t) is used directly, with no copying around calls.
 */

#include "runners.h"

#include <string.h>

#define _float_t kf_real
#include "tinyekf.h"

int TINYEKF_RUN(const scenario *sc, run_result *out) {
    ekf_t e;
    kf_real fx[EKF_N], hx[EKF_M], Hrb[EKF_M * EKF_N];
    const kf_real *H = sc->meas == MEAS_LINEAR ? sc->H : Hrb;
    metrics mt;

    if (sc->n != EKF_N || sc->m != EKF_M)
        return RUN_UNSUPPORTED;
    if (out->measure)
        metrics_begin(&mt, out);

    for (int trial = 0; trial < sc->trials; ++trial) {
        memcpy(e.x, sc->x0, sizeof e.x);
        memcpy(e.P, sc->P0, sizeof e.P);
        for (int k = 0; k < sc->steps; ++k) {
            /* TinyEKF takes f(x), h(x) and the Jacobians precomputed by the caller. */
            kf_mat_mul(fx, sc->F, e.x, EKF_N, EKF_N, 1);
            ekf_predict(&e, fx, sc->F, sc->Q);
            if (sc->meas == MEAS_LINEAR) {
                kf_mat_mul(hx, sc->H, e.x, EKF_M, EKF_N, 1);
            } else {
                scenario_range_bearing(sc, e.x, hx, Hrb);
            }
            const int ok = ekf_update(&e, scenario_z(sc, trial, k), hx, H, sc->R);
            if (out->measure) {
                if (metrics_step(&mt, sc, out, trial, k, e.x, e.P, ok))
                    goto done;
            } else if (!ok) {
                return RUN_FAILED;
            }
        }
    }
done:
    if (out->measure)
        metrics_end(&mt, sc, out);
    return RUN_OK;
}

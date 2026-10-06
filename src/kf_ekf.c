#include "kalman/kf_ekf.h"
#include "kalman/kf_linalg.h"
#include "kf_internal.h"

#include <string.h>

int kf_ekf_predict(kf_state *kf, kf_ekf_transition_fn f, void *ctx) {
    kf_real x[KF_MAX_STATE];
    kf_real F[KF_MAX_STATE * KF_MAX_STATE];

    if (kf == NULL || f == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    memset(x, 0, sizeof x);
    memset(F, 0, sizeof F);
    if (f(x, F, kf->x, kf->n, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    return kf_core_predict(kf, x, F);
}

int kf_ekf_update(kf_state *kf, const kf_real *z, kf_ekf_measurement_fn h, void *ctx) {
    kf_real hx[KF_MAX_MEAS];
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real y[KF_MAX_MEAS];

    if (kf == NULL || z == NULL || h == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kf_all_finite(z, kf->m)) {
        return KF_ERR_INVALID_INPUT;
    }
    memset(hx, 0, sizeof hx);
    memset(H, 0, sizeof H);
    if (h(hx, H, kf->x, kf->n, kf->m, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    kf_mat_sub(y, z, hx, kf->m, 1);
    return kf_core_update(kf, y, H);
}

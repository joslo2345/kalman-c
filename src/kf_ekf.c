#include "kalman/kf_ekf.h"
#include "kf_internal.h"
#include "kf_linalg_impl.h"

#include <string.h>

int kf_ekf_predict(kf_state *kf, kf_ekf_transition_fn f, void *ctx) {
    kf_real x[KF_MAX_STATE];
    kf_real F[KF_MAX_STATE * KF_MAX_STATE];

    if (kf == NULL || f == NULL || !kf_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(x, 0, sizeof x);
    (void)memset(F, 0, sizeof F);
    if (f(x, F, kf->x, kf->n, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    return kf_core_predict(kf, x, F);
}

int kf_ekf_update(kf_state *kf, const kf_real *z, kf_ekf_measurement_fn h, void *ctx) {
    kf_real hx[KF_MAX_MEAS];
    kf_real H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_real y[KF_MAX_MEAS];

    if (kf == NULL || z == NULL || h == NULL || !kf_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    if (!kfi_all_finite(z, kf->m)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(hx, 0, sizeof hx);
    (void)memset(H, 0, sizeof H);
    if (h(hx, H, kf->x, kf->n, kf->m, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    kfi_mat_sub(y, z, hx, kf->m, 1);
    return kf_core_update(kf, y, H);
}

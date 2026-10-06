#include "kalman/kf_linear.h"

#include <string.h>

int kf_init(kf_state *kf, int n, int m) {
    if (kf == NULL || n < 1 || n > KF_MAX_STATE || m < 1 || m > KF_MAX_MEAS) {
        return KF_ERR_INVALID_INPUT;
    }
    memset(kf, 0, sizeof *kf);
    kf->n = n;
    kf->m = m;
    return KF_OK;
}

/* TODO(Step 6.2): implement with the Joseph-form update. */
int kf_predict(kf_state *kf, const kf_real *F) {
    if (kf == NULL || F == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    return KF_ERR_NOT_IMPLEMENTED;
}

int kf_update(kf_state *kf, const kf_real *z, const kf_real *H) {
    if (kf == NULL || z == NULL || H == NULL) {
        return KF_ERR_INVALID_INPUT;
    }
    return KF_ERR_NOT_IMPLEMENTED;
}

#include "baselines/baseline_tinyekf.h"

#include <string.h>

#define EKF_N TINYEKF_N
#define EKF_M TINYEKF_M
#define _float_t kf_real
#include "tinyekf.h"

/* Our struct mirrors ekf_t, so the state is copied in and out around each call. */
typedef char layout_check[sizeof(ekf_t) == sizeof(tinyekf_4x2) ? 1 : -1];

void tinyekf_4x2_predict(tinyekf_4x2 *t, const kf_real *fx, const kf_real *F, const kf_real *Q) {
    ekf_t e;
    memcpy(&e, t, sizeof e);
    ekf_predict(&e, fx, F, Q);
    memcpy(t, &e, sizeof e);
}

int tinyekf_4x2_update(tinyekf_4x2 *t, const kf_real *z, const kf_real *hx, const kf_real *H,
                       const kf_real *R) {
    ekf_t e;
    memcpy(&e, t, sizeof e);
    if (!ekf_update(&e, z, hx, H, R)) {
        return -1;
    }
    memcpy(t, &e, sizeof e);
    return 0;
}

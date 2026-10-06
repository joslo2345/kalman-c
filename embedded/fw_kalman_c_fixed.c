#include <string.h>

#include "fw_data.h"
#include "fw_filter.h"
#include "kalman/kalman.h"

#if KF_FX_FRAC != S2_FX_FRAC
#error "the fixed-point data and the library use different Q formats"
#endif

static kf_fx_state kf;

void filter_init(void) {
    kf_fx_init(&kf, S2_N, S2_M);
    memcpy(kf.x, s2_fx_x0, sizeof s2_fx_x0);
    memcpy(kf.P, s2_fx_P0, sizeof s2_fx_P0);
    memcpy(kf.Q, s2_fx_Q, sizeof s2_fx_Q);
    memcpy(kf.R, s2_fx_R, sizeof s2_fx_R);
}

/* Uses the fixed-point copy of measurement row k. */
int filter_step(int k) {
    if (kf_fx_predict(&kf, s2_fx_F) != KF_OK)
        return 1;
    return kf_fx_update(&kf, s2_fx_z[k], s2_fx_H) != KF_OK;
}

const void *filter_x(void) {
    return kf.x;
}

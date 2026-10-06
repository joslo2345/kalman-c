#include <string.h>

#include "fw_data.h"
#include "fw_filter.h"
#include "kalman/kalman.h"

static kf_state kf;

void filter_init(void) {
    kf_init(&kf, S2_N, S2_M);
    memcpy(kf.x, s2_x0, sizeof s2_x0);
    memcpy(kf.P, s2_P0, sizeof s2_P0);
    memcpy(kf.Q, s2_Q, sizeof s2_Q);
    memcpy(kf.R, s2_R, sizeof s2_R);
}

int filter_step(const float *z) {
    if (kf_predict(&kf, s2_F) != KF_OK) return 1;
    return kf_update(&kf, z, s2_H) != KF_OK;
}

const float *filter_x(void) {
    return kf.x;
}

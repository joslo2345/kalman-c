#include <string.h>

#include "fw_data.h"
#include "fw_filter.h"
#include "kalman/kalman.h"

static kf_sr_state sr;

void filter_init(void) {
    kf_sr_init(&sr, S2_N, S2_M);
    memcpy(sr.x, s2_x0, sizeof s2_x0);
    kf_sr_set_P(&sr, s2_P0);
    kf_sr_set_Q(&sr, s2_Q);
    kf_sr_set_R(&sr, s2_R);
}

int filter_step(const float *z) {
    if (kf_sr_predict(&sr, s2_F) != KF_OK)
        return 1;
    return kf_sr_update(&sr, z, s2_H) != KF_OK;
}

const float *filter_x(void) {
    return sr.x;
}

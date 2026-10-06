#include <string.h>

#include "baselines/naive_kf.h"
#include "fw_data.h"
#include "fw_filter.h"

static naive_kf kf;

void filter_init(void) {
    kf.n = S2_N;
    kf.m = S2_M;
    memcpy(kf.x, s2_x0, sizeof s2_x0);
    memcpy(kf.P, s2_P0, sizeof s2_P0);
    memcpy(kf.F, s2_F, sizeof s2_F);
    memcpy(kf.H, s2_H, sizeof s2_H);
    memcpy(kf.Q, s2_Q, sizeof s2_Q);
    memcpy(kf.R, s2_R, sizeof s2_R);
}

int filter_step(int k) {
    return naive_kf_step(&kf, s2_z[k]);
}

const void *filter_x(void) {
    return kf.x;
}

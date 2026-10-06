#include <string.h>

#include "fw_data.h"
#include "fw_filter.h"

#define EKF_N S2_N
#define EKF_M S2_M
#include "tinyekf.h"

static ekf_t ekf;

void filter_init(void) {
    memcpy(ekf.x, s2_x0, sizeof s2_x0);
    memcpy(ekf.P, s2_P0, sizeof s2_P0);
}

int filter_step(int k) {
    float fx[EKF_N], hx[EKF_M];
    _mulvec(s2_F, ekf.x, fx, EKF_N, EKF_N);
    ekf_predict(&ekf, fx, s2_F, s2_Q);
    _mulvec(s2_H, ekf.x, hx, EKF_M, EKF_N);
    return !ekf_update(&ekf, s2_z[k], hx, s2_H, s2_R);
}

const void *filter_x(void) {
    return ekf.x;
}

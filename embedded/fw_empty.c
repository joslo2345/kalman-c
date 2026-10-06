/* Harness only: the same scenario data and loop with no filter, as the
 * baseline for flash, RAM and instruction counts. */
#include "fw_data.h"
#include "fw_filter.h"

static float x[S2_N];
static volatile float sink;

void filter_init(void) {
    /* Reference the model data so the image keeps the same constants. */
    for (int i = 0; i < S2_N * S2_N; ++i)
        sink = s2_F[i] + s2_Q[i] + s2_P0[i];
    for (int i = 0; i < S2_M * S2_N; ++i)
        sink = s2_H[i];
    for (int i = 0; i < S2_M * S2_M; ++i)
        sink = s2_R[i];
    for (int i = 0; i < S2_N; ++i)
        x[i] = s2_x0[i];
}

int filter_step(const float *z) {
    sink = z[0] + z[1];
    return 0;
}

const void *filter_x(void) {
    return x;
}

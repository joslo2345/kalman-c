/* Compiles the single-header implementation and runs it, together with a
 * second translation unit (use_api.c) that only includes the declarations. */
#include <stdio.h>

#define KALMAN_C_IMPLEMENTATION
#include "kalman_c.h"
#include "kalman_c.h" /* a second include must be harmless */

int run_two_steps(kf_state *kf);

int main(void) {
    kf_state kf;
    if (kf_init(&kf, 2, 1) != KF_OK)
        return 1;
    kf.P[0] = kf.P[3] = 10;
    kf.Q[0] = kf.Q[3] = 0.01f;
    kf.R[0] = 0.25f;
    if (run_two_steps(&kf) != 0)
        return 1;
    printf("single header (%s): x = [%.4f, %.4f]\n", KALMAN_C_VERSION, (double)kf.x[0],
           (double)kf.x[1]);
    return (kf.x[0] > 1.5f && kf.x[0] < 2.5f) ? 0 : 1;
}

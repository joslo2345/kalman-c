/* Uses the single-header API without the implementation; linked with impl_main.c. */
#include "kalman_c.h"

int run_two_steps(kf_state *kf) {
    const kf_real F[4] = {1, 1, 0, 1};
    const kf_real H[2] = {1, 0};
    const kf_real z[2][1] = {{1}, {2}};
    for (int k = 0; k < 2; ++k) {
        if (kf_predict(kf, F) != KF_OK || kf_update(kf, z[k], H) != KF_OK) {
            return 1;
        }
    }
    return 0;
}

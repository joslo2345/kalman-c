// A C++ caller (as an Arduino sketch would be). Built twice: against the
// regular headers with libkalman, and against the single header. Fails to link
// if the public headers lose their extern "C" guards.
#include <cstdio>

#ifdef KF_TEST_SINGLE_HEADER
#include "kalman_c.h"
#else
#include "kalman/kalman.h"
#endif

int main() {
    const kf_real F[4] = {1, 1, 0, 1};
    const kf_real H[2] = {1, 0};
    const kf_real z[1] = {1};
    kf_state kf;
    if (kf_init(&kf, 2, 1) != KF_OK)
        return 1;
    kf.P[0] = kf.P[3] = 10;
    kf.R[0] = 0.25f;
    if (kf_predict(&kf, F) != KF_OK || kf_update(&kf, z, H) != KF_OK)
        return 1;
    std::printf("C++ caller: x = [%.4f, %.4f]\n", static_cast<double>(kf.x[0]),
                static_cast<double>(kf.x[1]));
    return 0;
}

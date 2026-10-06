/*
 * 1D tracking with the linear Kalman filter.
 *
 * A target moves at a roughly constant velocity; a sensor measures its
 * position with noise (sigma = 2 m). The filter estimates position and
 * velocity, and its position error should be well below the sensor's.
 *
 * State x = [position, velocity], measurement z = [position].
 */
#include <math.h>
#include <stdio.h>

#include "kalman/kalman.h"

/* Deterministic noise so the example always prints the same numbers. */
static unsigned long long rng_state = 12345u;
static double gauss(void) {
    double u[2];
    for (int i = 0; i < 2; ++i) {
        rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
        u[i] = ((double)(rng_state >> 11) + 0.5) / 9007199254740992.0;
    }
    return sqrt(-2.0 * log(u[0])) * cos(6.283185307179586 * u[1]);
}

int main(void) {
    const kf_real dt = 0.1f, accel_sigma = 0.2f, meas_sigma = 2.0f;
    const kf_real F[4] = {1, dt, 0, 1};
    const kf_real H[2] = {1, 0};
    kf_state kf;

    kf_init(&kf, 2, 1);
    kf.P[0] = 100; /* position unknown to about 10 m */
    kf.P[3] = 25;  /* velocity unknown to about 5 m/s */
    /* Process noise from a random acceleration with standard deviation accel_sigma. */
    const kf_real q = accel_sigma * accel_sigma;
    kf.Q[0] = q * dt * dt * dt * dt / 4;
    kf.Q[1] = kf.Q[2] = q * dt * dt * dt / 2;
    kf.Q[3] = q * dt * dt;
    kf.R[0] = meas_sigma * meas_sigma;

    double pos = 0, vel = 3, filter_se = 0, sensor_se = 0;
    const int steps = 600;
    for (int k = 0; k < steps; ++k) {
        const double a = accel_sigma * gauss();
        pos += vel * dt + 0.5 * a * dt * dt;
        vel += a * dt;
        const kf_real z[1] = {(kf_real)(pos + meas_sigma * gauss())};

        if (kf_predict(&kf, F) != KF_OK || kf_update(&kf, z, H) != KF_OK) {
            fprintf(stderr, "filter step %d failed\n", k);
            return 1;
        }
        if (k >= 100) { /* skip the initial transient */
            filter_se += (kf.x[0] - pos) * (kf.x[0] - pos);
            sensor_se += (z[0] - pos) * (z[0] - pos);
        }
    }

    const double filter_rmse = sqrt(filter_se / (steps - 100));
    const double sensor_rmse = sqrt(sensor_se / (steps - 100));
    printf("true:     position %8.3f m, velocity %6.3f m/s\n", pos, vel);
    printf("estimate: position %8.3f m, velocity %6.3f m/s\n", (double)kf.x[0], (double)kf.x[1]);
    printf("position RMSE: sensor %.3f m, filter %.3f m\n", sensor_rmse, filter_rmse);
    printf("last NIS: %.3f (averages 1 for a consistent filter)\n", (double)kf.nis);
    return filter_rmse < 0.5 * sensor_rmse ? 0 : 1;
}

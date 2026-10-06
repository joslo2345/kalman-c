/*
 * Roll and pitch from a gyroscope and an accelerometer, with the EKF and the UKF.
 *
 * The gyroscope gives body rates (p, q, r), integrated in the predict step.
 * Integration alone drifts, so the accelerometer, which measures gravity in the
 * body frame, corrects it in the update step. Both filters run on the same
 * simulated IMU data, and both should beat the accelerometer-only angles.
 *
 * State x = [roll, pitch] in radians, measurement z = specific force (3 axes).
 */
#include <math.h>
#include <stdio.h>

#include "kalman/kalman.h"

#define G 9.81

typedef struct {
    kf_real dt;
    kf_real gyro[3]; /* p, q, r in rad/s for the current step */
} imu_input;

/* Euler-angle kinematics: roll' = p + (q sin roll + r cos roll) tan pitch,
 * pitch' = q cos roll - r sin roll, integrated over dt. */
static void attitude_step(kf_real *x_out, const kf_real *x, const imu_input *in) {
    const double phi = x[0], theta = x[1], p = in->gyro[0], q = in->gyro[1], r = in->gyro[2];
    x_out[0] = (kf_real)(phi + in->dt * (p + (q * sin(phi) + r * cos(phi)) * tan(theta)));
    x_out[1] = (kf_real)(theta + in->dt * (q * cos(phi) - r * sin(phi)));
}

/* Accelerometer at rest measures the reaction to gravity, rotated into the body frame. */
static void gravity_body(kf_real *z, const kf_real *x) {
    const double phi = x[0], theta = x[1];
    z[0] = (kf_real)(G * sin(theta));
    z[1] = (kf_real)(-G * cos(theta) * sin(phi));
    z[2] = (kf_real)(-G * cos(theta) * cos(phi));
}

/* ---- EKF callbacks: model and Jacobian together ---- */

static int ekf_f(kf_real *x_out, kf_real *F, const kf_real *x, int n, void *ctx) {
    const imu_input *in = ctx;
    const double phi = x[0], theta = x[1], q = in->gyro[1], r = in->gyro[2], dt = in->dt;
    const double t = tan(theta), c = cos(theta);
    (void)n;
    attitude_step(x_out, x, in);
    F[0] = (kf_real)(1 + dt * (q * cos(phi) - r * sin(phi)) * t);
    F[1] = (kf_real)(dt * (q * sin(phi) + r * cos(phi)) / (c * c));
    F[2] = (kf_real)(dt * (-q * sin(phi) - r * cos(phi)));
    F[3] = 1;
    return 0;
}

static int ekf_h(kf_real *z, kf_real *H, const kf_real *x, int n, int m, void *ctx) {
    const double phi = x[0], theta = x[1];
    (void)n;
    (void)m;
    (void)ctx;
    gravity_body(z, x);
    H[0] = 0; /* d/droll, d/dpitch for each axis */
    H[1] = (kf_real)(G * cos(theta));
    H[2] = (kf_real)(-G * cos(theta) * cos(phi));
    H[3] = (kf_real)(G * sin(theta) * sin(phi));
    H[4] = (kf_real)(G * cos(theta) * sin(phi));
    H[5] = (kf_real)(G * sin(theta) * cos(phi));
    return 0;
}

/* ---- UKF callbacks: model only ---- */

static int ukf_f(kf_real *x_out, const kf_real *x, int n, void *ctx) {
    (void)n;
    attitude_step(x_out, x, ctx);
    return 0;
}

static int ukf_h(kf_real *z, const kf_real *x, int n, int m, void *ctx) {
    (void)n;
    (void)m;
    (void)ctx;
    gravity_body(z, x);
    return 0;
}

static unsigned long long rng_state = 2024u;
static double gauss(void) {
    double u[2];
    for (int i = 0; i < 2; ++i) {
        rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
        u[i] = ((double)(rng_state >> 11) + 0.5) / 9007199254740992.0;
    }
    return sqrt(-2.0 * log(u[0])) * cos(6.283185307179586 * u[1]);
}

static void setup(kf_state *kf, kf_real dt, kf_real gyro_sigma, kf_real accel_sigma) {
    kf_init(kf, 2, 3);
    kf->P[0] = kf->P[3] = 0.1f; /* about 18 degrees of initial uncertainty */
    kf->Q[0] = kf->Q[3] = gyro_sigma * gyro_sigma * dt * dt;
    for (int i = 0; i < 3; ++i)
        kf->R[i * 3 + i] = accel_sigma * accel_sigma;
}

int main(void) {
    const kf_real dt = 0.01f, gyro_sigma = 0.02f, accel_sigma = 0.5f;
    const double deg = 57.29577951308232;
    imu_input in = {dt, {0, 0, 0}};
    kf_state ekf, ukf;
    double se_ekf = 0, se_ukf = 0, se_accel = 0;

    setup(&ekf, dt, gyro_sigma, accel_sigma);
    setup(&ukf, dt, gyro_sigma, accel_sigma);

    const int steps = 2000; /* 20 s at 100 Hz */
    for (int k = 1; k <= steps; ++k) {
        /* Truth: slow roll and pitch oscillations, with zero yaw rate. */
        const double t = k * dt, w1 = 0.5, w2 = 0.3;
        const double phi = 0.4 * sin(w1 * t), theta = 0.25 * sin(w2 * t);
        const double phi_dot = 0.4 * w1 * cos(w1 * t), theta_dot = 0.25 * w2 * cos(w2 * t);
        const double q = theta_dot / cos(phi);                /* from pitch' with r = 0 */
        const double p = phi_dot - q * sin(phi) * tan(theta); /* from roll' */
        const kf_real truth[2] = {(kf_real)phi, (kf_real)theta};

        in.gyro[0] = (kf_real)(p + gyro_sigma * gauss());
        in.gyro[1] = (kf_real)(q + gyro_sigma * gauss());
        in.gyro[2] = (kf_real)(gyro_sigma * gauss());
        kf_real z[3];
        gravity_body(z, truth);
        for (int i = 0; i < 3; ++i)
            z[i] += (kf_real)(accel_sigma * gauss());

        if (kf_ekf_predict(&ekf, ekf_f, &in) != KF_OK ||
            kf_ekf_update(&ekf, z, ekf_h, NULL) != KF_OK ||
            kf_ukf_predict(&ukf, NULL, ukf_f, &in) != KF_OK ||
            kf_ukf_update(&ukf, z, NULL, ukf_h, NULL) != KF_OK) {
            fprintf(stderr, "filter step %d failed\n", k);
            return 1;
        }

        /* Accelerometer-only angles, for comparison. */
        const double roll_acc = atan2(-z[1], -z[2]);
        const double pitch_acc = atan2(z[0], sqrt(z[1] * z[1] + z[2] * z[2]));
        if (k > 200) { /* skip the first 2 s while the filters converge */
            se_ekf += pow(ekf.x[0] - phi, 2) + pow(ekf.x[1] - theta, 2);
            se_ukf += pow(ukf.x[0] - phi, 2) + pow(ukf.x[1] - theta, 2);
            se_accel += pow(roll_acc - phi, 2) + pow(pitch_acc - theta, 2);
        }
    }

    const double n = 2.0 * (steps - 200);
    const double rmse_ekf = sqrt(se_ekf / n) * deg, rmse_ukf = sqrt(se_ukf / n) * deg;
    const double rmse_accel = sqrt(se_accel / n) * deg;
    printf("roll/pitch RMSE: accelerometer only %.2f deg, EKF %.2f deg, UKF %.2f deg\n", rmse_accel,
           rmse_ekf, rmse_ukf);
    return (rmse_ekf < 0.5 * rmse_accel && rmse_ukf < 0.5 * rmse_accel) ? 0 : 1;
}

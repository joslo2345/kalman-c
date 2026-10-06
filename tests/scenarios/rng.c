#include "scenarios/rng.h"

#include <math.h>

void rng_seed(rng *r, uint64_t seed) {
    r->state = seed;
    r->has_spare = 0;
    r->spare = 0;
}

/* splitmix64 */
static uint64_t next_u64(rng *r) {
    uint64_t z = (r->state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

double rng_uniform(rng *r) {
    return ((double)(next_u64(r) >> 11) + 0.5) / 9007199254740992.0; /* 2^53 */
}

/* Box-Muller */
double rng_gauss(rng *r) {
    if (r->has_spare) {
        r->has_spare = 0;
        return r->spare;
    }
    const double u1 = rng_uniform(r), u2 = rng_uniform(r);
    const double mag = sqrt(-2.0 * log(u1));
    const double two_pi = 6.283185307179586;
    r->spare = mag * sin(two_pi * u2);
    r->has_spare = 1;
    return mag * cos(two_pi * u2);
}

void rng_gauss_vec(rng *r, double *x, const double *L, int n) {
    double g[64];
    for (int i = 0; i < n; ++i)
        g[i] = rng_gauss(r);
    for (int i = 0; i < n; ++i) {
        x[i] = 0;
        for (int j = 0; j <= i; ++j)
            x[i] += L[i * n + j] * g[j];
    }
}

int rng_cholesky(double *L, const double *A, int n) {
    for (int i = 0; i < n * n; ++i)
        L[i] = 0;
    for (int j = 0; j < n; ++j) {
        double d = A[j * n + j];
        for (int p = 0; p < j; ++p)
            d -= L[j * n + p] * L[j * n + p];
        if (!(d > 0))
            return -1;
        L[j * n + j] = sqrt(d);
        for (int i = j + 1; i < n; ++i) {
            double s = A[i * n + j];
            for (int p = 0; p < j; ++p)
                s -= L[i * n + p] * L[j * n + p];
            L[i * n + j] = s / L[j * n + j];
        }
    }
    return 0;
}

#ifndef SCENARIO_RNG_H
#define SCENARIO_RNG_H

/* Deterministic, platform-independent random numbers for scenarios. */

#include <stdint.h>

typedef struct {
    uint64_t state;
    int has_spare;
    double spare;
} rng;

void rng_seed(rng *r, uint64_t seed);
double rng_uniform(rng *r); /* in (0, 1) */
double rng_gauss(rng *r);   /* standard normal */

/* x (n) = L * N(0, I) for lower-triangular L (n x n, row-major, double) */
void rng_gauss_vec(rng *r, double *x, const double *L, int n);

/* Lower Cholesky factor of A (n x n, double). Returns 0, or -1 if not positive-definite. */
int rng_cholesky(double *L, const double *A, int n);

#endif

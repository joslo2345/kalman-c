#include "kalman/kf_fixed.h"

#include <string.h>

/*
 * Fixed-point Kalman filter. Values are int32 with F = KF_FX_FRAC fractional
 * bits. Products of two values are int64 numbers with 2F fractional bits
 * ("wide" values below). Dot products drop FX_GUARD low bits from each
 * product so their sums cannot overflow (see wide_dot), and are rounded to F
 * bits once. Every narrowing to int32 is range-checked. Any overflow sets
 * *ovf, and the public functions then return KF_ERR_OVERFLOW without touching
 * the filter state.
 *
 * Rounding never right-shifts a negative number (implementation-defined in
 * C99): magnitudes are rounded and the sign reapplied.
 */

#define FX_F KF_FX_FRAC

/* 2^F, the scale between the Q format and real values (shifts stay unsigned). */
static const uint64_t fx_one_u = (uint64_t)1U << (unsigned)FX_F;
#define FX_ONE_WIDE ((int64_t)fx_one_u)

/* Guard bits for wide_dot: 2^FX_GUARD >= KF_MAX_DIM, the longest dot product. */
#if KF_MAX_DIM <= 4
#define FX_GUARD 2
#elif KF_MAX_DIM <= 16
#define FX_GUARD 4
#elif KF_MAX_DIM <= 64
#define FX_GUARD 6
#elif KF_MAX_DIM <= 256
#define FX_GUARD 8
#else
#error "KF_MAX_DIM above 256 is not supported by the fixed-point filter"
#endif

static int fx_dims_ok(const kf_fx_state *kf) {
    return (kf->n >= 1) && (kf->n <= KF_MAX_STATE) && (kf->m >= 1) && (kf->m <= KF_MAX_MEAS);
}

/* The int64 limits, derived without the INT64_* macros, whose spelling differs
 * between C libraries and trips static checkers on some of them. */
static int64_t wide_max(void) {
    const uint64_t umax = ~(uint64_t)0U >> 1U;
    return (int64_t)umax;
}

/* a + b, or *ovf = 1. */
static int64_t wide_add(int64_t a, int64_t b, int *ovf) {
    const int64_t max = wide_max();
    const int64_t min = -max - 1;
    if (((b > 0) && (a > (max - b))) || ((b < 0) && (a < (min - b)))) {
        *ovf = 1;
        return 0;
    }
    return a + b;
}

/* a - b, or *ovf = 1. */
static int64_t wide_sub(int64_t a, int64_t b, int *ovf) {
    const int64_t max = wide_max();
    const int64_t min = -max - 1;
    if (((b < 0) && (a > (max + b))) || ((b > 0) && (a < (min + b)))) {
        *ovf = 1;
        return 0;
    }
    return a - b;
}

/* A Q-format value as a wide (2F) value. Exact: |v| < 2^31, so |v| 2^F < 2^59. */
static int64_t to_wide(kf_fx v) {
    return (int64_t)v * FX_ONE_WIDE;
}

/* v / 2^shift rounded to nearest, ties away from zero, without shifting negatives. */
static int64_t round_shift(int64_t v, int shift) {
    const unsigned s = (unsigned)shift;
    const unsigned s_half = s - 1U;
    const uint64_t half = (uint64_t)1U << s_half;
    if (v >= 0) {
        const uint64_t mag = (uint64_t)v;
        const uint64_t r = (mag + half) >> s;
        return (int64_t)r;
    }
    const int64_t v_plus_1 = v + 1;
    const uint64_t mag = (uint64_t)(-v_plus_1) + 1U; /* |v|, also for INT64_MIN */
    const uint64_t r = (mag + half) >> s;
    return -(int64_t)r;
}

/* Narrow to the Q format, or *ovf = 1. */
static kf_fx narrow(int64_t v, int *ovf) {
    const int64_t fx_max = INT32_MAX;
    const int64_t fx_min = INT32_MIN;
    if ((v > fx_max) || (v < fx_min)) {
        *ovf = 1;
        return 0;
    }
    return (kf_fx)v;
}

/* A wide (2F) value rounded to the Q format. */
static kf_fx from_wide(int64_t w, int *ovf) {
    return narrow(round_shift(w, FX_F), ovf);
}

/*
 * sum_k a[k * sa] * b[k * sb], as a wide value.
 *
 * Each product is below 2^62 in magnitude. Dividing it by 2^FX_GUARD (which is
 * at least len) first makes the running sum provably below 2^62, so the loop
 * needs no overflow checks; one check remains when scaling back up. The
 * dropped bits are at most len * 2^(FX_GUARD - 2F) in real terms, far below
 * the 2^-F rounding step. (Division, not a shift: right-shifting a negative
 * value is implementation-defined in C99.) On a Cortex-M3, a per-term check
 * cost almost half of the filter's instructions.
 */
static int64_t wide_dot(const kf_fx *a, int sa, const kf_fx *b, int sb, int len, int *ovf) {
    const uint64_t guard_u = (uint64_t)1U << (unsigned)FX_GUARD;
    const int64_t fx_guard_scale = (int64_t)guard_u;
    int64_t s = 0;
    for (int k = 0; k < len; ++k) {
        s += ((int64_t)a[k * sa] * (int64_t)b[k * sb]) / fx_guard_scale;
    }
    const int64_t limit = wide_max() / fx_guard_scale;
    if ((s > limit) || (s < -limit)) {
        *ovf = 1;
        return 0;
    }
    return s * fx_guard_scale;
}

/* A wide numerator divided by a Q-format denominator, rounded, as a Q-format value. */
static kf_fx div_wide(int64_t num, kf_fx den, int *ovf) {
    int64_t q = num / (int64_t)den; /* C99: truncates toward zero */
    const int64_t r = num % (int64_t)den;
    const int64_t r_mag = (r < 0) ? -r : r;
    const int64_t d_mag = (den < 0) ? -(int64_t)den : (int64_t)den;
    if ((2 * r_mag) >= d_mag) {
        q += ((num < 0) != (den < 0)) ? -1 : 1;
    }
    return narrow(q, ovf);
}

/* floor(sqrt(value)). */
static uint64_t isqrt64(uint64_t value) {
    uint64_t v = value;
    uint64_t res = 0;
    uint64_t bit = (uint64_t)1U << 62U;
    while (bit > v) {
        bit >>= 2U;
    }
    while (bit != 0U) {
        if (v >= (res + bit)) {
            v -= res + bit;
            res = (res >> 1U) + bit;
        } else {
            res >>= 1U;
        }
        bit >>= 2U;
    }
    return res;
}

/* Cholesky A = L L^T (n x n). The square root of a wide value is a Q-format value. */
static int fx_cholesky(kf_fx *L, const kf_fx *A, int n, int *ovf) {
    for (int j = 0; j < n; ++j) {
        const int64_t d =
            wide_sub(to_wide(A[j * n + j]), wide_dot(&L[j * n], 1, &L[j * n], 1, j, ovf), ovf);
        if (d <= 0) {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        const kf_fx ljj = narrow((int64_t)isqrt64((uint64_t)d), ovf);
        if (ljj <= 0) {
            return KF_ERR_NOT_POSITIVE_DEFINITE;
        }
        L[j * n + j] = ljj;
        for (int i = j + 1; i < n; ++i) {
            const int64_t num =
                wide_sub(to_wide(A[i * n + j]), wide_dot(&L[i * n], 1, &L[j * n], 1, j, ovf), ovf);
            L[i * n + j] = div_wide(num, ljj, ovf);
        }
        for (int i = 0; i < j; ++i) {
            L[i * n + j] = 0;
        }
    }
    return KF_OK;
}

/* Solve L L^T X = B in place for X (n x c, stride c), L lower triangular (n x n). */
static void fx_cholesky_solve(kf_fx *X, const kf_fx *L, int n, int c, int *ovf) {
    for (int col = 0; col < c; ++col) {
        for (int i = 0; i < n; ++i) { /* L Y = B */
            const int64_t num =
                wide_sub(to_wide(X[i * c + col]), wide_dot(&L[i * n], 1, &X[col], c, i, ovf), ovf);
            X[i * c + col] = div_wide(num, L[i * n + i], ovf);
        }
        for (int i = n - 1; i >= 0; --i) { /* L^T X = Y */
            const int len = n - 1 - i;
            const int64_t num =
                wide_sub(to_wide(X[i * c + col]),
                         wide_dot(&L[(i + 1) * n + i], n, &X[(i + 1) * c + col], c, len, ovf), ovf);
            X[i * c + col] = div_wide(num, L[i * n + i], ovf);
        }
    }
}

int kf_fx_init(kf_fx_state *kf, int n, int m) {
    if ((kf == NULL) || (n < 1) || (n > KF_MAX_STATE) || (m < 1) || (m > KF_MAX_MEAS)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(kf, 0, sizeof *kf);
    kf->n = n;
    kf->m = m;
    return KF_OK;
}

/* Given the predicted state and transition matrix (or Jacobian) F, set
 * P = F P F^T + Q (upper triangle mirrored). Commits only on success. */
static int fx_core_predict(kf_fx_state *kf, const kf_fx *x, const kf_fx *F) {
    kf_fx FP[KF_MAX_STATE * KF_MAX_STATE];
    kf_fx P[KF_MAX_STATE * KF_MAX_STATE];
    int ovf = 0;

    /* Re-checked here: an EKF model callback receives ctx, which may alias *kf. */
    if (!fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;

    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < n; ++k) {
            FP[i * n + k] = from_wide(wide_dot(&F[i * n], 1, &kf->P[k], n, n, &ovf), &ovf);
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) {
            const int64_t q2 = to_wide(kf->Q[i * n + j]) + to_wide(kf->Q[j * n + i]); /* < 2^60 */
            const int64_t fpf = wide_dot(&FP[i * n], 1, &F[j * n], 1, n, &ovf);
            const int64_t w =
                wide_add(wide_add(fpf, fpf, &ovf), q2, &ovf); /* 2 FPF^T_ij + Q_ij + Q_ji */
            const kf_fx v = narrow(round_shift(w, FX_F + 1), &ovf);
            P[i * n + j] = v;
            P[j * n + i] = v;
        }
    }
    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    (void)memcpy(kf->x, x, (size_t)n * sizeof *x);
    (void)memcpy(kf->P, P, (size_t)n * (size_t)n * sizeof *P);
    return KF_OK;
}

/* x = F x, P = F P F^T + Q. */
int kf_fx_predict(kf_fx_state *kf, const kf_fx *F) {
    kf_fx x[KF_MAX_STATE];
    int ovf = 0;

    if ((kf == NULL) || (F == NULL) || !fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    for (int i = 0; i < n; ++i) {
        x[i] = from_wide(wide_dot(&F[i * n], 1, kf->x, 1, n, &ovf), &ovf);
    }
    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    return fx_core_predict(kf, x, F);
}

/*
 * Joseph-form update in the same O(n^2 m) arrangement as kf_linear.c:
 *   S = H P H^T + R, S K^T = H P, B = P - K (H P),
 *   P = (B - (B H^T) K^T) + K R K^T (upper triangle, symmetrized).
 * Given the innovation y and the measurement matrix (or Jacobian) H.
 */
static int fx_core_update(kf_fx_state *kf, const kf_fx *y, const kf_fx *H) {
    kf_fx w[KF_MAX_MEAS];
    kf_fx HP[KF_MAX_MEAS * KF_MAX_STATE];
    kf_fx S[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_fx L[KF_MAX_MEAS * KF_MAX_MEAS];
    kf_fx Kt[KF_MAX_MEAS * KF_MAX_STATE];
    kf_fx RKt[KF_MAX_MEAS * KF_MAX_STATE];
    kf_fx B[KF_MAX_STATE * KF_MAX_STATE];
    kf_fx BHt[KF_MAX_STATE * KF_MAX_MEAS];
    kf_fx P[KF_MAX_STATE * KF_MAX_STATE];
    kf_fx x[KF_MAX_STATE];
    int ovf = 0;
    int status;

    /* Re-checked here: an EKF model callback receives ctx, which may alias *kf. */
    if (!fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    const int m = kf->m;

    for (int a = 0; a < m; ++a) { /* H P */
        for (int c = 0; c < n; ++c) {
            HP[a * n + c] = from_wide(wide_dot(&H[a * n], 1, &kf->P[c], n, n, &ovf), &ovf);
        }
    }
    for (int a = 0; a < m; ++a) { /* S = H P H^T + R, symmetrized */
        for (int b = a; b < m; ++b) {
            int64_t s = wide_add(wide_dot(&HP[a * n], 1, &H[b * n], 1, n, &ovf),
                                 wide_dot(&HP[b * n], 1, &H[a * n], 1, n, &ovf), &ovf);
            s = wide_add(s, wide_add(to_wide(kf->R[a * m + b]), to_wide(kf->R[b * m + a]), &ovf),
                         &ovf);
            const kf_fx v = narrow(round_shift(s, FX_F + 1), &ovf);
            S[a * m + b] = v;
            S[b * m + a] = v;
        }
    }
    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    status = fx_cholesky(L, S, m, &ovf);
    if (status != (int)KF_OK) {
        return status;
    }

    (void)memcpy(Kt, HP, (size_t)m * (size_t)n * sizeof *HP);
    fx_cholesky_solve(Kt, L, m, n, &ovf); /* S K^T = H P */

    kf_fx nis;
    (void)memcpy(w, y, (size_t)m * sizeof *y);
    for (int i = 0; i < m; ++i) { /* w = L^-1 y */
        const int64_t num = wide_sub(to_wide(w[i]), wide_dot(&L[i * m], 1, w, 1, i, &ovf), &ovf);
        w[i] = div_wide(num, L[i * m + i], &ovf);
    }
    nis = from_wide(wide_dot(w, 1, w, 1, m, &ovf), &ovf);

    for (int i = 0; i < n; ++i) { /* x = x + K y */
        x[i] =
            from_wide(wide_add(to_wide(kf->x[i]), wide_dot(&Kt[i], n, y, 1, m, &ovf), &ovf), &ovf);
        for (int c = 0; c < n; ++c) { /* B = P - K (H P) */
            B[i * n + c] = from_wide(
                wide_sub(to_wide(kf->P[i * n + c]), wide_dot(&Kt[i], n, &HP[c], n, m, &ovf), &ovf),
                &ovf);
        }
    }
    for (int i = 0; i < n; ++i) { /* B H^T */
        for (int j = 0; j < m; ++j) {
            BHt[i * m + j] = from_wide(wide_dot(&B[i * n], 1, &H[j * n], 1, n, &ovf), &ovf);
        }
    }
    for (int a = 0; a < m; ++a) { /* R K^T */
        for (int c = 0; c < n; ++c) {
            RKt[a * n + c] = from_wide(wide_dot(&kf->R[a * m], 1, &Kt[c], n, m, &ovf), &ovf);
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int c = i; c < n; ++c) {
            /* 2 P_ic = (B_ic + B_ci) - ((BH^T)_i K^T_c + (BH^T)_c K^T_i) + 2 (K R K^T)_ic */
            int64_t t = wide_add(to_wide(B[i * n + c]), to_wide(B[c * n + i]), &ovf);
            t = wide_sub(t, wide_dot(&BHt[i * m], 1, &Kt[c], n, m, &ovf), &ovf);
            t = wide_sub(t, wide_dot(&BHt[c * m], 1, &Kt[i], n, m, &ovf), &ovf);
            const int64_t krk = wide_dot(&Kt[i], n, &RKt[c], n, m, &ovf);
            t = wide_add(wide_add(t, krk, &ovf), krk, &ovf);
            const kf_fx v = narrow(round_shift(t, FX_F + 1), &ovf);
            P[i * n + c] = v;
            P[c * n + i] = v;
        }
    }

    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    (void)memcpy(kf->x, x, (size_t)n * sizeof *x);
    (void)memcpy(kf->P, P, (size_t)n * (size_t)n * sizeof *P);
    kf->nis = nis;
    return KF_OK;
}

/* y = z - H x, then the Joseph-form update. */
int kf_fx_update(kf_fx_state *kf, const kf_fx *z, const kf_fx *H) {
    kf_fx y[KF_MAX_MEAS];
    int ovf = 0;

    if ((kf == NULL) || (z == NULL) || (H == NULL) || !fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    const int n = kf->n;
    for (int a = 0; a < kf->m; ++a) {
        y[a] = from_wide(wide_sub(to_wide(z[a]), wide_dot(&H[a * n], 1, kf->x, 1, n, &ovf), &ovf),
                         &ovf);
    }
    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    return fx_core_update(kf, y, H);
}

/* ---- Extended filter: the same cores, with the model from callbacks ---- */

int kf_fx_ekf_predict(kf_fx_state *kf, kf_fx_transition_fn f, void *ctx) {
    kf_fx x[KF_MAX_STATE];
    kf_fx F[KF_MAX_STATE * KF_MAX_STATE];

    if ((kf == NULL) || (f == NULL) || !fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(x, 0, sizeof x);
    (void)memset(F, 0, sizeof F);
    if (f(x, F, kf->x, kf->n, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    return fx_core_predict(kf, x, F);
}

int kf_fx_ekf_update(kf_fx_state *kf, const kf_fx *z, kf_fx_measurement_fn h, void *ctx) {
    kf_fx hx[KF_MAX_MEAS];
    kf_fx H[KF_MAX_MEAS * KF_MAX_STATE];
    kf_fx y[KF_MAX_MEAS];
    int ovf = 0;

    if ((kf == NULL) || (z == NULL) || (h == NULL) || !fx_dims_ok(kf)) {
        return KF_ERR_INVALID_INPUT;
    }
    (void)memset(hx, 0, sizeof hx);
    (void)memset(H, 0, sizeof H);
    if (h(hx, H, kf->x, kf->n, kf->m, ctx) != 0) {
        return KF_ERR_MODEL_FAILED;
    }
    for (int a = 0; (a < kf->m) && (a < KF_MAX_MEAS); ++a) { /* y = z - h(x), range-checked */
        y[a] = narrow((int64_t)z[a] - (int64_t)hx[a], &ovf);
    }
    if (ovf != 0) {
        return KF_ERR_OVERFLOW;
    }
    return fx_core_update(kf, y, H);
}

int kf_fx_from_double(double v, kf_fx *out) {
    if ((out == NULL) || (v != v)) { /* v != v: NaN */
        return KF_ERR_INVALID_INPUT;
    }
    const double scaled = v * (double)FX_ONE_WIDE;
    const double r = (scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5);
    if ((r >= 2147483648.0) || (r <= -2147483649.0)) {
        return KF_ERR_OVERFLOW;
    }
    *out = (kf_fx)(int64_t)r; /* truncation of r = rounding of scaled */
    return KF_OK;
}

double kf_fx_to_double(kf_fx v) {
    return (double)v / (double)FX_ONE_WIDE;
}

#include "scenario.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const kf_real zero_row[KF_MAX_STATE + KF_MAX_MEAS];

static int read_values(FILE *f, kf_real *out, int count, const char *key, const char *expected) {
    char tok[64];
    if (fscanf(f, "%63s", tok) != 1 || strcmp(tok, expected) != 0) {
        fprintf(stderr, "scenario: expected '%s' but found '%s'\n", expected, key);
        return -1;
    }
    for (int i = 0; i < count; ++i) {
        double v;
        if (fscanf(f, "%lf", &v) != 1) {
            fprintf(stderr, "scenario: not enough values for '%s'\n", expected);
            return -1;
        }
        out[i] = (kf_real)v;
    }
    return 0;
}

static int read_int(FILE *f, const char *expected, int *out) {
    char tok[64];
    if (fscanf(f, "%63s %d", tok, out) != 2 || strcmp(tok, expected) != 0) {
        fprintf(stderr, "scenario: expected '%s'\n", expected);
        return -1;
    }
    return 0;
}

/* Fill the Q-format copies; fx_ok = 0 if any value is out of range. */
static int to_fx(const kf_real *src, kf_fx *dst, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (kf_fx_from_double((double)src[i], &dst[i]) != KF_OK) {
            return 0;
        }
    }
    return 1;
}

static void scenario_convert_fx(scenario *sc) {
    const size_t n = (size_t)sc->n, m = (size_t)sc->m;
    sc->fx_ok = to_fx(sc->F, sc->fx_F, n * n) && to_fx(sc->H, sc->fx_H, m * n) &&
                to_fx(sc->Q, sc->fx_Q, n * n) && to_fx(sc->R, sc->fx_R, m * m) &&
                to_fx(sc->x0, sc->fx_x0, n) && to_fx(sc->P0, sc->fx_P0, n * n);
    if (sc->fx_ok && sc->z != NULL) {
        const size_t len = (size_t)sc->trials * (size_t)sc->steps * m;
        sc->fx_z = malloc(len * sizeof *sc->fx_z);
        sc->fx_ok = sc->fx_z != NULL && to_fx(sc->z, sc->fx_z, len);
    }
}

int scenario_load(scenario *sc, const char *dir, const char *id) {
    char path[1024], tok[64], line[256];
    int version;
    FILE *f;

    memset(sc, 0, sizeof *sc);
    snprintf(path, sizeof path, "%s/%s.txt", dir, id);
    f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "scenario: cannot open %s\n", path);
        return -1;
    }

    if (fscanf(f, "%63s %d", tok, &version) != 2 || strcmp(tok, "kalman-vectors") != 0 ||
        version != 1) {
        fprintf(stderr, "scenario: %s is not a version 1 kalman-vectors file\n", path);
        goto fail;
    }
    if (fscanf(f, " id %15s", sc->id) != 1)
        goto fail;
    if (fscanf(f, " description%255[^\n]", line) != 1)
        goto fail;
    if (read_int(f, "n", &sc->n) || read_int(f, "m", &sc->m) || read_int(f, "steps", &sc->steps) ||
        read_int(f, "trials", &sc->trials)) {
        goto fail;
    }
    if (sc->n > KF_MAX_STATE || sc->m > KF_MAX_MEAS) {
        fprintf(stderr, "scenario: %s needs KF_MAX_STATE >= %d and KF_MAX_MEAS >= %d\n", id, sc->n,
                sc->m);
        goto fail;
    }
    if (fscanf(f, " dt %lf measurement %63s", &sc->dt, tok) != 2)
        goto fail;
    if (strcmp(tok, "linear") == 0) {
        sc->meas = MEAS_LINEAR;
    } else if (strcmp(tok, "range_bearing") == 0) {
        double sx, sy;
        if (fscanf(f, "%lf %lf", &sx, &sy) != 2)
            goto fail;
        sc->meas = MEAS_RANGE_BEARING;
        sc->sensor[0] = (kf_real)sx;
        sc->sensor[1] = (kf_real)sy;
    } else {
        fprintf(stderr, "scenario: unknown measurement model '%s'\n", tok);
        goto fail;
    }

    const int n = sc->n, m = sc->m;
    if (read_values(f, sc->F, n * n, "F", "F") || read_values(f, sc->H, m * n, "H", "H") ||
        read_values(f, sc->Q, n * n, "Q", "Q") || read_values(f, sc->R, m * m, "R", "R") ||
        read_values(f, sc->x0, n, "x0", "x0") || read_values(f, sc->P0, n * n, "P0", "P0")) {
        goto fail;
    }

    if (fscanf(f, " data %63s", tok) != 1)
        goto fail;
    if (strcmp(tok, "zero") == 0) {
        fclose(f);
        return 0;
    }
    if (strcmp(tok, "rows") != 0)
        goto fail;

    const size_t rows = (size_t)sc->trials * (size_t)sc->steps;
    sc->truth = malloc(rows * (size_t)n * sizeof *sc->truth);
    sc->z = malloc(rows * (size_t)m * sizeof *sc->z);
    if (sc->truth == NULL || sc->z == NULL)
        goto fail;

    for (size_t r = 0; r < rows; ++r) {
        int trial, k;
        double v;
        if (fscanf(f, "%d %d", &trial, &k) != 2 || (size_t)trial * sc->steps + k != r) {
            fprintf(stderr, "scenario: bad data row %zu in %s\n", r, path);
            goto fail;
        }
        for (int i = 0; i < n; ++i) {
            if (fscanf(f, "%lf", &v) != 1)
                goto fail;
            sc->truth[r * n + i] = (kf_real)v;
        }
        for (int i = 0; i < m; ++i) {
            if (fscanf(f, "%lf", &v) != 1)
                goto fail;
            sc->z[r * m + i] = (kf_real)v;
        }
    }
    fclose(f);
    scenario_convert_fx(sc);
    return 0;

fail:
    fprintf(stderr, "scenario: failed to parse %s\n", path);
    fclose(f);
    scenario_free(sc);
    return -1;
}

void scenario_free(scenario *sc) {
    free(sc->truth);
    free(sc->z);
    free(sc->fx_z);
    sc->truth = NULL;
    sc->z = NULL;
    sc->fx_z = NULL;
}

const kf_real *scenario_truth(const scenario *sc, int trial, int k) {
    if (sc->truth == NULL)
        return zero_row;
    return &sc->truth[((size_t)trial * sc->steps + k) * sc->n];
}

const kf_real *scenario_z(const scenario *sc, int trial, int k) {
    if (sc->z == NULL)
        return zero_row;
    return &sc->z[((size_t)trial * sc->steps + k) * sc->m];
}

void scenario_range_bearing(const scenario *sc, const kf_real *x, kf_real *z, kf_real *H) {
    const double dx = (double)x[0] - (double)sc->sensor[0];
    const double dy = (double)x[1] - (double)sc->sensor[1];
    const double r2 = dx * dx + dy * dy, r = sqrt(r2);
    z[0] = (kf_real)r;
    z[1] = (kf_real)atan2(dy, dx);
    if (H != NULL) {
        memset(H, 0, 2 * (size_t)sc->n * sizeof *H);
        H[0] = (kf_real)(dx / r);
        H[1] = (kf_real)(dy / r);
        H[sc->n + 0] = (kf_real)(-dy / r2);
        H[sc->n + 1] = (kf_real)(dx / r2);
    }
}

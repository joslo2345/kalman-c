/*
 * Desktop benchmark harness (guide Step 8). Appends CSV rows with the schema
 *   library,library_version,scenario,filter,precision,metric,value,unit,commit,cpu,os,toolchain,date
 *
 * Usage: bench "<commit>,<cpu>,<os>,<toolchain>,<date>" [vectors-dir]
 *
 *   S1, S2, S5  KF   time_per_step (median of 30 timed runs after a warm-up), rmse,
 *                    and max_abs_diff of each baseline from kalman-c
 *               SRKF the same for kalman-c's square-root (UD) filter
 *   S3          EKF/UKF  rmse and nees, averaged over all trials
 *   S4          KF   steps_to_failure (equal to the scenario length if it never failed)
 *
 * A library that doesn't support a scenario or filter writes no row; the
 * table shows it as n/a. A build with KF_SPECIALIZE reports as kalman-c-specialized.
 */
#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "runners/runners.h"
#include "scenario.h"

#define REPEATS 30

typedef struct {
    const char *name;
    const char *version;
    const char *filter;
    runner_fn run;
} library;

static const char *env = "unknown,unknown,unknown,unknown,unknown";

static double now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e9 + (double)t.tv_nsec;
}

static int cmp_double(const void *a, const void *b) {
    const double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void row(const library *lib, const scenario *sc, const char *metric, double value,
                const char *unit) {
    printf("%s,%s,%s,%s,%s,%s,%.6g,%s,%s\n", lib->name, lib->version, sc->id, lib->filter,
           KF_PRECISION_NAME, metric, value, unit, env);
}

/* Median nanoseconds per step over REPEATS timed runs, after one warm-up. */
static double time_per_step(const library *lib, const scenario *sc) {
    double times[REPEATS];
    run_result res = {0};
    const double steps = (double)sc->steps * sc->trials;

    if (lib->run(sc, &res) != RUN_OK)
        return NAN;
    for (int r = 0; r < REPEATS; ++r) {
        const double t0 = now_ns();
        if (lib->run(sc, &res) != RUN_OK)
            return NAN;
        times[r] = (now_ns() - t0) / steps;
    }
    qsort(times, REPEATS, sizeof *times, cmp_double);
    return times[REPEATS / 2];
}

static void bench_linear(const scenario *sc, const library *libs, int nlibs) {
    const size_t len = (size_t)sc->trials * sc->steps * sc->n;
    kf_real *ours = malloc(len * sizeof *ours);
    kf_real *theirs = malloc(len * sizeof *theirs);
    if (ours == NULL || theirs == NULL) {
        fprintf(stderr, "bench: out of memory\n");
        exit(1);
    }

    for (int l = 0; l < nlibs; ++l) {
        run_result res = {0};
        res.measure = 1;
        res.estimates = l == 0 ? ours : theirs;
        const int rc = libs[l].run(sc, &res);
        if (rc == RUN_FAILED) {
            fprintf(stderr, "bench: %s %s could not run %s\n", libs[l].name, libs[l].filter,
                    sc->id);
        }
        if (rc != RUN_OK)
            continue;
        if (res.steps_to_failure >= 0) {
            fprintf(stderr, "bench: %s failed on %s at step %ld; skipping\n", libs[l].name, sc->id,
                    res.steps_to_failure);
            continue;
        }
        row(&libs[l], sc, "rmse", res.rmse, "state");
        if (l > 0) {
            double max_diff = 0;
            for (size_t i = 0; i < len; ++i) {
                const double d = fabs((double)ours[i] - (double)theirs[i]);
                if (d > max_diff)
                    max_diff = d;
            }
            row(&libs[l], sc, "max_abs_diff", max_diff, "state");
        }
        const double t = time_per_step(&libs[l], sc);
        if (!isnan(t))
            row(&libs[l], sc, "time_per_step", t, "ns");
        fflush(stdout);
    }
    free(ours);
    free(theirs);
}

static void bench_accuracy(const scenario *sc, const library *libs, int nlibs) {
    for (int l = 0; l < nlibs; ++l) {
        run_result res = {0};
        res.measure = 1;
        if (libs[l].run(sc, &res) != RUN_OK)
            continue;
        if (res.steps_to_failure >= 0) {
            fprintf(stderr, "bench: %s %s failed on %s at step %ld; skipping\n", libs[l].name,
                    libs[l].filter, sc->id, res.steps_to_failure);
            continue;
        }
        row(&libs[l], sc, "rmse", res.rmse, "state");
        row(&libs[l], sc, "nees", res.nees, "-");
    }
}

static void bench_stability(const scenario *sc, const library *libs, int nlibs) {
    for (int l = 0; l < nlibs; ++l) {
        run_result res = {0};
        res.measure = 1;
        if (libs[l].run(sc, &res) != RUN_OK)
            continue;
        const long steps = res.steps_to_failure >= 0 ? res.steps_to_failure : (long)sc->steps;
        row(&libs[l], sc, "steps_to_failure", (double)steps, "steps");
    }
}

static int load(scenario *sc, const char *dir, const char *id) {
    fprintf(stderr, "bench: loading %s\n", id);
    return scenario_load(sc, dir, id);
}

int main(int argc, char **argv) {
    const char *dir = argc > 2 ? argv[2] : KF_VECTORS_DIR;
    const library linear_libs[] = {
        {KF_BENCH_NAME, KALMAN_C_VERSION, "KF", run_ours_kf},
        {"naive", "textbook", "KF", run_naive},
        {"tinyekf", TINYEKF_COMMIT, "KF", run_tinyekf},
        {KF_BENCH_NAME, KALMAN_C_VERSION, "SRKF", run_ours_sr},
    };
    const library nonlinear_libs[] = {
        {KF_BENCH_NAME, KALMAN_C_VERSION, "EKF", run_ours_ekf},
        {KF_BENCH_NAME, KALMAN_C_VERSION, "UKF", run_ours_ukf},
        {"tinyekf", TINYEKF_COMMIT, "EKF", run_tinyekf},
    };
    const int nlinear = (int)(sizeof linear_libs / sizeof *linear_libs);
    const int nnonlinear = (int)(sizeof nonlinear_libs / sizeof *nonlinear_libs);
    const char *linear_ids[] = {"S1", "S2", "S5"};
    scenario *sc = malloc(sizeof *sc);

    if (argc > 1)
        env = argv[1];
    if (sc == NULL)
        return 1;

    for (size_t s = 0; s < sizeof linear_ids / sizeof *linear_ids; ++s) {
        if (load(sc, dir, linear_ids[s]) != 0)
            return 1;
        bench_linear(sc, linear_libs, nlinear);
        scenario_free(sc);
    }

    if (load(sc, dir, "S3") != 0)
        return 1;
    bench_accuracy(sc, nonlinear_libs, nnonlinear);
    scenario_free(sc);

    if (load(sc, dir, "S4") != 0)
        return 1;
    bench_stability(sc, linear_libs, nlinear);
    scenario_free(sc);

    free(sc);
    return 0;
}

/*
 * Instruction-count regression probe. Runs one kalman-c runner on the first
 * STEPS steps of one scenario (one trial), inside regress_loop(), so a counter
 * restricted to that function (valgrind --tool=callgrind
 * --toggle-collect=regress_loop) measures only filter work: no file parsing
 * or process setup.
 *
 * Usage: regress <vectors-dir> <scenario> <runner> <steps>
 *   runner: kf, ekf, ukf, sr, fx
 *
 * Driven by scripts/check_regression.py.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "runners/runners.h"
#include "scenario.h"

#if defined(__GNUC__) || defined(__clang__)
#define REGRESS_NOINLINE __attribute__((noinline))
#else
#define REGRESS_NOINLINE
#endif

/* The only function the counter collects in. */
REGRESS_NOINLINE int regress_loop(const scenario *sc, runner_fn run, run_result *res);
REGRESS_NOINLINE int regress_loop(const scenario *sc, runner_fn run, run_result *res) {
    return run(sc, res);
}

int main(int argc, char **argv) {
    static const struct {
        const char *name;
        runner_fn run;
    } runners[] = {{"kf", run_ours_kf},
                   {"ekf", run_ours_ekf},
                   {"ukf", run_ours_ukf},
                   {"sr", run_ours_sr},
                   {"fx", run_ours_fx}};
    runner_fn run = NULL;
    run_result res;
    scenario *sc;

    if (argc != 5) {
        fprintf(stderr, "usage: %s <vectors-dir> <scenario> <runner> <steps>\n", argv[0]);
        return 2;
    }
    for (size_t i = 0; i < sizeof runners / sizeof *runners; ++i) {
        if (strcmp(argv[3], runners[i].name) == 0)
            run = runners[i].run;
    }
    const int steps = atoi(argv[4]);
    sc = malloc(sizeof *sc);
    if (run == NULL || steps < 1 || sc == NULL || scenario_load(sc, argv[1], argv[2]) != 0) {
        fprintf(stderr, "regress: bad arguments or scenario\n");
        return 2;
    }
    if (steps < sc->steps)
        sc->steps = steps;
    sc->trials = 1;

    memset(&res, 0, sizeof res); /* measure = 0: predict + update only */
    const int rc = regress_loop(sc, run, &res);
    printf("%s %s %s %d steps: %s\n", KF_PRECISION_NAME, argv[2], argv[3], sc->steps,
           rc == RUN_OK ? "ok" : "failed");
    scenario_free(sc);
    free(sc);
    return rc == RUN_OK ? 0 : 1;
}

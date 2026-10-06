#include "runners.h"

int run_tinyekf_2x1(const scenario *sc, run_result *out);
int run_tinyekf_4x2(const scenario *sc, run_result *out);
int run_tinyekf_15x6(const scenario *sc, run_result *out);

int run_tinyekf(const scenario *sc, run_result *out) {
    if (sc->n == 2 && sc->m == 1) return run_tinyekf_2x1(sc, out);
    if (sc->n == 4 && sc->m == 2) return run_tinyekf_4x2(sc, out);
    if (sc->n == 15 && sc->m == 6) return run_tinyekf_15x6(sc, out);
    return RUN_UNSUPPORTED;
}

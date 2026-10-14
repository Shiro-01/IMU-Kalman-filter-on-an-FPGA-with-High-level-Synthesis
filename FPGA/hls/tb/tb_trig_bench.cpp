// =============================================================================
// tb_trig_bench.cpp
// =============================================================================
#include "trig_bench.h"
#include <cstdio>
#include <cmath>

int main() {
    float x = 0.6f;   // safe domain for asin/acos (must be in [-1, 1])
    float y = 0.3f;

    float s, c, t, as, ac, at, a2, sh, ch, th, sq;
    trig_bench(x, y, s, c, t, as, ac, at, a2, sh, ch, th, sq);

    struct { const char *name; float v; } results[] = {
        {"sin", s}, {"cos", c}, {"tan", t}, {"asin", as}, {"acos", ac},
        {"atan", at}, {"atan2", a2}, {"sinh", sh}, {"cosh", ch},
        {"tanh", th}, {"sqrt", sq},
    };

    bool ok = true;
    for (auto &r : results) {
        bool finite = std::isfinite(r.v);
        printf("%-6s = %+.6f  %s\n", r.name, r.v, finite ? "OK" : "NOT FINITE");
        ok &= finite;
    }

    printf("\n%s\n", ok ? "ALL FINITE" : "SOME NON-FINITE RESULTS");
    return ok ? 0 : 1;
}
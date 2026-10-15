// tb_update.cpp
// Isolated testbench for update() ONLY. Calls update<T>() directly, not
// through ekf_update_top, so there's no dependency on the top-level wrapper
// or its interface pragmas — this is purely: same state in, same sample in,
// does the hardware core produce the same X/P as the Python reference.
//
// Generic over T: compile with -DUSE_FLOAT for a float run (isolates
// algorithm/logic bugs from fixed-point effects), or without it for the
// project's actual ap_fixed type (tests real hardware precision).
//
//   g++ run:      g++ -DUSE_FLOAT -std=c++17 tb_update.cpp -o tb_float && ./tb_float
//   Vitis csim:   add as testbench file, no -DUSE_FLOAT -> uses ap_fixed path
//
// Golden values from EKF.py, this session: x0=zeros(6),
// P0=diag([EULER_EST_SIGMA_SQUARE]*3+[1e-6]*3), synthetic tilt
// phi=0.10 theta=0.05 psi=0.20 fed back as a perfect (noiseless) measurement.
// This is a logic/index-correctness check, not a noisy-sensor accuracy check.

#ifdef USE_FLOAT
    typedef float T;
#else
    #include <ap_fixed.h>
    // NOTE: keep this in sync with ekf_fixed_t in top.h by hand — this file
    // intentionally does not include top.h, to stay independent of the
    // top-level wrapper/interfaces. If you change the type in top.h, mirror
    // it here too.
    //typedef ap_fixed<32, 14, AP_TRN, AP_WRAP> T;
    typedef float T;
#endif

#include "update.h"   // your actual project file — ekf_state_t<T>, preparer_out<T>, update<T>()
#include <cmath>
#include <cstdio>

static const double TOL_X = 1e-3;
static const double TOL_P = 1e-3;
static int fail_count = 0;

static void check(const char* label, double got, double expected, double tol) {
    double diff = std::fabs(got - expected);
    bool ok = diff <= tol;
    if (!ok) fail_count++;
    std::printf("%s  %-10s got=%.8f  expected=%.8f  diff=%.8f\n",
                ok ? "pass" : "FAIL", label, got, expected, diff);
}

int main() {
    // ---- inputs ----
    const double dt = 0.00088888888888888893;
    const double ax = -0.49029565054535446;
    const double ay =  0.97814186505568157;
    const double az =  9.7487921645135973;
    const double gx =  0.001;
    const double gy = -0.0005;
    const double gz =  0.0002;
    const double mx = 15.854681755586066;
    const double my = 1.000367853831138;
    const double mz = 46.86506514010312;

    // ---- initial state ----
    const double EULER_EST_SIGMA_SQUARE = 0.017453292519943295 * 0.017453292519943295;
    const double p0_diag[6] = {
        EULER_EST_SIGMA_SQUARE, EULER_EST_SIGMA_SQUARE, EULER_EST_SIGMA_SQUARE,
        1e-6, 1e-6, 1e-6
    };

    // ---- expected outputs (Python golden, float64) ----
    const double X_expected[6] = {
        0.08679101422519461, 0.04439579259596591, 0.00410234862822108,
        0.0, 0.0, 0.0
    };
    const double P_expected[6][6] = {
        {0.00003517, 0.0,         0.00000222, 0.0, 0.0, 0.0},
        {0.0,        0.00003505,  0.0,        0.0, 0.0, 0.0},
        {0.00000222, 0.0,         0.00029699, 0.0, 0.0, 0.0},
        {0.0,        0.0,         0.0,        0.000001, 0.0, 0.0},
        {0.0,        0.0,         0.0,        0.0, 0.000001, 0.0},
        {0.0,        0.0,         0.0,        0.0, 0.0, 0.000001},
    };

    // ---- build DUT inputs ----
    ekf_state_t<T> state;
    for (int i = 0; i < 6; i++) state.x[i] = (T)0.0;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            state.p[i][j] = (i == j) ? (T)p0_diag[i] : (T)0.0;

    preparer_out<T> sample;
    sample.dt = (T)dt;
    sample.prepared_data[0] = (T)ax; sample.prepared_data[1] = (T)ay; sample.prepared_data[2] = (T)az;
    sample.prepared_data[3] = (T)gx; sample.prepared_data[4] = (T)gy; sample.prepared_data[5] = (T)gz;
    sample.prepared_data[6] = (T)mx; sample.prepared_data[7] = (T)my; sample.prepared_data[8] = (T)mz;
    sample.mag_fresh = true;

    // ---- call update() directly, isolated, no top wrapper ----
    update<T>(state, sample);

    // ---- checks ----
    std::printf("=== State vector ===\n");
    const char* xlabels[6] = {"phi", "theta", "psi", "bx", "by", "bz"};
    for (int i = 0; i < 6; i++)
        check(xlabels[i], (double)state.x[i], X_expected[i], TOL_X);

    std::printf("\n=== Covariance (P[i][j] with |expected|>0 or |got|>tol) ===\n");
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            char label[16];
            std::snprintf(label, sizeof(label), "P[%d][%d]", i, j);
            double got = (double)state.p[i][j];
            if (std::fabs(P_expected[i][j]) > 1e-9 || std::fabs(got) > TOL_P)
                check(label, got, P_expected[i][j], TOL_P);
        }
    }

    std::printf("\n%d check(s) failed.\n", fail_count);
    return fail_count == 0 ? 0 : 1;
}
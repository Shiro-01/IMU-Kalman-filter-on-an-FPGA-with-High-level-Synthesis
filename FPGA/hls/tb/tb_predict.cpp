// =========================================================================
// tb_predict.cpp -- compares predict()'s C-simulation output against the
// golden vector generated from the Python EKF reference model
// (ekf_predict_golden.py / golden_predict.txt).
//
// Layout assumption for preparer_out<T>::prepared_data, matching how
// update() already reads it (prepared_data[0..2]=accel, [6..8]=mag):
//   [0..2] = ax, ay, az
//   [3..5] = gx, gy, gz
//   [6..8] = mx, my, mz
// Adjust the indices below if your real struct differs.
// =========================================================================
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "predict.h"

typedef float T;

int main() {
    ekf_state_t<T> ekf_state;
    ekf_state.x[0] = 0.1f;
    ekf_state.x[1] = 0.2f;
    ekf_state.x[2] = 0.3f;
    ekf_state.x[3] = 0.001f;
    ekf_state.x[4] = -0.002f;
    ekf_state.x[5] = 0.0015f;

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            ekf_state.p[i][j] = (i == j) ? 0.01f : 0.0f;

    preparer_out<T> prepared_sample;
    prepared_sample.dt = 0.0008888888888888889f;   // AG_FS_PERIOD
    prepared_sample.prepared_data[0] = 0.0f;        // ax (unused by predict)
    prepared_sample.prepared_data[1] = 0.0f;        // ay
    prepared_sample.prepared_data[2] = -9.81f;       // az
    prepared_sample.prepared_data[3] = 0.05f;        // gx
    prepared_sample.prepared_data[4] = -0.03f;       // gy
    prepared_sample.prepared_data[5] = 0.02f;        // gz
    prepared_sample.prepared_data[6] = 18.0f;        // mx (unused by predict)
    prepared_sample.prepared_data[7] = 0.0f;         // my
    prepared_sample.prepared_data[8] = 46.0f;        // mz

    bool gimbal_lock_out = false;

    predict<T>(ekf_state, prepared_sample, gimbal_lock_out);

    // ---- golden X ----
    const double X_golden[6] = {
        0.10004636867378387, 0.19997359374681267, 0.30001415980119855,
        0.001, -0.002, 0.0015
    };

    // ---- golden P (row-major) ----
    const double P_golden[6][6] = {
        { 9.9999011887e-03,  5.7031843630e-09, -2.6779409508e-07, -8.8888888889e-06, -1.7988653719e-07, -1.7928651527e-06},
        { 5.7031843630e-09,  1.0000007916e-02,  2.8706918880e-08,  0.0000000000e+00, -8.8444814691e-06,  8.8740814797e-07},
        {-2.6779409508e-07,  2.8706918880e-08,  1.0000008249e-02,  0.0000000000e+00, -9.0545700469e-07, -9.0243680063e-06},
        {-8.8888888889e-06,  0.0000000000e+00,  0.0000000000e+00,  1.0000000002e-02,  0.0000000000e+00,  0.0000000000e+00},
        {-1.7988653719e-07, -8.8444814691e-06, -9.0545700469e-07,  0.0000000000e+00,  1.0000000001e-02,  0.0000000000e+00},
        {-1.7928651527e-06,  8.8740814797e-07, -9.0243680063e-06,  0.0000000000e+00,  0.0000000000e+00,  1.0000000002e-02}
    };

    const double TOL = 1e-5;   // float precision -- loosen/tighten vs. what you actually need
    int fail_count = 0;
    double max_diff = 0.0;

    printf("=== State vector (X) comparison ===\n");
    for (int i = 0; i < 6; i++) {
        double dut = (double)ekf_state.x[i];
        double diff = std::fabs(dut - X_golden[i]);
        max_diff = std::max(max_diff, diff);
        bool ok = diff < TOL;
        if (!ok) fail_count++;
        printf("X[%d]: dut=%.10f golden=%.10f diff=%.3e %s\n",
               i, dut, X_golden[i], diff, ok ? "OK" : "FAIL");
    }

    printf("\n=== Covariance matrix (P) comparison ===\n");
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double dut = (double)ekf_state.p[i][j];
            double diff = std::fabs(dut - P_golden[i][j]);
            max_diff = std::max(max_diff, diff);
            bool ok = diff < TOL;
            if (!ok) fail_count++;
            printf("P[%d][%d]: dut=%.6e golden=%.6e diff=%.3e %s\n",
                   i, j, dut, P_golden[i][j], diff, ok ? "OK" : "FAIL");
        }
    }

    printf("\nMax abs diff: %.3e\n", max_diff);
    printf("gimbal_lock_out: %d (expected 0)\n", gimbal_lock_out);

    if (fail_count == 0) {
        printf("\n*** TEST PASSED ***\n");
        return 0;
    } else {
        printf("\n*** TEST FAILED: %d mismatches ***\n", fail_count);
        return 1;
    }
}
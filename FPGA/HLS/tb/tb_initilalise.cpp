// =============================================================================
// tb_initilalise.cpp
//
// =============================================================================
#include "preparer.h"
#include <cstdio>
#include <cmath>

static bool nearly_equal(float a, float b, float tol = 1e-3f) {
    return std::fabs(a - b) <= tol * std::fabs(b) + tol;
}

int main() {
    converter_out_t s;
    s.dt = 0.0f;  // unused by initilalise()

    // -- accel/gyro: constant across all 100 samples --
    s.raw_accel[0] = 0;    s.raw_accel[1] = 0;    s.raw_accel[2] = 2048;
    s.raw_gyro[0]  = 10;   s.raw_gyro[1]  = -5;   s.raw_gyro[2]  = 3;
    s.accel[0] = 0.0f * ACCEL_SCALE[0];
    s.accel[1] = 0.0f * ACCEL_SCALE[1];
    s.accel[2] = 2048.0f * ACCEL_SCALE[2];
    s.gyro[0]  = 10.0f * GYRO_SCALE[0];
    s.gyro[1]  = -5.0f * GYRO_SCALE[1];
    s.gyro[2]  = 3.0f  * GYRO_SCALE[2];

    // -- mag: 4 distinct segments of 25 samples each (100 total) --
    int16_t mag_segments_raw[4][3] = {
        {120, -10, 300},
        {125,  -5, 310},
        {118,   0, 295},
        {122,   5, 305},
    };

    ekf_init_t init;
    init.x0[0] = 0; 
    bool done = false;

    for (int seg = 0; seg < 4; seg++) {
        int16_t rmx = mag_segments_raw[seg][0];
        int16_t rmy = mag_segments_raw[seg][1];
        int16_t rmz = mag_segments_raw[seg][2];
        s.raw_mag[0] = rmx; s.raw_mag[1] = rmy; s.raw_mag[2] = rmz;
        
        s.mag[0] = (float)rmx * MAG_SCALE[0];
        s.mag[1] = (float)rmy * MAG_SCALE[1];
        s.mag[2] = (float)rmz * MAG_SCALE[2];

        for (int i = 0; i < 25; i++) {
            initialize(s, init, done);
        }
    }

    if (!done) {
        printf("FAIL: done_flag not set after 100 samples\n");
        return 1;
    }

    // -- expected values (computed independently in Python) --
    float exp_x0[6] = {
        0.0000000000f,   // phi0
        0.0000000000f,   // theta0
        -0.4565611466f,  // psi0
        0.0106422515f,   // gx_mean (bias x)
        0.0053211258f,   // gy_mean (bias y)
        -0.0031926755f,  // gz_mean (bias z)
    };
    float exp_p0[6] = {
        3.0461741979e-04f, 3.0461741979e-04f, 3.0461741979e-04f,
        2.0277455653e-07f, 1.6837409524e-07f, 1.8813247300e-07f,
    };
    const char *names[6] = {"phi0", "theta0", "psi0", "bx", "by", "bz"};

    bool ok = true;
    printf("=== x0 ===\n");
    for (int i = 0; i < 6; i++) {
        bool this_ok = nearly_equal(init.x0[i], exp_x0[i]);
        printf("  %-6s got=%+.8f expected=%+.8f  %s\n",
               names[i], init.x0[i], exp_x0[i], this_ok ? "OK" : "MISMATCH");
        ok &= this_ok;
    }

    printf("=== p0_diag ===\n");
    for (int i = 0; i < 6; i++) {
        bool this_ok = nearly_equal(init.p0_diag[i], exp_p0[i]);
        printf("  p0[%d]  got=%+.10e expected=%+.10e  %s\n",
               i, init.p0_diag[i], exp_p0[i], this_ok ? "OK" : "MISMATCH");
        ok &= this_ok;
    }

    printf("\n%s\n", ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return ok ? 0 : 1;
}
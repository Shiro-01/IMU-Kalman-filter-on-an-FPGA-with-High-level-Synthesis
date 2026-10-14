// =============================================================================
// tb_convert.cpp
//
// Pushes TWO 15-word frames through convert() and checks the results.
// scale constants in constants.h.
// =============================================================================
#include "converter.h"
#include <cstdio>
#include <cmath>
#include "constants.h"

static bool nearly_equal(float a, float b, float tol = 1e-4f) {
    return std::fabs(a - b) <= tol * std::fabs(b) + tol;
}

// Pushes one 15-word MODE_0 frame. ts is the raw 64-bit tick count for
// this frame; sensor_raw is the 10 signed sensor words in frame order
// (ax,ay,az,gx,gy,gz,temp,mx,my,mz); status is the raw status word.
static void push_frame(my_stream &A, uint64_t ts, const int16_t sensor_raw[10],
                        uint16_t status) {
    // 4 timestamp words, MSB first
    for (int shift = 48; shift >= 0; shift -= 16) {
        data_t w;
        w.data = (uint16_t)((ts >> shift) & 0xFFFF);
        w.last = 0;
        A.write(w);
    }
    // 10 signed sensor words
    for (int i = 0; i < 10; i++) {
        data_t w;
        w.data = sensor_raw[i];
        w.last = 0;
        A.write(w);
    }
    // 1 status word, tlast set here
    data_t w;
    w.data = (int16_t)status;
    w.last = 1;
    A.write(w);
}

// exp_conv is the unified 9-element expected array: 0-2 accel, 3-5 gyro, 6-8 mag
static bool check_sample(const converter_out_t &s, float exp_dt,
                          const float exp_conv[9], bool exp_mag_fresh) {
    bool ok = true;
    printf("  dt: got=%.8f expected=%.8f  %s\n", s.dt, exp_dt,
           nearly_equal(s.dt, exp_dt) ? "OK" : "MISMATCH");
    if (!nearly_equal(s.dt, exp_dt)) ok = false;

    const char *field_names[9] = {
        "accel_x", "accel_y", "accel_z",
        "gyro_x",  "gyro_y",  "gyro_z",
        "mag_x",   "mag_y",   "mag_z"
    };
    for (int i = 0; i < 9; i++) {
        bool c_ok = nearly_equal(s.converted_data[i], exp_conv[i]);
        printf("  %s: got=%+.6f expected=%+.6f  %s\n", field_names[i],
               s.converted_data[i], exp_conv[i], c_ok ? "OK" : "MISMATCH");
        ok &= c_ok;
    }

    printf("  mag_fresh: got=%d expected=%d  %s\n", (int)s.mag_fresh,
           (int)exp_mag_fresh, (s.mag_fresh == exp_mag_fresh) ? "OK" : "MISMATCH");
    if (s.mag_fresh != exp_mag_fresh) ok = false;

    return ok;
}

int main() {
    my_stream A;

    // Same raw sensor counts for both frames -- only the timestamp changes,
    // so both frames' converted_data checks share the same expected values.
    int16_t sensor_raw[10] = {
        2048, 1024, -2048,     // ax, ay, az
        164, -164, 328,         // gx, gy, gz
        1000,                    // temp (converted but not stored -- struct has no field for it)
        100, 200, -100,          // mx, my, mz
    };

    // Frame 1: ts = 100000 ticks. last_sample_ts starts at 0 (static init),
    // so dt here is 100000/1e8 -- see file header note.
    push_frame(A, 100000, sensor_raw, 0x0001);
    // Frame 2: ts = 188889 ticks -> delta of 88889 ticks (~1 sample period
    // at 1125Hz/100MHz) -- this is the dt value that actually means something.
    push_frame(A, 188889, sensor_raw, 0x0001);

    // unified 9-element expected array: 0-2 accel, 3-5 gyro, 6-8 mag
    float exp_conv[9] = {
        -9.81f, 4.905f, -9.81f,                          // accel
        0.17453293f, 0.17453293f, -0.34906585f,           // gyro
        15.0f, 30.0f, -15.0f,                              // mag
    };

    printf("=== Frame 1 ===\n");
    converter_out_t sample1;
    convert(A, sample1);
    bool ok1 = check_sample(sample1, 100000.0f / TIMESTAMP_CLK_FREQ,
                              exp_conv, true);

    printf("\n=== Frame 2 ===\n");
    converter_out_t sample2;
    convert(A, sample2);
    bool ok2 = check_sample(sample2, 88889.0f / TIMESTAMP_CLK_FREQ,
                              exp_conv, true);

    bool all_ok = ok1 && ok2;
    printf("\n%s\n", all_ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return all_ok ? 0 : 1;
}
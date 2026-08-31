/**
 * @file    constants.h
 * @brief   All the constants used to run the filter gathered in one place
 *          For tuning or changing smth in the filter simply change the constant below
 *
 * Kept in a separate header so that the FreeRTOS logger task and every
 * IMU driver can share a single definition of IMUData without creating
 * circular dependencies.
 * 
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

/** ensor sensitivities (ICM-20948, AK09916) */
static const float ACC_LSB_PER_1_G  = 2048.0;                                        // LSBs/g.   - ±16g
static const float GYRO_LSB_PER_DPS = 16.4;                                          // LSBs/(dps) - ±2000dps
static const float MAG_LSB          = 0.15;                                          // µT/LSB, fixed per AK09916 spec
static const float TEMP_LSB_PER_C =  333.87;
static const float TEMP_OFFSET = 21.0;
static const float TIMESTAMP_CLK_FREQ = 100000000;                                   // # 100_000_000in Hz

/*--------------------------------------------------------------------------*/
/** physical constants*/
/*--------------------------------------------------------------------------*/
static const float PI_F     = 3.14159265358979323846f;
static const float DEG2RAD  = 0.017453292519943295f;               // pi / 180
static const float RAD2DEG  = 57.29577951308232f;
static const float GRAVITY = 9.81f;                                // m/s^2
static const float MAG_N_UT = 18.54f;                              // horizontal component (mN)
static const float MAG_D_UT = 45.88f;                              // vertical component (mD)


/*--------------------------------------------------------------------------*/
/** Noise Charactristics and uncertainities*/
/*--------------------------------------------------------------------------*/
static const float GYRO_ARW[3] = {0.000134255, 0.000122338, 0.000129317};
static const float GYRO_BI[3] = {4.28161e-5, 4.02741e-05, 4.48088e-05};
static const float ACCEL_DEV[3] = {0.0625, 0.0625, 0.0625};
static const float MAG_DEV[3] = {2, 2, 2};


/*--------------------------------------------------------------------------*/
/** Sensor offsets*/
/*--------------------------------------------------------------------------*/
static const float HARD_IRON_OFFSET[3] = {-31.741723, -24.290997, 22.052194};                 // uT
static const float SOFT_IRON_MATRIX[3][3] = {{ 1.176901,  0.015028,  0.004831},
                    { 0.015028,  1.285583, -0.103001},
                    { 0.004831, -0.103001,  1.311687},
                    };
static const float ACCEL_BIAS_OFFSET[3] = {0.0f, 0.0f, 0.0f};

/*--------------------------------------------------------------------------*/
/** Axis sign correction (validated against physical mounting) */
/*--------------------------------------------------------------------------*/
static const float ACCEL_SIGN[3] = {-1.0f, 1.0f, 1.0f};   // ax negated, ay/az not
static const float GYRO_SIGN[3]  = { 1.0f,-1.0f,-1.0f};   // gy/gz negated, gx not
static const float MAG_SIGN[3]  = { 1.0f, 1.0f, 1.0f};   // gy/gz negated, gx not


/*--------------------------------------------------------------------------*/
/** Stream propetises*/
/*--------------------------------------------------------------------------*/
static const int READ_STREAM_SIZE = 15;         //in  words

#define AXIS_ENABLE_LAST 0b00010000
#define AXIS_ENABLE_DATA 0b00000001

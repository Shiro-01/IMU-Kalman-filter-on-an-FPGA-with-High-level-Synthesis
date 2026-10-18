#ifndef CONSTANTS_H
#define CONSTANTS_H

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
static const float TIMESTAMP_CLK_PERIOD = 1.0/100000000;  
/*--------------------------------------------------------------------------*/
/** physical constants*/
/*--------------------------------------------------------------------------*/
static const float PI_F     = 3.14159265358979323846f;
static const float DEG2RAD  = 0.017453292519943295f;               // pi / 180
static const float RAD2DEG  = 57.29577951308232f;
static const float GRAVITY = 9.81f;                                // m/s^2
static const float MAG_N_UT = 18.225f;                              // horizontal component (mN)
static const float MAG_D_UT = 46.710f;                              // vertical component (mD)


/*--------------------------------------------------------------------------*/
/** Noise Charactristics and uncertainities*/
/*--------------------------------------------------------------------------*/
static const float GYRO_ARW[3] = {0.0001313f, 0.0001181f, 0.0001266f};               // rad/sqrt(s)
static const float GYRO_BI[3] = {6.98e-5f, 5.89e-5f, 6.38e-5f};                      // rad/s
static const float ACCEL_DEV[3] = {0.1f, 0.1f, 0.1f};                                // m/s^2
static const float MAG_DEV[3] = {2.0f, 2.0f, 2.0f};                                  // uT


//Used paramters in the hardware
static const float ACCEL_VAR[3] = {ACCEL_DEV[0] * ACCEL_DEV[0], ACCEL_DEV[1] * ACCEL_DEV[1], ACCEL_DEV[2] * ACCEL_DEV[2]};
static const float MAG_VAR[3] = {MAG_DEV[0] * MAG_DEV[0], MAG_DEV[1] * MAG_DEV[1], MAG_DEV[2] * MAG_DEV[2]};
static const float GYRO_ARW_SQUARED[3] = {GYRO_ARW[0] * GYRO_ARW[0], GYRO_ARW[1] * GYRO_ARW[1], GYRO_ARW[2] * GYRO_ARW[2]};
static const float GYRO_BI_SQUARED[3] = {GYRO_BI[0]*GYRO_BI[0], GYRO_BI[1]*GYRO_BI[1], GYRO_BI[2]*GYRO_BI[2]};
/*--------------------------------------------------------------------------*/
/** Sensor offsets*/
/*--------------------------------------------------------------------------*/
static const float HARD_IRON_OFFSET[3] = {-42.27960395312267f, -32.05143380329302f, 32.445942863812704f};   // uT
static const float SOFT_IRON_MATRIX[3][3] = {
    { 1.0923425642852367f,   0.005731459604421616f,   0.010700866604354578f},
    { 0.005731459604421628f, 0.9971976191544878f,    -0.00013714721151409658f},
    { 0.01070086660435455f, -0.00013714721151402752f, 1.1317227852427314f}};
static const float ACCEL_BIAS_OFFSET[3] = {-0.284029f, -0.0266191f, 0.131762f};

/*--------------------------------------------------------------------------*/
/** Axis sign correction (validated against physical mounting) */
/*--------------------------------------------------------------------------*/
static const float ACCEL_SIGN[3] = {-1.0f, 1.0f, 1.0f};   // ax negated, ay/az not
static const float GYRO_SIGN[3]  = { 1.0f,-1.0f,-1.0f};   // gy/gz negated, gx not
static const float MAG_SIGN[3]  = { 1.0f, 1.0f, 1.0f};   // gy/gz negated, gx not


static const float ACCEL_SCALE[3] =  { ACCEL_SIGN[0] * GRAVITY / ACC_LSB_PER_1_G ,  ACCEL_SIGN[1] * GRAVITY / ACC_LSB_PER_1_G,  ACCEL_SIGN[2] * GRAVITY / ACC_LSB_PER_1_G};
static const float GYRO_SCALE[3] =  { GYRO_SIGN[0] * DEG2RAD / GYRO_LSB_PER_DPS ,  GYRO_SIGN[1] * DEG2RAD / GYRO_LSB_PER_DPS,  GYRO_SIGN[2] * DEG2RAD / GYRO_LSB_PER_DPS};
static const float MAG_SCALE[3] =  { MAG_SIGN[0] * MAG_LSB ,  MAG_SIGN[1] * MAG_LSB,  MAG_SIGN[2] * MAG_LSB};
static const float SCALE_ALL[9] = {
    ACCEL_SCALE[0], ACCEL_SCALE[1], ACCEL_SCALE[2],
    GYRO_SCALE[0],  GYRO_SCALE[1],  GYRO_SCALE[2],
    MAG_SCALE[0],   MAG_SCALE[1],   MAG_SCALE[2]
};
/*--------------------------------------------------------------------------*/
/** Frequancies */
/*--------------------------------------------------------------------------*/
static const int MAG_FS = 100;
static const float MAG_FS_PERIOD = 0.1;

static const int AG_FS = 1125;
static const float AG_FS_PERIOD = 1.0/AG_FS;
/*--------------------------------------------------------------------------*/
/** Modules Properties */
/*--------------------------------------------------------------------------*/
/** Stream propetises*/
static const int READ_STREAM_SIZE = 15;             //in  words

/** Preparer Properties */
static const int INIT_AVG_WINDOW_SIZE = 100;         //  Averaging window size
static const float INIT_AVG_WINDOW_SIZE_INV = 1.0/ ((float)INIT_AVG_WINDOW_SIZE);        //  Averaging window size
//static const float INIT_AVG_MAG_WINDOW_SIZE_INV = (float)AG_FS / ((float)MAG_FS * (float)INIT_AVG_WINDOW_SIZE); // = 1125.0f / (100.0f * 100) = 0.1125 not used replaced by a counter

/** Initial state uncertaintiy */
static const float EULER_EST_SIGMA_SQUARE = DEG2RAD * DEG2RAD;
static const float GIMBAL_LOCK_COS_THRESHOLD = 0.017452406f;  // cos(89 degrees)


// calculating bias sigma from GYRO_ARW
static const float BIAS_VAR_X = GYRO_ARW_SQUARED[0] * (INIT_AVG_WINDOW_SIZE_INV * AG_FS); 
static const float BIAS_VAR_Y = GYRO_ARW_SQUARED[1] * (INIT_AVG_WINDOW_SIZE_INV * AG_FS); 
static const float BIAS_VAR_Z = GYRO_ARW_SQUARED[2] * (INIT_AVG_WINDOW_SIZE_INV * AG_FS); 
#endif // CONSTANTS_H
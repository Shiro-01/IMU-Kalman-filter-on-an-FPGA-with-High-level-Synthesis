/**
 * @file    IMUTypes.h
 * @brief   Shared data types used across all IMU drivers.
 *
 * Kept in a separate header so that the FreeRTOS logger task and every
 * IMU driver can share a single definition of IMUData without creating
 * circular dependencies.
 */

#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

/**
 * @brief Raw 16-bit output from any supported IMU.
 *
 * Layout:
 * @code
 *   data[0] = Accel X    data[1] = Accel Y    data[2] = Accel Z
 *   data[3] = Temp
 *   data[4] = Gyro X     data[5] = Gyro Y     data[6] = Gyro Z
 * @endcode
 */
struct __attribute__((packed)) IMUData {
    int16_t data[7];
};


/**
 * @brief Raw byte stream from a single burst SPI read spanning
 *        ACCEL_XOUT_H (0x2D) through EXT_SLV_SENS_DATA_08 (0x43).
 *
 * All values are big-endian on the wire (ICM20948 native byte order) —
 * no swap is performed here; endianness conversion and int16_t
 * reassembly are left to the PC-side parser over UART.
 *
 * Layout:
 * @code
 *   data[0]  = ACCEL_XOUT_H       data[1]  = ACCEL_XOUT_L
 *   data[2]  = ACCEL_YOUT_H       data[3]  = ACCEL_YOUT_L
 *   data[4]  = ACCEL_ZOUT_H       data[5]  = ACCEL_ZOUT_L
 *
 *   data[6]  = GYRO_XOUT_H        data[7]  = GYRO_XOUT_L
 *   data[8]  = GYRO_YOUT_H        data[9]  = GYRO_YOUT_L
 *   data[10] = GYRO_ZOUT_H        data[11] = GYRO_ZOUT_L
 *
 *   data[12] = TEMP_OUT_H         data[13] = TEMP_OUT_L
 *
 *   data[14] = EXT_SLV_SENS_DATA_00  (AK09916 ST1  — bit0 = DRDY, bit1 = DOR)
 *   data[15] = EXT_SLV_SENS_DATA_01  (AK09916 HXL)
 *   data[16] = EXT_SLV_SENS_DATA_02  (AK09916 HXH)
 *   data[17] = EXT_SLV_SENS_DATA_03  (AK09916 HYL)
 *   data[18] = EXT_SLV_SENS_DATA_04  (AK09916 HYH)
 *   data[19] = EXT_SLV_SENS_DATA_05  (AK09916 HZL)
 *   data[20] = EXT_SLV_SENS_DATA_06  (AK09916 HZH)
 *   data[21] = EXT_SLV_SENS_DATA_07  (reserved / TMPS, unused)
 *   data[22] = EXT_SLV_SENS_DATA_08  (AK09916 ST2  — bit3 = HOFL overflow)
 * @endcode
 */
struct __attribute__((packed)) IMUDataStream {
    uint8_t data[23];   // exact SPI burst, big-endian accel/gyro/temp, L-endian mag
};


/**
 * @brief raw counts, parsed and endianness fixed
 *
 */
struct IMUSample {
    int16_t accel[3];   // raw counts, native endianness resolved
    int16_t gyro[3];
    int16_t temp;
    int16_t mag[3];

    bool magValid;       // ST1 DRDY
    bool magOverrun;      // ST1 DOR
    bool magOverflow;    // ST2 HOFL
};



/**
 * @brief Sensors readings translated into physical quantities
 *
 */
struct IMUPhysical {
    float accel[3];   // m/s^2
    float gyro[3];    // rad/s
    float temp;        // degC
    float mag[3];       // µT
    bool  magValid;
};


/**
 * @brief Timing statistics from a benchmark run.
 */
struct BenchResult {
    uint32_t minTime;   // microseconds
    uint32_t maxTime;   // microseconds
    float    avgTime;   // microseconds
    int      samples;
};


#endif // IMU_TYPES_H
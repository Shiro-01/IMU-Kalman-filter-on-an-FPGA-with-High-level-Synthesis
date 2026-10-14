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

#endif // IMU_TYPES_H
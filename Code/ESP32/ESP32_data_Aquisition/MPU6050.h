/**
 * @file    MPU6050.h
 * @brief   Driver interface for the MPU-6050 6-axis IMU (Accel + Gyro only).
 *
 * Configured for Allan Variance benchmarking:
 *   - Accel full-scale : ±2 g
 *   - Gyro  full-scale : ±250 dps
 *   - DLPF             : disabled (maximum bandwidth)
 *                        Accel BW = 260 Hz | Gyro BW = 256 Hz
 *   - Sample rate      : 1 kHz (SMPRT_DIV = 7, Gyro Fs = 8 kHz → 8/8 = 1 kHz)
 *
 * @note    With DLPF disabled, the gyroscope internal sample rate is 8 kHz
 *          and the accelerometer is 1 kHz. SMPRT_DIV divides the gyro rate,
 *          so DIV = 7 yields 1 kHz output for both.
 *
 * @note    Requires the Arduino Wire library. Call Wire.begin() before
 *          calling any function in this driver.
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include "IMUTypes.h"

/********************************* REGISTER MAP & MACROS **********************************/

/*************** Device Information ***************/
#define MPU6050_I2C_ADDR_LOW        0x68    ///< AD0 pin LOW  (default)
#define MPU6050_I2C_ADDR_HIGH       0x69    ///< AD0 pin HIGH

#define MPU6050_WHO_AM_I            0x75    ///< WHO_AM_I register address
#define MPU6050_WHO_AM_I_VAL        0x68    ///< Expected response

/*************** Configuration Registers ***************/
#define MPU6050_SMPRT_DIV           0x19    ///< Sample Rate Divider
#define MPU6050_CONFIG              0x1A    ///< General config (DLPF)
#define MPU6050_GYRO_CONFIG         0x1B    ///< Gyro full-scale range
#define MPU6050_ACCEL_CONFIG        0x1C    ///< Accel full-scale range
#define MPU6050_FIFO_EN             0x23    ///< FIFO enable
#define MPU6050_INT_ENABLE          0x38    ///< Interrupt enable
#define MPU6050_INT_STATUS          0x3A    ///< Interrupt status

/*************** Register Config Values ***************/
// Sample rate: Fs / (1 + DIV) = 8000 / (1 + 7) = 1000 Hz
#define MPU6050_SMPRT_DIV_VAL       0x07

// DLPF disabled → Gyro BW 256 Hz, Accel BW 260 Hz
#define MPU6050_CONFIG_VAL          0x00

// Gyro full-scale ±250 dps → FS_SEL = 0b00 → bits [4:3] = 00
#define MPU6050_GYRO_CONFIG_VAL     0x00

// Accel full-scale ±2g → AFS_SEL = 0b00 → bits [4:3] = 00
#define MPU6050_ACCEL_CONFIG_VAL    0x00

/*************** Power Management ***************/
#define MPU6050_PWR_MGMT_1          0x6B    ///< Power management 1
#define MPU6050_PWR_MGMT_2          0x6C    ///< Power management 2

// PWR_MGMT_1 bit masks
#define MPU6050_RESET               0x80    ///< Bit 7 — full device reset
#define MPU6050_SLEEP_EN            0x40    ///< Bit 6 — sleep mode enable
#define MPU6050_CYCLE               0x20    ///< Bit 5 — cycle mode (periodic wake)
#define MPU6050_TEMP_DIS            0x08    ///< Bit 3 — disable temperature sensor
#define MPU6050_CLK_PLL_XGYRO       0x01    ///< Bits[2:0] — PLL with X-axis gyro ref 
                                            ///< Datasheet recommends any gyro PLL (0x01–0x03)
                                            ///< over internal 8 MHz oscillator for better stability.
                                            ///< X-axis chosen by convention.
/*************** Raw Data Output Registers ***************/
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40

#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42

#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

/****************** END OF MACROS ********************/

/**************** Functions' Headers *****************/
namespace MPU6050 {

  /**
  * @brief Initialise the MPU-6050 for Allan Variance benchmarking.
  *
  * Performs the following sequence:
  *  1. Verifies the WHO_AM_I register (expected 0x68).
  *  2. Resets the device and waits for it to boot.
  *  3. Wakes the device; selects PLL with X-axis gyro as clock source.
  *  4. Disables DLPF — Accel BW = 260 Hz, Gyro BW = 256 Hz.
  *  5. Configures gyro  : ±250 dps.
  *  6. Configures accel : ±2 g.
  *  7. Sets sample rate : 1 kHz (SMPRT_DIV = 7).
  *
  * @return  0  on success.
  * @return -1  if WHO_AM_I does not match (device not found / wrong address).
  */
  int benchmarkSetup();

  /**
  * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
  *
  * Reads registers 0x3B–0x48 in a single I²C transaction and populates the
  * IMUData struct with raw signed 16-bit values:
  *
  * @code
  *   data[0] = Accel X    data[1] = Accel Y    data[2] = Accel Z
  *   data[3] = Temp
  *   data[4] = Gyro X     data[5] = Gyro Y     data[6] = Gyro Z
  * @endcode
  *
  * @param[out] imuData  Pointer to the IMUData struct to populate.
  *                      Must not be NULL.
  */
  void readData(IMUData* imuData);

} 

#endif 
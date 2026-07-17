/**
 * @file    ICM20948.h
 * @brief   Driver interface for the ICM-20948 9-axis IMU (Accel + Gyro only).
 *
 * Configured for Allan Variance benchmarking:
 *   - Accel full-scale : ±2 g
 *   - Gyro  full-scale : ±250 dps
 *   - Sample rate      : 1.125 kHz  (SMPLRT_DIV = 0)
 *   - Accel DLPF       : 3 dB @ 246 Hz,   NBW = 265 Hz
 *   - Gyro  DLPF       : 3 dB @ 196.6 Hz, NBW = 229.8 Hz
 *   - ODR alignment    : enabled (synchronises accel and gyro start times
 *                        within the same device; has no effect across
 *                        separate physical IMUs)
 *
 * @note    Requires the Arduino Wire library. Call Wire.begin() before
 *          calling any function in this driver.
 *
 *
 * @note For more details about the module, datasheet notes and implementation
 *       please refer to the module page on Notion.
 * @author Abdelrahman Hewala
 * @note Prof. Lutz Leutelt
 * @date    2026
 */

#ifndef ICM20948_H
#define ICM20948_H

#include <Wire.h>
#include <stdint.h>
#include "IMUTypes.h"

/********************************* REGISTER MAP & MACROS **********************************/
/*************** REGISTER BANKS *****************/
#define ICM20948_REG_BANK_SEL          0x7F
// Values to write to REG_BANK_SEL
#define ICM20948_BANK_0           0x00
#define ICM20948_BANK_1           0x10
#define ICM20948_BANK_2           0x20
#define ICM20948_BANK_3           0x30

// AD0 pin LOW  → 0x68
// AD0 pin HIGH → 0x69
#define ICM20948_I2C_ADDR_LOW   0x68
#define ICM20948_I2C_ADDR_HIGH  0x69

/********* Bank0, Power, Control & out uts Registers **********/
// WHO_AM_I → should read 0xEA
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_WHO_AM_I_VAL   0xEA

// Power Management
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07

// PWR_MGMT_1 bit masks
#define ICM20948_RESET          0x80   // full device reset
#define ICM20948_SLEEP_EN       0x40   // sleep mode
#define ICM20948_LP_EN          0x20   // low power mode
#define ICM20948_TEMP_DIS       0x08   // disable temp sensor - not used! - Temp is enabled
#define ICM20948_CLK_PLL        0x01   // auto-select best clock (recommended)

// PWR_MGMT_2: write 0x00 to enable BOTH accel + gyro (all axes)
#define ICM20948_ACCEL_GYRO_ON  0x00

// Raw data output registers (Bank 0)
#define ICM20948_ACCEL_XOUT_H   0x2D
#define ICM20948_ACCEL_XOUT_L   0x2E
#define ICM20948_ACCEL_YOUT_H   0x2F
#define ICM20948_ACCEL_YOUT_L   0x30
#define ICM20948_ACCEL_ZOUT_H   0x31
#define ICM20948_ACCEL_ZOUT_L   0x32

#define ICM20948_GYRO_XOUT_H    0x33
#define ICM20948_GYRO_XOUT_L    0x34
#define ICM20948_GYRO_YOUT_H    0x35
#define ICM20948_GYRO_YOUT_L    0x36
#define ICM20948_GYRO_ZOUT_H    0x37
#define ICM20948_GYRO_ZOUT_L    0x38

// Interrupt status (poll for data ready)
#define ICM20948_INT_STATUS_1   0x1A   // bit 0 = RAW_DATA_0_RDY

/***********Bank 2 Gyro & Accel Config ***********/
/***** GYROSCOPE CONFIG *****/
#define ICM20948_GYRO_SMPLRT_DIV    0x00
// ODR = 1.125 kHz / (1 + DIV) → DIV=0 → 1.125 kHz ≈ 1 kHz target
#define ICM20948_GYRO_SMPLRT_DIV_VAL  0x00

#define ICM20948_GYRO_CONFIG_1      0x01
// Bits: [7:6] reserved | [5:3] GYRO_DLPFCFG | [2:1] GYRO_FS_SEL | [0] GYRO_FCHOICE
//
// GYRO_FS_SEL  = 0b00 → ±250 °/s   (bits [2:1])
// GYRO_FCHOICE = 1    → enable DLPF (bit [0])
// GYRO_DLPFCFG = 0b110 → 3dB @ 196.6 Hz, NBW 229.8 Hz  (bits [5:3]) ← closest to MPU6050's 256 Hz
//
// Value: 0b00_110_00_1 = 0x31
#define ICM20948_GYRO_CONFIG_1_VAL  0x31

#define ICM20948_GYRO_CONFIG_2      0x02
// Bits [2:0] = GYRO_AVGCFG (averaging filter in low-power; irrelevant here, keep 0)
// Bits [4:3] = XGYRO/YGYRO/ZGYRO self-test (keep 0)
#define ICM20948_GYRO_CONFIG_2_VAL  0x00   // no self-test, no averaging

/***- ACCELEROMETER CONFIG *****/
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10   // High byte of 12-bit divider
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11   // Low  byte of 12-bit divider
// ODR = 1.125 kHz / (1 + DIV)
// For 1.125 kHz: DIV = 0  → write 0x00 to both _1 and _2
#define ICM20948_ACCEL_SMPLRT_DIV_1_VAL  0x00
#define ICM20948_ACCEL_SMPLRT_DIV_2_VAL  0x00

#define ICM20948_ACCEL_CONFIG       0x14
// Bits: [5:3] ACCEL_DLPFCFG | [2:1] ACCEL_FS_SEL | [0] ACCEL_FCHOICE
//
// ACCEL_FS_SEL  = 0b00 → ±2g         (bits [2:1])
// ACCEL_FCHOICE = 1    → enable DLPF  (bit [0])
// ACCEL_DLPFCFG = 0b110 → 3dB @ 246 Hz, NBW 265 Hz  (bits [5:3]) ← closest to MPU6050's 260 Hz
//
// Value: 0b00_110_00_1 = 0x31
#define ICM20948_ACCEL_CONFIG_VAL   0x31

#define ICM20948_ACCEL_CONFIG_2     0x15
// Bits [1:0] = DEC3_CFG (number of samples averaged; 0 = 1 sample, no extra averaging)
#define ICM20948_ACCEL_CONFIG_2_VAL 0x00

// ODR alignment (to sync accel + gyro timing)
#define ICM20948_ODR_ALIGN_EN       0x09
#define ICM20948_ODR_ALIGN_EN_VAL   0x01   // enable ODR start-time alignment
/****************** END OF MACROS*********************/

/**************** Functions' Headers *****************/
namespace ICM20948_I2C {

/**
 * @brief Initialise the ICM-20948 for Allan Variance benchmarking.
 *
 * Performs the following sequence:
 *  1. Verifies the WHO_AM_I register (expected 0xEA).
 *  2. Resets the device and waits for it to boot.
 *  3. Disables sleep mode; selects auto clock (PLL).
 *  4. Enables accel + gyro on all axes.
 *  5. Configures gyro : ±250 dps, DLPF 3 dB @ 196.6 Hz (NBW 229.8 Hz),
 *                       ODR 1.125 kHz.
 *  6. Configures accel: ±2 g,     DLPF 3 dB @ 246 Hz   (NBW 265 Hz),
 *                       ODR 1.125 kHz.
 *  7. Enables ODR start-time alignment between accel and gyro.
 *
 * @return  0  on success.
 * @return -1  if WHO_AM_I does not match (device not found / wrong address).
 */
int benchmarkSetup();

//int operationSetup();   // Will be Coded soon

/**
 * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
 *
 * Reads registers 0x2D–0x3A in a single I²C transaction and populates the
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


} // namespace ICM20948

#endif 
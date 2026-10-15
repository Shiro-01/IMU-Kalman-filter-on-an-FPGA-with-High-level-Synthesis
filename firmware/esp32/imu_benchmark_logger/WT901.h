/**
 * @file    WT901.h
 * @brief   Driver interface for the WitMotion WT901 9-axis IMU (Accel + Gyro only).
 *
 * Configured for Allan Variance benchmarking:
 *   - Accel full-scale : ±16 g (fixed — not configurable by design)
 *   - Gyro  full-scale : ±2000 dps (fixed — not configurable by design)
 *   - Sample rate      : 200 Hz (hardware maximum)
 *   - Bandwidth        : 256 Hz (maximum, register 0x1F = 0x0000)
 *   - Algorithm        : 6-axis mode (magnetometer fusion disabled)
 *   - Orientation      : Horizontal
 *
 * @note    Fixed full-scale ranges result in higher quantisation noise compared
 *          to the MPU6050 and ICM20948. Quantisation noise is identifiable as a
 *          distinct -1 slope at short averaging times in the Allan Variance plot
 *          and does not affect bias instability or drift characterisation.
 *          Effective noise density is normalised by full-scale range for comparison.
 *
 * @note    WT901 is little-endian — no byte swapping required (unlike MPU6050
 *          and ICM20948 which are big-endian).
 *
 * @note    Output rate is hardware-limited to 200 Hz. Allan Variance τ_min = 5 ms.
 *          Bias instability and drift coefficients remain fully comparable with
 *          higher-rate sensors as these are intrinsic sensor properties independent
 *          of sample rate.
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

#ifndef WT901_H
#define WT901_H

#include <Wire.h>
#include <stdint.h>
#include "IMUTypes.h"

/********************************* REGISTER MAP & MACROS **********************************/

/*************** Device I2C Address ***************/
#define WT901_I2C_ADDR              0x50    ///< Default I2C address (fixed)

/*************** Configuration Registers ***************/
#define WT901_REG_SAVE              0x00    ///< Save current config to flash
#define WT901_REG_CALSW             0x01    ///< Calibration control
#define WT901_REG_RRATE             0x03    ///< Output rate control
#define WT901_REG_BAUD              0x04    ///< Baud rate (UART only)
#define WT901_REG_ORIENT            0x23    ///< Orientation setting
#define WT901_REG_AXIS6             0x24    ///< Fusion algorithm (6 vs 9 axis)
#define WT901_REG_LOCK              0x69    ///< Register lock/unlock

/*************** Configuration Values ***************/
// Unlock code — must be written to LOCK register before any config change
// Little-endian: LSB first → {0x88, 0xB5}
#define WT901_UNLOCK_L              0x88
#define WT901_UNLOCK_H              0xB5

// Save code — must be written to SAVE register after any config change
#define WT901_SAVE_L                0x00
#define WT901_SAVE_H                0x00

// Output rate → 200 Hz (maximum)
// From register map: 0x000B = 200 Hz
#define WT901_RRATE_200HZ_L         0x0B
#define WT901_RRATE_200HZ_H         0x00

// Orientation → Horizontal (0x0000)
#define WT901_ORIENT_HORIZONTAL_L   0x00
#define WT901_ORIENT_HORIZONTAL_H   0x00

// Fusion algorithm → 6-axis (magnetometer disabled) (0x0001)
#define WT901_AXIS6_L               0x01
#define WT901_AXIS6_H               0x00

/*************** Data Output Registers ***************/
// All registers are 16-bit, little-endian (LSB at lower address)
// Each register address returns 2 bytes

#define WT901_REG_AX                0x34    ///< Accel X low byte
#define WT901_REG_AY                0x35    ///< Accel Y low byte
#define WT901_REG_AZ                0x36    ///< Accel Z low byte
#define WT901_REG_GX                0x37    ///< Gyro  X low byte
#define WT901_REG_GY                0x38    ///< Gyro  Y low byte
#define WT901_REG_GZ                0x39    ///< Gyro  Z low byte
#define WT901_REG_TEMP              0x40    ///< Temperature low byte

#define WT901_REG_BANDWIDTH         0x1F    ///< Bandwidth control register

// Bandwidth values — bits [3:0] only, bits [15:4] reserved
// Default is 0x0004 = 20Hz — we want maximum = 256Hz
#define WT901_BANDWIDTH_256HZ_L     0x00    ///< 256 Hz — maximum bandwidth (0x0000)
#define WT901_BANDWIDTH_256HZ_H     0x00
/****************** END OF MACROS ********************/

/****************** Functions ************************/
namespace WT901 {

/**
 * @brief Initialise the WT901 for Allan Variance benchmarking.
 *
 * Performs the following sequence:
 *  0. Unlocks the configuration registers.
 *  1. Sets Kalman bandwidth to 256 Hz (maximum) — default is 20 Hz.
 *  2. Sets output rate to 200 Hz (hardware maximum).
 *  3. Sets orientation to horizontal.
 *  4. Enables 6-axis fusion mode (magnetometer disabled).
 *  5. Saves configuration to flash.
 *
 * @note    Accel (±16g) and Gyro (±2000°/s) full-scale ranges are fixed
 *          by the manufacturer and cannot be configured.
 *
 * @note    The WT901 applies internal Kalman filtering with a configurable
 *          bandwidth of 5–256 Hz. For benchmarking, bandwidth is set to
 *          256 Hz (maximum) to minimise filter influence on ARW characterisation.
 *          ARW normalisation uses NBW = 256 Hz directly from the datashee
 *
 * @return  0  on success.
 * @return -1  if device not found on I²C bus.
 */
int benchmarkSetup();

/**
 * @brief Burst-read accelerometer and gyroscope data (12 bytes) from the WT901.
 *
 * Reads registers 0x34–0x39 in a single I²C transaction and populates the
 * IMUData struct with raw signed 16-bit values:
 *
 * @code
 *   data[0] = Accel X    data[1] = Accel Y    data[2] = Accel Z
 *   data[3] = Temp       (read separately — not contiguous with accel/gyro)
 *   data[4] = Gyro X     data[5] = Gyro Y     data[6] = Gyro Z
 * @endcode
 *
 * @note    WT901 is little-endian — no byte swapping required.
 *
 * @note    Accel and Gyro registers (0x34–0x39) are contiguous and burst-readable
 *          in a single transaction. Temperature (0x40) is read separately.
 *
 * @param[out] imuData  Pointer to the IMUData struct to populate.
 *                      Must not be NULL.
 */
void readData(IMUData* imuData);

} // namespace WT901

#endif // WT901_H
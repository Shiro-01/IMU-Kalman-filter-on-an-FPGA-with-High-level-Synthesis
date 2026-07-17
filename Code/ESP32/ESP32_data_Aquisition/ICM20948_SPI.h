/**
 * @file    ICM20948_SPI.h
 * @brief   SPI driver interface for the ICM-20948 9-axis IMU (Accel + Gyro only).
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
 * @note    SPI bus replaces I²C for significantly reduced read latency.
 *          At 7MHz SPI, 14-byte burst read ≈ 16µs vs ~140µs on I²C at 1MHz.
 *
 * @note    SPI read protocol: assert CS low, send (reg | 0x80) for read,
 *          send (reg & 0x7F) for write, then transfer data bytes, deassert CS.
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

#ifndef ICM20948_SPI_H
#define ICM20948_SPI_H

#include <SPI.h>
#include <stdint.h>
#include "IMUTypes.h"
#include <Arduino.h>  

/********************************* PIN DEFINITIONS **********************************/
#define ICM_CS      32      ///< Chip Select — active LOW
#define ICM_SCK     25      ///< SPI Clock
#define ICM_MISO    26      ///< Master In Slave Out
#define ICM_MOSI    27      ///< Master Out Slave In

#define ICM_SPI_FREQ    6000000     ///< 7MHz — within ICM20948 spec (max 7MHz) - under real Conditions /Mhz doesn'T work, so we falled after test to 6 MHz
#define ICM_SPI_MODE    SPI_MODE3   ///< CPOL=1, CPHA=1 — per ICM20948 datasheet

/********************************* REGISTER MAP & MACROS **********************************/

/*************** REGISTER BANKS ***************/
#define ICM20948_REG_BANK_SEL           0x7F

#define ICM20948_BANK_0                 0x00
#define ICM20948_BANK_1                 0x10
#define ICM20948_BANK_2                 0x20
#define ICM20948_BANK_3                 0x30

/********* Bank 0 — Power, Control & Output Registers **********/
#define ICM20948_WHO_AM_I               0x00
#define ICM20948_WHO_AM_I_VAL           0xEA

#define ICM20948_PWR_MGMT_1             0x06
#define ICM20948_PWR_MGMT_2             0x07

// PWR_MGMT_1 bit masks
#define ICM20948_RESET                  0x80
#define ICM20948_SLEEP_EN               0x40
#define ICM20948_LP_EN                  0x20
#define ICM20948_TEMP_DIS               0x08
#define ICM20948_CLK_PLL                0x01

#define ICM20948_ACCEL_GYRO_ON          0x00

// Raw data output registers
#define ICM20948_ACCEL_XOUT_H           0x2D
#define ICM20948_GYRO_XOUT_H            0x33
#define ICM20948_INT_STATUS_1           0x1A

/********* Bank 2 — Gyro & Accel Config **********/
#define ICM20948_GYRO_SMPLRT_DIV        0x00
#define ICM20948_GYRO_SMPLRT_DIV_VAL    0x00

#define ICM20948_GYRO_CONFIG_1          0x01
// GYRO_FS_SEL=00 (±250dps) | FCHOICE=1 | DLPFCFG=000 (196.6Hz, NBW 229.8Hz)
// Value: 0b00_000_00_1 = 0x01
#define ICM20948_GYRO_CONFIG_1_VAL      0x01

#define ICM20948_GYRO_CONFIG_2          0x02
#define ICM20948_GYRO_CONFIG_2_VAL      0x00

#define ICM20948_ACCEL_SMPLRT_DIV_1     0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2     0x11
#define ICM20948_ACCEL_SMPLRT_DIV_1_VAL 0x00
#define ICM20948_ACCEL_SMPLRT_DIV_2_VAL 0x00

#define ICM20948_ACCEL_CONFIG           0x14
// ACCEL_FS_SEL=00 (±2g) | FCHOICE=1 | DLPFCFG=000 (246Hz, NBW 265Hz)
// Value: 0b00_000_00_1 = 0x01
#define ICM20948_ACCEL_CONFIG_VAL       0x01

#define ICM20948_ACCEL_CONFIG_2         0x15
#define ICM20948_ACCEL_CONFIG_2_VAL     0x00

#define ICM20948_ODR_ALIGN_EN           0x09
#define ICM20948_ODR_ALIGN_EN_VAL       0x01

/****************** END OF MACROS ********************/

/****************** Functions ************************/
namespace ICM20948{

/**
 * @brief Initialise the ICM-20948 for Allan Variance benchmarking over SPI.
 *
 * Performs the following sequence:
 *  1. Initialises SPI bus and CS pin.
 *  2. Verifies WHO_AM_I register (expected 0xEA).
 *  3. Resets the device and waits for it to boot.
 *  4. Disables sleep mode; selects auto clock (PLL).
 *  5. Enables accel + gyro on all axes.
 *  6. Configures gyro : ±250 dps, DLPF 3dB @ 196.6 Hz (NBW 229.8 Hz),
 *                       ODR 1.125 kHz.
 *  7. Configures accel: ±2 g,     DLPF 3dB @ 246 Hz   (NBW 265 Hz),
 *                       ODR 1.125 kHz.
 *  8. Enables ODR start-time alignment between accel and gyro.
 *
 * @return  0  on success.
 * @return -1  if WHO_AM_I does not match (device not found / wrong wiring).
 */
int benchmarkSetup();

/**
 * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
 *
 * Single SPI transaction — assert CS, send ACCEL_XOUT_H | 0x80,
 * clock out 14 bytes, deassert CS.
 *
 * @code
 *   data[0] = Accel X    data[1] = Accel Y    data[2] = Accel Z
 *   data[3] = Temp
 *   data[4] = Gyro X     data[5] = Gyro Y     data[6] = Gyro Z
 * @endcode
 *
 * @note    ICM20948 is big-endian — byte swapping applied via __builtin_bswap16.
 * @note    Bank 0 is assumed active — benchmarkSetup() restores it at end
 *          of initialisation.
 *
 * @param[out] imuData  Pointer to the IMUData struct to populate.
 *                      Must not be NULL.
 */
void readData(IMUData* imuData);

} // namespace ICM20948

#endif 
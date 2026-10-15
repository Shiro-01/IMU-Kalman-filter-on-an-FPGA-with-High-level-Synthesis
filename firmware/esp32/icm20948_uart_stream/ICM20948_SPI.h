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

// GYRO_FS_SEL=11 (±2000dps) | FCHOICE=1 | DLPFCFG=111 ( 7 -> 361.4 Hz)
// Value: 0b00_111_11_1 = 0x01
#define ICM20948_GYRO_CONFIG_1_VAL_RUNNING      0x3F

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

// ACCEL_FS_SEL=11 (±16g) | FCHOICE=1 | DLPFCFG=111 (3 dB: 473 Hz)
// Value: 0b00_111_11_1 = 0x3f
#define ICM20948_ACCEL_CONFIG_VAL_RUNNING       0x3f

#define ICM20948_ACCEL_CONFIG_2         0x15
#define ICM20948_ACCEL_CONFIG_2_VAL     0x00

#define ICM20948_ODR_ALIGN_EN           0x09
#define ICM20948_ODR_ALIGN_EN_VAL       0x01      // Enables ODR start-time alignment

/************************ MAGNETOMETER MACROS ***********************************/
/******** ICM20948 I2C MATER Control registers **********/
/****************** Bank 0 ****************/
#define ICM20948_USER_CTRL_ADDR         0x03
#define ICM20948_USER_CTRL_VAL          0x20            // Enable internal I2C Master of ICM20948 
#define ICM20948_I2C_MST_RST            0x02            // Internal I2C buss Reset Value for the user control register



#define ICM20948_INT_PIN_CFG_REG        0x0f
#define ICM20948_INT_PIN_CFG_VAL        0x00            // Disable the BYPASS Option for I2C

#define ICM20948_I2C_MST_STATUS         0x17            // status of the internal I2C buss
#define ICM20948_SLV4_DONE_MASK         0X40


/**************     Bank 3     ************/
#define ICM20948_MST_CTRL_ADDR          0x01            
#define ICM20948_MST_CTRL_VAL           0x07   // clock=7, P_NSR=1 (Restart between reads)

#define ICM20948_I2C_MST_DELAY_CTRL_ADDR  0x02
#define ICM20948_I2C_MST_DELAY_CTRL_VAL   0x80          // Delays shadowing of external sensor data until all data is received.-  // DELAY_ES_SHADOW + SLV0_DELAY_EN

/****************SLV4 Regsiters******************/
// Slv4 is used to write AK09916 CTRL2 regsiter only once
// SLV4 only runs once and doesn't get repeated-> hardware constrained
#define ICM20948_I2C_SLV4_ADDR          0x13      // Value =  0x80 (read bit) | 0x0C (AK09916 addr)
#define ICM20948_I2C_SLV4_REG_ADDR      0x14      // Value = AK09916_CTRL2_ADDR
#define ICM20948_I2C_SLV4_DO_ADDR       0x16      // Value = AK09916_CTRL2_VAL // Mode 4, 100 Hz - Should be written before writing the control reg
#define ICM20948_I2C_SLV4_CTRL_ADDR     0x15      
#define ICM20948_I2C_SLV4_CTRL_VAL      0x80      // Value = Enable data transfer - no INT - no register setting - no delays  

#define ICM20948_I2C_SLV4_DI_ADDR        0x17
/*************** SLV0 Registers *****************/
// SLV0 is being used to continously read the MAG readings. 
// Note the ODR is set to equalt he GYRO ODR 1.125 kHz. and the MAG MAX Freq is 100 Hz. we will check the status flag if a new transition is done or not
#define ICM20948_I2C_SLV0_ADDR          0x03      // 0x80 (read bit) | 0x0C (AK09916 addr)
#define ICM20948_I2C_SLV0_REG_ADDR      0x04      // Value = AK09916_ST1_ADDR

#define ICM20948_I2C_SLV0_CTRL_ADDR     0x05      
#define ICM20948_I2C_SLV0_ENABLE_BIT    0x80     // Value = Enable data transfer- no swaping - no DIS - no grouping- 9 bytes. 1,0,0,0, 1001

#define ICM20948_EXT_SLV_SENS_DATA_00   0x3B     // Address of the first read byte 
#define ICM20948_EXT_SLV_SENS_DATA_08   0x43     // Address of the last read byte 

/************* AK09916 Mag Chip Registers **************/
#define AK09916_I2C_ADDR                0x0C
#define AK09916_ST1_ADDR                0x10
#define AK09916_ST2_ADDR                0x18
#define AK09916_CTRL2_ADDR              0x31 
#define AK09916_CTRL2_VAL               0x08            // Mode 4: 100 Hz. 
#define AK09916_CNTL_3                 0x32            // reset register 
#define AK09916_CNTL_3_VAL              0x01            // reset Value

#define AK09916_WIA_1    0x00 // Who I am, Company ID
#define AK09916_WIA_2    0x01 // Who I am, Device ID

#define AK09916_WHO_AM_I_1      0x4809
#define AK09916_WHO_AM_I_2      0x0948

/************* Physical Conversion Macros **************/
#define ACC_LSBs_PER_1_G                  2048.0f       // LSB/g -  ±16g
#define GYRO_LSBs_PER_DPS                 16.4f         //LSB/(dps) -  ±2000dps


/****************** END OF MACROS ********************/

/************* Mag Parameters *************/
static int16_t last_mag[3] = {0, 0, 0};         // Hold the last read mag data. Used to determine if the new data point is new or not
static bool first_read = true;                  // True if the new mag data is differnet from the old one




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

/**
 * @brief Initialise the ICM-20948 for running over SPI.
 *
 * Performs the following sequence:
 *  1. Initialises SPI bus and CS pin.
 *  2. Verifies WHO_AM_I register (expected 0xEA).
 *  3. Resets the device and waits for it to boot.
 *  4. Disables sleep mode; selects auto clock (PLL).
 *  5. Enables accel + gyro on all axes + enable Mag.
 * 
 *  *** Gyro DLPF setting -> 7: 3DB BW: 361.4 Hz
 *            *** Acc  DLPF setting -> 7: 3DB BW: 473 Hz
 *            *** Mag rate Set to 100 Hz
 * 
 * @note     DLPF can be turned off totally, but in this specifc situation the BW is much larger than the max
 *           Possible sampling Frequancy. This leads to alaising effect to occur. 
 *           and its effect can not be undone even with KF
 *           making our resutls much worse. So we decided to go with the bigest possiple BW filter instead
 * 
 * @note     Mag Sensitivity is constant : 0.15 µT/LSB (typ.)
 * 
 * @return  0  on success.
 * @return -1  if WHO_AM_I does not match (device not found / wrong wiring).
 */
int runningSetup();



/**
 * @brief Parse IMU Raw data stream into counts with the right endians
 *
 * @param IMUDataStream  Pointer to the IMUDataStream struct to read the data from
 * @return  IMUSample
 */
IMUSample parseIMUData(const IMUDataStream* raw);


/**
 * @brief convert IMUSample into meaningfull physical values based on the defined ranges macros in the section "Physical Conversion Macro"
 * @param IMUSample  Pointer to the IMUSample struct that needs to be converted
 * @return  IMUPhysical  returns the physical reading of the IMU stored in IMUPhysical struct
 */
IMUPhysical toPhysical(const IMUSample& s);


/**
 * @brief Read IMU data ready to be used in Physical quantities
 */
IMUPhysical readStreamData();

/**
 * @brief Benchmarks ICM20948::readStreamData() timing over N samples.
 *
 * @note  Debug/characterization utility — not part of the runtime data
 *        pipeline. Excludes any print/serial overhead from the measured
 *        region.
 *
 * @param sampleCount  Number of readStreamData() calls to time.
 * @return BenchResult with min/max/avg timing in microseconds.
 */
BenchResult benchmarkReadStreamData(int sampleCount);

/**
 * @brief Prints a BenchResult in a readable format.
 */
void printBenchResult(const char* label, const BenchResult& r); 

}

#endif 
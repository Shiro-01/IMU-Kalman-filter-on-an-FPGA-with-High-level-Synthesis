/**
 * @file    WT901.cpp
 * @brief   Driver implementation for the WitMotion WT901 9-axis IMU (Accel + Gyro only).
 *
 * Implements the functions declared in WT901.h. Communication is handled
 * via the Arduino Wire library (I²C).
 *
 * All configuration register writes follow the WT901 write protocol:
 *  1. Unlock the chip (write unlock code to LOCK register)
 *  2. Write the configuration register
 *  3. Save to flash (write save code to SAVE register)
 *
 * Unlike ICM20948, the WT901 has no single-die register bank —
 * all registers are directly accessible at a fixed I²C address of 0x50.
 * No bank switching is required.
 *
 * @note    WT901 is little-endian — no byte swapping required.
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

/********************************* Includes **********************************/
#include "WT901.h"

/********************************* Helpers ***********************************/

/**
 * @brief   Unlock the WT901 configuration registers.
 *
 * @note    Must be called before every configuration write.
 *          The WT901 re-locks after each save operation.
 */
static void unlock() {
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(WT901_REG_LOCK);
    Wire.write(WT901_UNLOCK_L);
    Wire.write(WT901_UNLOCK_H);
    Wire.endTransmission();
}

/**
 * @brief   Save current configuration to flash.
 *
 * @note    Must be called after every configuration write.
 *          Failure to save means settings revert on power cycle.
 */
static void save() {
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(WT901_REG_SAVE);
    Wire.write(WT901_SAVE_L);
    Wire.write(WT901_SAVE_H);
    Wire.endTransmission();
}

/**
 * @brief   Write a 16-bit value to a WT901 configuration register.
 *
 * @note    Follows the WT901 write protocol: unlock → write → save.
 *          Little-endian: LSB sent first.
 *
 * @param   reg     Register address to write to.
 * @param   valL    Low byte of the 16-bit value.
 * @param   valH    High byte of the 16-bit value.
 */
static void writeReg(uint8_t reg, uint8_t valL, uint8_t valH) {
    unlock();
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(reg);
    Wire.write(valL);
    Wire.write(valH);
    Wire.endTransmission();
    save();
}

namespace WT901 {

  /**
    * @brief Initialise the WT901 for Allan Variance benchmarking.
    * @return  0  on success, -1 if device not found on I²C bus.
    */
  int benchmarkSetup() {

    /*── 0. Verify device is present on I²C bus ──────────────────*/
    Wire.beginTransmission(WT901_I2C_ADDR);
    if (Wire.endTransmission() != 0) return -1;

    /*── 1. Set output rate — 200 Hz (hardware maximum) ──────────*/
    writeReg(WT901_REG_RRATE,
              WT901_RRATE_200HZ_L,
              WT901_RRATE_200HZ_H);
    delay(10);

    /*── 2. Set bandwidth — 256 Hz (maximum) ─────────────────────*/
    // Default is 20Hz (0x0004) — must explicitly set to 256Hz (0x0000)
    // This is the internal Kalman filter bandwidth of the onboard MCU
    writeReg(WT901_REG_BANDWIDTH,
            WT901_BANDWIDTH_256HZ_L,
            WT901_BANDWIDTH_256HZ_H);
    delay(10);

    /*── 3. Set orientation — horizontal ─────────────────────────*/
    writeReg(WT901_REG_ORIENT,
              WT901_ORIENT_HORIZONTAL_L,
              WT901_ORIENT_HORIZONTAL_H);
    delay(10);

    /*── 4. Set fusion algorithm — 6-axis (mag disabled) ─────────*/
    writeReg(WT901_REG_AXIS6,
              WT901_AXIS6_L,
              WT901_AXIS6_H);
    delay(10);

    return 0;
  }

  /**
    * @brief Burst-read accelerometer, temperature and gyroscope data.
    *
    * Two I²C transactions:
    *  - Transaction 1: registers 0x34–0x39 (12 bytes) → accel + gyro
    *  - Transaction 2: register  0x40       (2 bytes)  → temperature
    *
    * Temperature register is not contiguous with accel/gyro (registers
    * 0x3A–0x3F contain Roll, Pitch, Yaw which are not needed), so a
    * separate transaction is used rather than a wasteful 26-byte burst.
    *
    * @param[out] imuData  Pointer to the IMUData struct to populate.
    *                      Must not be NULL.
    */
  void readData(IMUData* imuData) {

    /*── Transaction 1: Accel + Gyro (12 bytes) ──────────────────*/
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(WT901_REG_AX);
    Wire.endTransmission(false);
    Wire.requestFrom(WT901_I2C_ADDR, 12, true);

    // Direct block read into struct memory — no intermediate buffer
    // WT901 is little-endian — no byte swapping required
    Wire.readBytes((uint8_t*)&imuData->data[0], 12);

    /*── Transaction 2: Temperature (2 bytes) ────────────────────*/
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(WT901_REG_TEMP);
    Wire.endTransmission(false);
    Wire.requestFrom(WT901_I2C_ADDR, 2, true);

    Wire.readBytes((uint8_t*)&imuData->data[6], 2);
    /*── Fix layout — swap temp into correct position ────────────*/
    // Current:  AX AY AZ GX GY GZ TEMP
    // Target:   AX AY AZ TEMP GX GY GZ
    int16_t gx   = imuData->data[3];
    int16_t gy   = imuData->data[4];
    int16_t gz   = imuData->data[5];
    int16_t temp = imuData->data[6];
    imuData->data[3] = temp;
    imuData->data[4] = gx;
    imuData->data[5] = gy;
    imuData->data[6] = gz;
  }

} // namespace WT901
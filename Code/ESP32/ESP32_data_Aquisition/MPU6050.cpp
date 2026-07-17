/**
 * @file    MPU6050.cpp
 * @brief   Driver implementation for the MPU-6050 6-axis IMU (Accel + Gyro only).
 *
 * Implements the functions declared in MPU6050.h. Communication is handled
 * via the Arduino Wire library (I²C).
 *
 * Unlike the ICM-20948, the MPU-6050 has a single register bank so no
 * bank switching is required. All registers are directly accessible.
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

/********************************* Includes **********************************/
#include "MPU6050.h"

/************************* Functions Implementations *************************/
namespace MPU6050 {

  /**
    * @brief Initialise the MPU-6050 for Allan Variance benchmarking.
    *
    * Performs the following sequence:
    *  1. Verifies the WHO_AM_I register (expected 0x68).
    *  2. Resets the device and waits for it to boot.
    *  3. Wakes the device; selects PLL with X-axis gyro as clock source.
    *  4. Configures gyro  : ±250 dps.
    *  5. Configures accel : ±2 g.
    *  6. Disables DLPF    : Accel BW = 260 Hz, Gyro BW = 256 Hz.
    *  7. Sets sample rate : 1 kHz (SMPRT_DIV = 7,
    *                        Fs = 8 kHz / (1 + 7) = 1 kHz).
    *
    * @return  0  on success.
    * @return -1  if WHO_AM_I does not match (device not found / wrong address).
    */
  int benchmarkSetup() {
    /*── 1. Verify WHO_AM_I ──────────────────────────────────────*/
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_I2C_ADDR_LOW, 1);
    if (Wire.read() != MPU6050_WHO_AM_I_VAL) return -1;

    /*── 2. Full device reset ────────────────────────────────────*/
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(MPU6050_RESET);
    Wire.endTransmission();
    delay(50);                          // wait for reset to complete

    /*── 3. Wake up + PLL clock source ──────────────────────────*/
    // Datasheet (Section 4.28) recommends using any gyro axis PLL (CLKSEL = 1, 2, or 3)
    // over the internal 8 MHz oscillator for improved clock stability.
    // X-axis gyro PLL (0x01) selected by convention
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(MPU6050_CLK_PLL_XGYRO);  // clears SLEEP, selects PLL clock
    Wire.endTransmission();

    /*── 4. Gyro full-scale ±250 dps ────────────────────────────*/
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(MPU6050_GYRO_CONFIG_VAL);
    Wire.endTransmission();

    /*── 5. Accel full-scale ±2 g ───────────────────────────────*/
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(MPU6050_ACCEL_CONFIG_VAL);
    Wire.endTransmission();

    /*── 6. Disable DLPF — maximum bandwidth ────────────────────*/
    // DLPF_CFG = 0 → Accel BW 260 Hz, Gyro BW 256 Hz
    // Gyro internal Fs = 8 kHz when DLPF is disabled
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_CONFIG);
    Wire.write(MPU6050_CONFIG_VAL);
    Wire.endTransmission();

    /*── 7. Sample rate — 1 kHz ─────────────────────────────────*/
    // Fs = Gyro Fs / (1 + SMPRT_DIV) = 8000 / (1 + 7) = 1000 Hz
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_SMPRT_DIV);
    Wire.write(MPU6050_SMPRT_DIV_VAL);
    Wire.endTransmission();

    return 0;
  }

  /**
    * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
    *
    * Reads registers 0x3B–0x48 in a single I²C transaction and populates
    * the IMUData struct with raw signed 16-bit values:
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
  void readData(IMUData* imuData) {
    Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_I2C_ADDR_LOW, 14, true);

    // Direct block read into struct memory — no intermediate buffer
    Wire.readBytes((uint8_t*)imuData->data, 14);

    // MPU-6050 is big-endian, ESP32 is little-endian — swap each 16-bit word
    for (int i = 0; i < 7; i++) {
        imuData->data[i] = __builtin_bswap16(imuData->data[i]);
    }
  }
} 
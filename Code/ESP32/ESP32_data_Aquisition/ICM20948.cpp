/**
 * @file    ICM20948.cpp
 * @brief   Driver implementation for the ICM-20948 9-axis IMU (Accel + Gyro only).
 *
 * Implements the functions declared in ICM20948.h. Communication is handled
 * via the Arduino Wire library (I²C). 
 *
 * All register access must be return their Bank Selection back to bank0 
 * This to allow seamingless Data read from the sensors without the need to 
 * reset the Bank in each cycle
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

/********************************* Includes *********************************/
#include "ICM20948.h"

/********************************* Helpers **********************************/

/**
 * @brief   Select a user bank via the REG_BANK_SEL register.
 *
 * @note    This is a helper function. should be used before writing to any register
 *
 * @param   bank  Bank selector value: ICM20948_BANK_0 … ICM20948_BANK_3
 */
static void selectBank(uint8_t bank) {
    Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
    Wire.write(ICM20948_REG_BANK_SEL);
    Wire.write(bank);
    Wire.endTransmission();
}

/************************* Functions Implementations *************************/
namespace ICM20948_I2C {

    /**
    * @brief Initialise the ICM-20948 for Allan Variance benchmarking.
    * @return  0  on success, -1 if WHO_AM_I does not match.
    */
    int benchmarkSetup() {

        /*── 1. Verify WHO_AM_I ──────────────────────────────────────*/
        selectBank(ICM20948_BANK_0);
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_WHO_AM_I);
        Wire.endTransmission(false);
        Wire.requestFrom(ICM20948_I2C_ADDR_HIGH, (uint8_t)1);
        if (Wire.read() != ICM20948_WHO_AM_I_VAL) return -1;

        /*── 2. Full device reset ────────────────────────────────────*/
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_PWR_MGMT_1);
        Wire.write(ICM20948_RESET);
        Wire.endTransmission();
        delay(50);                          // wait for reset to complete

        /*── 3. Wake up + auto clock (PLL) ──────────────────────────*/
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_PWR_MGMT_1);
        Wire.write(ICM20948_CLK_PLL);       // clears SLEEP, selects best clock
        Wire.endTransmission();

        /*── 4. Enable accel + gyro on all axes ─────────────────────*/
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_PWR_MGMT_2);
        Wire.write(ICM20948_ACCEL_GYRO_ON);
        Wire.endTransmission();

        /*── 5. Bank 2 — Gyro config ─────────────────────────────────*/
        selectBank(ICM20948_BANK_2);

        // 5a. Gyro ODR — 1.125 kHz
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_GYRO_SMPLRT_DIV);
        Wire.write(ICM20948_GYRO_SMPLRT_DIV_VAL);
        Wire.endTransmission();

        // 5b. Gyro ±250 dps, DLPF enabled, 3dB @ 196.6 Hz, NBW 229.8 Hz
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_GYRO_CONFIG_1);
        Wire.write(ICM20948_GYRO_CONFIG_1_VAL);
        Wire.endTransmission();

        // 5c. No self-test, no extra averaging
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_GYRO_CONFIG_2);
        Wire.write(ICM20948_GYRO_CONFIG_2_VAL);
        Wire.endTransmission();

        /*── 6. Bank 2 — Accel config ────────────────────────────────*/
        // 6a. Accel ODR — 1.125 kHz (12-bit divider, high byte then low byte)
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ACCEL_SMPLRT_DIV_1);
        Wire.write(ICM20948_ACCEL_SMPLRT_DIV_1_VAL);
        Wire.endTransmission();

        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ACCEL_SMPLRT_DIV_2);
        Wire.write(ICM20948_ACCEL_SMPLRT_DIV_2_VAL);
        Wire.endTransmission();

        // 6b. Accel ±2g, DLPF enabled, 3dB @ 246 Hz, NBW 265 Hz
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ACCEL_CONFIG);
        Wire.write(ICM20948_ACCEL_CONFIG_VAL);
        Wire.endTransmission();

        // 6c. No extra averaging (DEC3_CFG = 0)
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ACCEL_CONFIG_2);
        Wire.write(ICM20948_ACCEL_CONFIG_2_VAL);
        Wire.endTransmission();

        /*── 7. ODR alignment — sync accel & gyro start times ───────*/
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ODR_ALIGN_EN);
        Wire.write(ICM20948_ODR_ALIGN_EN_VAL);
        Wire.endTransmission();

        selectBank(ICM20948_BANK_0);

        return 0;
    }

    /**
    * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
    *
    * Bank 0 is assumed active — benchmarkSetup() restores it at the end
    * of initialisation, avoiding redundant selectBank() calls on every read.
    *
    * @param[out] imuData  Pointer to the IMUData struct to populate. Must not be NULL.
    */
    void readData(IMUData* imuData) {
        // Benchmark method change the bank back to zero in the end. 
        //This allow us to avoid un neceary repeated calls of the funtion selectBank()
        Wire.beginTransmission(ICM20948_I2C_ADDR_HIGH);
        Wire.write(ICM20948_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(ICM20948_I2C_ADDR_HIGH, 14, true);

        // Direct block read into struct memory — no intermediate buffer
        Wire.readBytes((uint8_t*)imuData->data, 14);

        // ICM20948 is big-endian, ESP32 is little-endian — swap each 16-bit word
        for (int i = 0; i < 7; i++) {
            imuData->data[i] = __builtin_bswap16(imuData->data[i]);
        }
    }
} 
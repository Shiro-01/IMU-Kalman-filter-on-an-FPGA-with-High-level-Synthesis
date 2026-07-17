/**
 * @file    ICM20948_SPI.cpp
 * @brief   SPI driver implementation for the ICM-20948 9-axis IMU (Accel + Gyro only).
 *
 * Implements the functions declared in ICM20948_SPI.h. Communication is handled
 * via the Arduino SPI library.
 *
 * SPI protocol for ICM20948:
 *   - Read  : assert CS, send (reg | 0x80), clock in data bytes, deassert CS
 *   - Write : assert CS, send (reg & 0x7F), send data byte,      deassert CS
 *
 * Bank switching is handled internally via selectBank() before every
 * register access. benchmarkSetup() restores Bank 0 at the end so
 * readData() requires no bank switching in the data loop.
 *
 * @note    ICM20948 is big-endian — __builtin_bswap16 applied in readData().
 *
 * @note    For register map details, configuration rationale, and hardware
 *          wiring refer to the module page on Notion.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
 */

/********************************* Includes **********************************/
#include "ICM20948_SPI.h"

/********************************* Helpers ***********************************/

/**
 * @brief   SPI settings instance for ICM20948.
 *          Defined once here — reused in every transaction.
 */
static SPISettings icmSPISettings(ICM_SPI_FREQ, SPI_MSBFIRST, ICM_SPI_MODE);

/**
 * @brief   ICM Moudule Uses the second SPI of ESP32 (HSPI)
 *          Defined once here — reused in every transaction.
 *
 * @note    SD card, uses the Defualt setting of SPI lib, where it 
 *          sets the Periferal to the first SPI (VSPI)
 */
static SPIClass icmSPI(HSPI);

/**
 * @brief   Write a single byte to an ICM20948 register over SPI.
 *
 * @param   reg     Register address (bit 7 must be 0 — write mode).
 * @param   val     Byte value to write.
 */
static void spiWriteReg(uint8_t reg, uint8_t val) {
    icmSPI.beginTransaction(icmSPISettings);
    digitalWrite(ICM_CS, LOW);
    icmSPI.transfer(reg & 0x7F);       // bit 7 = 0 → write
    icmSPI.transfer(val);
    digitalWrite(ICM_CS, HIGH);
    icmSPI.endTransaction();
}

/**
 * @brief   Read a single byte from an ICM20948 register over SPI.
 *
 * @param   reg     Register address.
 * @return  Byte read from the register.
 */
static uint8_t spiReadReg(uint8_t reg) {
    uint8_t val;
    // icmSPI.beginTransaction(icmSPISettings);
    digitalWrite(ICM_CS, LOW);
    icmSPI.transfer(reg | 0x80);       // bit 7 = 1 → read
    val = icmSPI.transfer(0x00);       // dummy byte to clock in data
    digitalWrite(ICM_CS, HIGH);
    icmSPI.endTransaction();
    return val;
}

/**
 * @brief   Select a user bank via the REG_BANK_SEL register.
 *
 * @note    File-scope static — not exposed in the public header.
 *          Must be called before any register access in a different bank.
 *
 * @param   bank  ICM20948_BANK_0 … ICM20948_BANK_3
 */
static void selectBank(uint8_t bank) {
    spiWriteReg(ICM20948_REG_BANK_SEL, bank);
}

/************************* Function Implementations **************************/
namespace ICM20948 {

    /**
     * @brief Initialise the ICM-20948 for Allan Variance benchmarking over SPI.
     * @return  0  on success, -1 if WHO_AM_I does not match.
     */
    int benchmarkSetup() {

        /*── 1. Initialise SPI bus and CS pin ───────────────────────*/
        pinMode(ICM_CS, OUTPUT);
        digitalWrite(ICM_CS, HIGH);         // deassert CS
        icmSPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);

        /*── 2. Verify WHO_AM_I ──────────────────────────────────────*/
        selectBank(ICM20948_BANK_0);
        if (spiReadReg(ICM20948_WHO_AM_I) != ICM20948_WHO_AM_I_VAL) return -1;

        /*── 3. Full device reset ────────────────────────────────────*/
        spiWriteReg(ICM20948_PWR_MGMT_1, ICM20948_RESET);
        delay(50);                          // wait for reset to complete

        /*── 4. Wake up + auto clock (PLL) ──────────────────────────*/
        spiWriteReg(ICM20948_PWR_MGMT_1, ICM20948_CLK_PLL);

        /*── 5. Enable accel + gyro on all axes ─────────────────────*/
        spiWriteReg(ICM20948_PWR_MGMT_2, ICM20948_ACCEL_GYRO_ON);

        /*── 6. Bank 2 — Gyro config ─────────────────────────────────*/
        selectBank(ICM20948_BANK_2);

        // 6a. Gyro ODR — 1.125 kHz
        spiWriteReg(ICM20948_GYRO_SMPLRT_DIV, ICM20948_GYRO_SMPLRT_DIV_VAL);

        // 6b. Gyro ±250 dps, DLPF enabled, 3dB @ 196.6 Hz, NBW 229.8 Hz
        spiWriteReg(ICM20948_GYRO_CONFIG_1, ICM20948_GYRO_CONFIG_1_VAL);

        // 6c. No self-test, no extra averaging
        spiWriteReg(ICM20948_GYRO_CONFIG_2, ICM20948_GYRO_CONFIG_2_VAL);

        /*── 7. Bank 2 — Accel config ────────────────────────────────*/
        // 7a. Accel ODR — 1.125 kHz
        spiWriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, ICM20948_ACCEL_SMPLRT_DIV_1_VAL);
        spiWriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, ICM20948_ACCEL_SMPLRT_DIV_2_VAL);

        // 7b. Accel ±2g, DLPF enabled, 3dB @ 246 Hz, NBW 265 Hz
        spiWriteReg(ICM20948_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_VAL);

        // 7c. No extra averaging (DEC3_CFG = 0)
        spiWriteReg(ICM20948_ACCEL_CONFIG_2, ICM20948_ACCEL_CONFIG_2_VAL);

        /*── 8. ODR alignment — sync accel & gyro start times ───────*/
        spiWriteReg(ICM20948_ODR_ALIGN_EN, ICM20948_ODR_ALIGN_EN_VAL);

        /*── 9. Restore Bank 0 for readData() ───────────────────────*/
        selectBank(ICM20948_BANK_0);

        return 0;
    }

    /**
     * @brief Burst-read accelerometer, temperature and gyroscope data (14 bytes).
     *
     * Single SPI transaction — CS asserted once for all 14 bytes.
     * Significantly faster than I²C: ~16µs vs ~140µs at equivalent speeds.
     *
     * @param[out] imuData  Pointer to the IMUData struct to populate.
     *                      Must not be NULL.
     */
    void readData(IMUData* imuData) {
        icmSPI.beginTransaction(icmSPISettings);
        digitalWrite(ICM_CS, LOW);

        icmSPI.transfer(ICM20948_ACCEL_XOUT_H | 0x80);    // read mode

        // Burst clock in 14 bytes directly into struct memory
        icmSPI.transferBytes(nullptr, (uint8_t*)imuData->data, 14);

        digitalWrite(ICM_CS, HIGH);
        icmSPI.endTransaction();

        // ICM20948 is big-endian, ESP32 is little-endian — swap each 16-bit word
        for (int i = 0; i < 7; i++) {
            imuData->data[i] = __builtin_bswap16(imuData->data[i]);
        }

        uint16_t gx = imuData->data[3];
        uint16_t gy = imuData->data[4];
        uint16_t gz = imuData->data[5];

        uint16_t temp = imuData->data[6];

        imuData->data[3] = temp;

        imuData->data[4] = gx;
        imuData->data[5] = gy;
        imuData->data[6] = gz;
    }

} // namespace ICM20948
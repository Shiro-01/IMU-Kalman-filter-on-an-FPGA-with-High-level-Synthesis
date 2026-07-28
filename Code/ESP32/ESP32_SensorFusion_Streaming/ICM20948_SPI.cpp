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
 * @recognation The Mag parts are guided through Wolfgang (Wolle) Ewald lib.
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
 *          Defined once here & reused in every transaction.
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
 * @brief 
 * 
 *
 * @param   reg     Register address.
 * @return  Byte read from the register.
 */
static uint8_t spiReadReg(uint8_t reg) {
    uint8_t val;
    icmSPI.beginTransaction(icmSPISettings);
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

/***************** Mag Helper functions **************/

/**
 * @brief   Enables and Sets up the internal I2C Master of ICM20948 chip
 *
 * @note    This Function enables the internal I2C Master, Disable the bypass flag, and
 *          sets up the SPI clk to the recommended one in the data sheet.
 *          Clock freq: I2C_MST_CLK = 7 (345.6 kHz / 46.67% duty cycle).
 *
 */
void static enableI2CMaster(){

    selectBank(ICM20948_BANK_0);
    spiWriteReg(ICM20948_USER_CTRL_ADDR, ICM20948_USER_CTRL_VAL);           // Enable internal I2C Master of ICM20948 
    spiWriteReg(ICM20948_INT_PIN_CFG_REG, 0x00);                            // Disable the BYPASS Option for I2C

    selectBank(ICM20948_BANK_3);
    spiWriteReg(ICM20948_MST_CTRL_ADDR, ICM20948_MST_CTRL_VAL);             // recommended setting for I2C clock 
    delay(10);

    selectBank(ICM20948_BANK_0);                                            // setting the bank back to bank no. 0
}

/**
 * @brief   Writes the passed value to the passed register on AK09916 Mag chip
 *
 * @note    This Function utlise the one time slave4 regs of the internal bus to
 *          write one specified regsiter with the passed value.
 *          The writing process occurs once over the internal I2C buss
 * 
 * @param   reg  AK09916 register Address that we want to write to. 
 * @param   val  Value to write to the passed  AK09916register.
 */
void static writeAK09916Register8_SLV4(uint8_t reg, uint8_t val){
    selectBank(ICM20948_BANK_3);

    spiWriteReg(ICM20948_I2C_SLV4_ADDR, AK09916_I2C_ADDR);                    // write mode, bit7=0
    spiWriteReg(ICM20948_I2C_SLV4_DO_ADDR, val);                              // Value we want to write to the register
    spiWriteReg(ICM20948_I2C_SLV4_REG_ADDR, reg);                             // AK09916 Address wanted to write to
    spiWriteReg(ICM20948_I2C_SLV4_CTRL_ADDR, ICM20948_I2C_SLV4_CTRL_VAL);     // Enable data transfer - no INT - no register setting - no delays 
    
    unsigned long int startRead = millis();                                   // to avoid that the code hangs
    while((spiReadReg(ICM20948_I2C_SLV4_CTRL_ADDR) & ICM20948_I2C_SLV4_CTRL_VAL) && (millis() - startRead < 100)){;} // ICM20948_I2C_SLV4_CTRL_ADDR en bit get cleared once the transfer / write is done!
    
    selectBank(ICM20948_BANK_0);
}

/**
 * @brief   Read a register Value from AK09916 Mag chip
 *
 * @note    This Function utlise the one time slave4 regs to read one specified register
 *          The Reading process occurs once over the internal I2C buss. 
 *          and saves the read value to "ICM20948_I2C_SLV4_DI" Regsiter
 * 
 * @param   reg  AK09916 register Address that we want to read from. 
 * @return  Returns the bit vector stored in that register. 
 */
uint8_t static readAK09916Register8_SLV4(uint8_t reg){ 
    selectBank(ICM20948_BANK_3);
    
    spiWriteReg(ICM20948_I2C_SLV4_ADDR, AK09916_I2C_ADDR | 0x80);                   // read AK09916
    spiWriteReg(ICM20948_I2C_SLV4_REG_ADDR, reg);                                   // define AK09916 register to be read
    spiWriteReg(ICM20948_I2C_SLV4_CTRL_ADDR, ICM20948_I2C_SLV4_CTRL_VAL);

    unsigned long int startRead = millis();                                         // to avoid that the code hangs
    while((spiReadReg( ICM20948_I2C_SLV4_CTRL_ADDR) & ICM20948_I2C_SLV4_CTRL_VAL) && (millis() - startRead < 100)){;}

    uint8_t value = spiReadReg(ICM20948_I2C_SLV4_DI_ADDR);
    selectBank(ICM20948_BANK_0);
    return value;
}

/**
 * @brief   Resets the AK09916 Mag sensor
 *
 * @note    The reset command is sent to AK09916 chip through the internal  
 *          I2C bus master. using slave4 register set of ICM20948 Chip.
 *          this is done utlising the helper function "writeAK09916Register8_SLV4()"
 */
void static resetMag(){
    writeAK09916Register8_SLV4(AK09916_CNTL_3, AK09916_CNTL_3_VAL);
    delay(100);
}

/**
 * @brief   Resets ICM20948 Chip
 *
 */
void static reset_ICM20948(){
    selectBank(ICM20948_BANK_0);
    spiWriteReg(ICM20948_PWR_MGMT_1, ICM20948_RESET);
    delay(100);  // wait for registers to reset
}

/**
 * @brief   Resets the internal I2C bus Master
 *
 */
void i2cMasterReset(){
    selectBank(ICM20948_BANK_0);

    uint8_t regVal = spiReadReg(ICM20948_USER_CTRL_ADDR);
    regVal |= ICM20948_I2C_MST_RST;    
    spiWriteReg(ICM20948_USER_CTRL_ADDR, regVal);
    delay(10);  
}

/**
 * @brief   sets the regsiters that we would like to read from AK09916 Mag chip
 *
 * @note    The read start addreee and how many bytes to read are sent to AK09916 chip through the internal
 *          I2C bus 'Master. using slave0 registers set of ICM20948 Chip.
 * 
 * @param   reg  AK09916 register Address that we want to start reading from. 
 * @param   bytes  number of bytes ti read from that start address. 
 */
void static enableMagDataRead(uint8_t reg, uint8_t bytes){
    selectBank(ICM20948_BANK_3);

    spiWriteReg(ICM20948_I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDR);       // bit 7 = 1 for read mode
    spiWriteReg(ICM20948_I2C_SLV0_REG_ADDR, reg);                       // define AK09916 first register to be read
    spiWriteReg(ICM20948_I2C_SLV0_CTRL_ADDR, ICM20948_I2C_SLV0_ENABLE_BIT | bytes); //enable read | number of byte
    delay(10);

    selectBank(ICM20948_BANK_0);
}

/**
 * @brief   Reads Who AM I Regsiters on the AK09916 Mag chip
 * @note    The reset command is sent to AK09916 chip through the internal
 *          I2C bus master. using slave4 register set of ICM20948 Chip.
 *          this is done utlising the helper function "readAK09916Register8_SLV4()"
 * 
 * @return  Returns the WHOAMI Value either (0x4809) or (0x0948) depending on the endians. 
 */
uint16_t whoAmIMag(){ 
    uint8_t MSByte = readAK09916Register8_SLV4(AK09916_WIA_1);
    uint8_t LSByte = readAK09916Register8_SLV4(AK09916_WIA_2);
    uint16_t magID = (MSByte<<8) | LSByte;
    return magID;
}

/**
 * @brief  Sets up the AK09916 Mag chip to start measuring
 * 
 * 
 * @note    - this fucntion sets up ICM20948 internal I2C Bus Master to Communicate with AK09916.
 *          - Sets the Mag mode to Mode 4: 100 Hz.
 *          - Resets teh I2C bus Master if hangs
 *          - Specify the regsiters that we would like to read
 *          - read regsiters are stored in ICM20948 regsiters EXT_SLV_SENS_DATA_00 .... EXT_SLV_SENS_DATA_08 in order
 *          - Mag read ODR is done with Gyro Freq. So it will always overrun the mag true output.
 *          - the new data point is determined through the ready bit in the ST1 register of AK09916 Mag chip
 *    
 * @return  true if everything is done correctly and no errors. 
 */
bool initMagnetometer(){
    enableI2CMaster();
    resetMag();
    //reset_ICM20948();
    selectBank(ICM20948_BANK_2);
    spiWriteReg(ICM20948_ODR_ALIGN_EN, 1); // aligns ODR 
    
    bool initSuccess = false;
    uint8_t tries = 0;
    while(!initSuccess && (tries < 10)){ // max. 10 tries to init the magnetometer
        delay(10);
        enableI2CMaster();
        delay(10);
        
        int16_t whoAmI = whoAmIMag();
       if(! ((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))){
           initSuccess = false;
            i2cMasterReset();
            tries++;
            //printf("it is noooooot there!");
        }
        else {
            initSuccess = true;
            //printf("it is there!");
        }
    }
    if(initSuccess){
        // One-time, write the MAG mode via SLV4
        writeAK09916Register8_SLV4(AK09916_CTRL2_ADDR, AK09916_CTRL2_VAL);
        delay(10);

        // Configure SLV0 for continuous read
        enableMagDataRead(AK09916_ST1_ADDR,0x09);
    }
    return initSuccess;
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
        reset_ICM20948();

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
     * @brief Initialise the ICM-20948 for Running state.
     * 
     *       **** Running Initialization Setting 
     *            *** Gyro Range -> ±2000 °/s
     *            *** Acc Range. -> ±16g
     *            *** Sampling Rate For Both -> 1.125 kHz
     *            *** Gyro DLPF setting -> 7: 3DB BW: 361.4 Hz
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
     * @return  0  on success, -1 if WHO_AM_I does not match.
     */
    int runningSetup(){

        /*── 1. Initialise SPI bus and CS pin ───────────────────────*/
        pinMode(ICM_CS, OUTPUT);
        digitalWrite(ICM_CS, HIGH);         // deassert CS
        icmSPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);

        /*── 2. Verify WHO_AM_I ──────────────────────────────────────*/
        selectBank(ICM20948_BANK_0);
        if (spiReadReg(ICM20948_WHO_AM_I) != ICM20948_WHO_AM_I_VAL) return -1;


        /*── 3. Full device reset ────────────────────────────────────*/
        //spiWriteReg(ICM20948_PWR_MGMT_1, ICM20948_RESET);
        //delay(50);                          // wait for reset to complete
        reset_ICM20948();
        /*── 4. Wake up + auto clock (PLL) ──────────────────────────*/
        spiWriteReg(ICM20948_PWR_MGMT_1, ICM20948_CLK_PLL);

        /*── 5. Enable accel + gyro on all axes ─────────────────────*/
        spiWriteReg(ICM20948_PWR_MGMT_2, ICM20948_ACCEL_GYRO_ON);


        /*── 6. Bank 2 — Gyro config ─────────────────────────────────*/
        selectBank(ICM20948_BANK_2);

        // 6a. Gyro ODR — 1.125 kHz
        spiWriteReg(ICM20948_GYRO_SMPLRT_DIV, ICM20948_GYRO_SMPLRT_DIV_VAL);

        // 6b. GYRO_FS_SEL=11 (±2000dps) | FCHOICE=1 | DLPFCFG=111 ( 7 -> 361.4 Hz), Value: 0b00_111_11_1 = 0x3F
        spiWriteReg(ICM20948_GYRO_CONFIG_1, ICM20948_GYRO_CONFIG_1_VAL_RUNNING);

        // 6c. No self-test, no extra averaging
        spiWriteReg(ICM20948_GYRO_CONFIG_2, ICM20948_GYRO_CONFIG_2_VAL);


        /*── 7. Bank 2 — Accel config ────────────────────────────────*/
        // 7a. Accel ODR — 1.125 kHz
        spiWriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, ICM20948_ACCEL_SMPLRT_DIV_1_VAL);
        spiWriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, ICM20948_ACCEL_SMPLRT_DIV_2_VAL);

        // 7b. Accel ±2g, DLPF enabled, 3dB @ 246 Hz, NBW 265 Hz
        spiWriteReg(ICM20948_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_VAL_RUNNING);

        // 7c. No Self test, No extra averaging (DEC3_CFG = 0)
        spiWriteReg(ICM20948_ACCEL_CONFIG_2, ICM20948_ACCEL_CONFIG_2_VAL);

        /*── 8. ODR alignment — sync accel & gyro start times ───────*/
        spiWriteReg(ICM20948_ODR_ALIGN_EN, ICM20948_ODR_ALIGN_EN_VAL);

        /*── 9. Initalize the MAG ───────────────────────*/
        if(!initMagnetometer()){
            //printf("Mag did NOT init!\n");
        }

        /*── 10. Restore Bank 0 for readData() ───────────────────────*/
        selectBank(ICM20948_BANK_0);

        return 0;
    }    

    /**
     * @brief Read accelerometer, temperature and gyroscope data (14 bytes).
     * 
     * @note this read fucntion is made for Allan variance testbench
     * 
     * Single SPI transaction — CS asserted once for all 14 bytes.
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


    /**
     * @brief   This function is made to read all the IMU data from the two chips as one stream,
     *          parse it into its phyical meaning and return it.
     * 
     * @note this read fucntion is made for streaming and sensor fusion
     * 
     * Single SPI transaction — CS asserted once for all 23 bytes.
     * 
     * This function uses the two function (parseIMUData(...), toPhysical(...)) for data translation.
     *
     * @return  IMUPhysical  Pointer to the IMUPhysical struct with the IMU data translated to its physical meaning.
     */
    IMUPhysical readStreamData(){
        IMUDataStream raw{}; 
        icmSPI.beginTransaction(icmSPISettings);
        digitalWrite(ICM_CS, LOW);

        icmSPI.transfer(ICM20948_ACCEL_XOUT_H | 0x80);          // read mode

        icmSPI.transferBytes(nullptr, (uint8_t*)&raw, 23);       // burst 23 bytes into raw

        digitalWrite(ICM_CS, HIGH);
        icmSPI.endTransaction();

        IMUSample sample = parseIMUData(&raw);

        bool isNewMag = first_read ||
                    (sample.mag[0] != last_mag[0]) ||
                    (sample.mag[1] != last_mag[1]) ||
                    (sample.mag[2] != last_mag[2]);


        if (isNewMag) {
            sample.magValid = true;   // overwrite the raw DRDY bit with the real signal
            last_mag[0] = sample.mag[0];
            last_mag[1] = sample.mag[1];
            last_mag[2] = sample.mag[2];
            first_read = false;
        }

        return toPhysical(sample);
    }



    /**
     * @brief Parse IMU Raw data stream into counts with the right endians
     *
     * @param IMUDataStream  Pointer to the IMUDataStream struct to read the data from
     * @return  IMUSample
     */
    IMUSample parseIMUData(const IMUDataStream* raw) {
        IMUSample s{};

        auto be16 = [](uint8_t hi, uint8_t lo) -> int16_t {
            return (int16_t)((hi << 8) | lo);
        };

        s.accel[0] = be16(raw->data[0],  raw->data[1]);
        s.accel[1] = be16(raw->data[2],  raw->data[3]);
        s.accel[2] = be16(raw->data[4],  raw->data[5]);

        s.gyro[0]  = be16(raw->data[6],  raw->data[7]);
        s.gyro[1]  = be16(raw->data[8],  raw->data[9]);
        s.gyro[2]  = be16(raw->data[10], raw->data[11]);

        s.temp     = be16(raw->data[12], raw->data[13]);

        uint8_t st1 = raw->data[14];
        s.magValid    = st1 & 0x01;
        s.magOverrun  = st1 & 0x02;

        s.mag[0] = (int16_t)((raw->data[16] << 8) | raw->data[15]);
        s.mag[1] = (int16_t)((raw->data[18] << 8) | raw->data[17]);
        s.mag[2] = (int16_t)((raw->data[20] << 8) | raw->data[19]);

        uint8_t st2 = raw->data[22];
        s.magOverflow = st2 & 0x08;

        return s;
    }

    /**
     * @brief   Converts IMUSample into meaningfull physical values based on 
     *          the defined ranges macros in the section "Physical Conversion Macro" in the .h file
     * 
     * @param IMUSample  Pointer to the IMUSample struct that needs to be converted
     * @return  IMUPhysical  returns the physical reading of the IMU stored in IMUPhysical struct
     */
    IMUPhysical toPhysical(const IMUSample& s) {
        IMUPhysical p{};

        constexpr float ACCEL_SENS = 9.81f / ACC_LSBs_PER_1_G;                   // LSB to m/s^2
        constexpr float GYRO_SENS  = (3.14159265f/180.0f) /GYRO_LSBs_PER_DPS;    // LSB to rad/s
        constexpr float MAG_SENS   = 0.15f;                                      // µT/LSB, fixed per AK09916 spec

        for (int i = 0; i < 3; i++) {
            p.accel[i] = s.accel[i] * ACCEL_SENS;
            p.gyro[i]  = s.gyro[i]  * GYRO_SENS;
            p.mag[i]   = s.mag[i]   * MAG_SENS;
        }
        p.temp = (s.temp / 333.87f) + 21.0f;                                       // ICM20948 temp formula
        p.magValid = s.magValid;

        return p;
    }

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
    BenchResult benchmarkReadStreamData(int sampleCount) {
        BenchResult result{};
        result.minTime = UINT32_MAX;
        result.maxTime = 0;
        result.samples = sampleCount;

        uint64_t totalTime = 0;

        for (int i = 0; i < sampleCount; i++) {
            uint32_t t0 = micros();
            IMUPhysical p = ICM20948::readStreamData();
            uint32_t dt = micros() - t0;

            if (dt < result.minTime) result.minTime = dt;
            if (dt > result.maxTime) result.maxTime = dt;
            totalTime += dt;

            // Prevent the compiler from optimizing away the call
            if (p.temp < -1000.0f) { totalTime += 0; }
        }

        result.avgTime = (float)totalTime / sampleCount;
        return result;
    }

    /**
     * @brief Prints a BenchResult in a readable format.
     */
    void printBenchResult(const char* label, const BenchResult& r) {
        Serial.printf(
            "[%s] over %d samples: min=%lu us, max=%lu us, avg=%.2f us\n",
            label, r.samples, r.minTime, r.maxTime, r.avgTime
        );
    }

} // namespace ICM20948
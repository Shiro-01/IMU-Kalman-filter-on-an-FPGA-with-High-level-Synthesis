/**
 * @author Abdelrahman Hewala 
 * @supervisor: Prof. Lutz Leutelt
 * @brief This is the implementation of the header file MPU6050.h. Main purpose it to provide functions to 
 * 1) configure the IMU for Becnhmarking where Gyro and Acc are setted to the highest resolution, DLPF is off (Max Bandwidth), and sampling Freq of 1 KHz 
 * 2) configure the IMU for Error modelling at the use case resolution suitable for aircrafts
 * 3) reading the data out of the IMU
 *
 * for More detials about the Module, data sheet Notes and the implementation pls refer to the Module page I made on Notion
 */
#include "MPU6050.h"

/** @brief Initilising the IMU 
 *  The function initilize the IMU for Benchmarking, by setting the Resolution to its Highest Values
 *  ± 250°/s,  ± 2 g and DLPF is off, setting the divider to zero leading to a sampling rate of 8 KHz for the whole system
 *  @note With the current Config where DLPF is off, we have a delay of 1 ms. Also at DLPF is off, Fs of Gyro is 8 KHz, while the Acc is 1 kHz. but as the divider is 7, the output rate will be uniforn of 1 KHz for Both. 
 *  @return 0 on success, -1 if not found
 */
int IMUBenchmarkSetup(){
  // Sanity Check that the module exists
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_WHO_AM_I); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_I2C_ADDR_LOW, 1);
  if(Wire.read() != MPU6050_I2C_ADDR_LOW) {
    return -1;
  }

  // Wake up MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_PWR_MGMT_1);  // Power management register
  Wire.write(0);     // Set to zero (wakes up MPU6050) to awake the Module. MPU is on sleep mode by defaukt once woke up
  Wire.endTransmission(true);

  // For Benchmarking: setting Gyroscope resolution to the highest lvl of sesnitivity ± 250 °/s, 
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_GYRO_CONFIG);  // Gyro Config register
  Wire.write(0);     // Set to zero Bit4 Bit3  FS_SEL[1:0]  = 00 represents  ± 250 °/s Range
  Wire.endTransmission(true);

  // For Benchmarking: setting Acceleration resolution to the highest lvl ± 2 g
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_ACCEL_CONFIG);  // Accelometer register
  Wire.write(0);     // Set to zero Bit4 Bit3  AFS_SEL[1:0]  = 00. represents  ± 2 g Range
  Wire.endTransmission(true);

  // For Benchmarking: setting Digital Low Pass Filter (DLPF) off (Max Band width), we want to look at the snesor Performance without any masks. and turning off external Frame Synchronization (FSYNC)
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_CONFIG);  //  external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF)  register
  Wire.write(0);     // Set to zero No external Frame Synchronization and no filtering and having the larhest bandwidth 260 hz (acc) and 256 hz (gyro) at DLPF_CFG[2:0] 000. 
  Wire.endTransmission(true);

  // For Benchmarking: setting the sampling rate divider to have a unofrom output of 1 KHz for both gyro and acc
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_SMPRT_DIV);  // Accelometer register
  Wire.write(0x07);     // SMPLRT_DIV[7:0] = 7, do divider, we want to get an FS of 1 Khz as discussed on the Requirment page.   FS = Gyro FS /(SMPLRT_DIV + 1)
  Wire.endTransmission(true);

  return 0;
}


/** @brief Read Gyro, temp and Acc data into a struct
 *
 *  @param data is a pointer to the Struct that will holds the IMU data in.
 *  @return Void
 */
 void readIMU(IMUData* data){
  Wire.beginTransmission(MPU6050_I2C_ADDR_LOW);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_I2C_ADDR_LOW, 14, true);

  data->ax = (int16_t) (Wire.read() << 8 | Wire.read());
  data->ay = (int16_t) (Wire.read() << 8 | Wire.read());
  data->az = (int16_t) (Wire.read() << 8 | Wire.read());
  
  data->temp = (int16_t) (Wire.read() << 8 | Wire.read());
  
  data->gx = (int16_t) (Wire.read() << 8 | Wire.read());
  data->gy = (int16_t) (Wire.read() << 8 | Wire.read());
  data->gz = (int16_t) (Wire.read() << 8 | Wire.read());
  }


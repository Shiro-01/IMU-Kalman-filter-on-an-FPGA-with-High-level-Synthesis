/**
 * @author Abdelrahman Hewala 
 * @supervisor: Prof. Lutz Leutelt
 * @brief This is  the header file MPU6050.h. Main purpose it to provide functions to 
 * 1) configure the IMU for Becnhmarking where Gyro and Acc are setted to the highest resolution, DLPF is off (Max Bandwidth), and sampling Freq of 1 KHz 
 * 2) configure the IMU for Error modelling at the use case resolution suitable for aircrafts
 * 3) reading the data out of the IMU
 *
 * for More detials about the Module, data sheet Notes and the implementation pls refer to the Module page I made on Notion
 */
#include <Wire.h>

const int MPU_ADDR = 0x68;  // MPU6050 I2C address

#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

/* --- Device Information --- */
#define MPU6050_I2C_ADDR_LOW   0x68 // AD0 pin low, we are using that address
#define MPU6050_I2C_ADDR_HIGH  0x69 // AD0 pin high
#define MPU6050_WHO_AM_I       0x75 // Should return 0x68

/* --- Configuration Registers --- */
#define MPU6050_SMPRT_DIV      0x19 // Sample Rate Divider
#define MPU6050_CONFIG         0x1A // General Config (DLFP)
#define MPU6050_GYRO_CONFIG    0x1B // Gyro Full Scale Range
#define MPU6050_ACCEL_CONFIG   0x1C // Accel Full Scale Range
#define MPU6050_FIFO_EN        0x23
#define MPU6050_INT_ENABLE     0x38
#define MPU6050_INT_STATUS     0x3A

/* --- Accelerometer Data --- */
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_XOUT_L   0x3C
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_YOUT_L   0x3E
#define MPU6050_ACCEL_ZOUT_H   0x3F
#define MPU6050_ACCEL_ZOUT_L   0x40

/* --- Temperature Data --- */
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_TEMP_OUT_L     0x42

/* --- Gyroscope Data --- */
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_GYRO_XOUT_L    0x44
#define MPU6050_GYRO_YOUT_H    0x45
#define MPU6050_GYRO_YOUT_L    0x46
#define MPU6050_GYRO_ZOUT_H    0x47
#define MPU6050_GYRO_ZOUT_L    0x48

/* --- Power Management --- */
#define MPU6050_PWR_MGMT_1     0x6B // Device Reset, Sleep, Clock Source
// | 7 | DEVICE_RESET | Set to 1 to reset all internal registers to default. |
// | 6 | SLEEP | 1 = Asleep (Default), 0 = Awake. |
// | 5 | CYCLE | Allows the chip to "wake up" periodically to take a sample. |
// | 3 | TEMP_DIS | Set to 1 to disable the internal temperature sensor. |
// | 2:0 | CLKSEL | Selects the clock source (Internal 8MHz or Gyro PLL). |
#define MPU6050_PWR_MGMT_2     0x6C

/** @brief Struct to hold the Read IMU data */
struct IMUData
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};


/** @brief Initilising the IMU 
 *  The function initilize the IMU for Benchmarking, by setting the Resolution to its Highest Values
 *  ± 250°/s,  ± 2 g and DLPF is off, setting the divider to zero leading to a sampling rate of 8 KHz for the whole system
 *  @note With the current Config where DLPF is off, we have a delay of 1 ms. Also at DLPF is off, Fs of Gyro is 8 KHz, while the Acc is 1 kHz, So for one Acc we will have 8 Gyro
 *  @return 0 on success, -1 if not found
 */
int IMUBenchmarkSetup(); 

/** @brief Read Gyro and Acc data
 *
 *  @param data is a pointer to the Struct that will holds the IMU data in
 *  @return Void
 */
 void readIMU(IMUData* data);


#endif

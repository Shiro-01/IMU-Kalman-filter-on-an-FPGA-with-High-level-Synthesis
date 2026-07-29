#include <Wire.h>
#include "ICM20948_SPI.h"
#include "IMUTypes.h"
#include "UART_Streaming.h"

#include "ICM20948_SPI.h"

#define SERIALCONFIG SERIAL_8O1
#define SERIALRATE 115200

void setup() {
    Serial.begin(SERIALRATE, SERIALCONFIG);
    delay(1000);

    int result = ICM20948::runningSetup();
    if (result != 0) {
        Serial.println("ICM20948 init FAILED — check wiring/CS/WHO_AM_I");
        while (1) delay(1000);
    }
    Serial.println("ICM20948 init OK");
}

void loop() {
    IMUPhysical imu = ICM20948::readStreamData();

    // IMUPhysical p = ICM20948::readStreamData();
    // Serial.printf(
    //     "accel[%.3f, %.3f, %.3f] gyro[%.4f, %.4f, %.4f] mag[%.2f, %.2f, %.2f] valid=%d temp=%.1f\n",
    //     p.accel[0], p.accel[1], p.accel[2],
    //     p.gyro[0],  p.gyro[1],  p.gyro[2],
    //     p.mag[0],   p.mag[1],   p.mag[2],
    //     p.magValid, p.temp
    // );


    // BenchResult r = ICM20948::benchmarkReadStreamData(1000);    // 1000 sample
    // ICM20948::printBenchResult("readStreamData", r);
    // delay(1000);   // slow print rate for readability — real loop will run much faster

    sendIMUPacket(
        imu.accel[0], imu.accel[1], imu.accel[2],
        imu.gyro[0], imu.gyro[1], imu.gyro[2],
        imu.mag[0], imu.mag[1], imu.mag[2],
        0, 0, 0,
        0, 0, 0,
        imu.magValid
    );
    delay(1000);
}



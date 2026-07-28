#include <Wire.h>
#include "ICM20948_SPI.h"
#include "IMUTypes.h"


#include "ICM20948_SPI.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    int result = ICM20948::runningSetup();
    if (result != 0) {
        Serial.println("ICM20948 init FAILED — check wiring/CS/WHO_AM_I");
        while (1) delay(1000);
    }
    Serial.println("ICM20948 init OK");
}

void loop() {
    // IMUPhysical p = ICM20948::readStreamData();
    // Serial.printf(
    //     "accel[%.3f, %.3f, %.3f] gyro[%.4f, %.4f, %.4f] mag[%.2f, %.2f, %.2f] valid=%d temp=%.1f\n",
    //     p.accel[0], p.accel[1], p.accel[2],
    //     p.gyro[0],  p.gyro[1],  p.gyro[2],
    //     p.mag[0],   p.mag[1],   p.mag[2],
    //     p.magValid, p.temp
    // );


    BenchResult r = ICM20948::benchmarkReadStreamData(1000);    // 1000 sample
    ICM20948::printBenchResult("readStreamData", r);
    delay(1000);   // slow print rate for readability — real loop will run much faster
}



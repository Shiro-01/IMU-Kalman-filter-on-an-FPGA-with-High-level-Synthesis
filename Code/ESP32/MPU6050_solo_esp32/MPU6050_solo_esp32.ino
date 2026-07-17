#include "MPU6050.h"

/*Global Parameters*/
int re;

/*Functions Declerations*/
void printIMU(const IMUData* data);

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA = 21, SCL = 22 

  re = IMUBenchmarkSetup(); // setting the IMU with Benchmarking Parameters
  if(re == 0){
    Serial.println("MPU6050 Initialized For Benchmarking");
  } else {
    Serial.println("MPU6050 not found");
  }
}

void loop() {
  IMUData sample;
  readIMU(&sample);
  WriteToSD(&sample);

  delay(1000);
}



/** Debugging Functions */
/** @brief ptinting one sample to the console 
 *
 *  @param data is a pointer to the Struct that holds the read IMU data
 *  @return Void
 */
void printIMU(const IMUData* data) {
  Serial.print("ACC [");
  Serial.print(data->ax); Serial.print(", ");
  Serial.print(data->ay); Serial.print(", ");
  Serial.print(data->az); Serial.print("]  ");

  Serial.print("TEMP: ");
  Serial.print(data->temp);
  Serial.print("  ");

  Serial.print("GYRO [");
  Serial.print(data->gx); Serial.print(", ");
  Serial.print(data->gy); Serial.print(", ");
  Serial.print(data->gz); Serial.println("]");
}
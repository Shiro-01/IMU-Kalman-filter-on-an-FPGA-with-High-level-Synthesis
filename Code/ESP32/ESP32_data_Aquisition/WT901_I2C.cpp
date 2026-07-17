
#include "WT901_I2C.h"
#include "string.h"
//#include <cstdio>
#include <Arduino.h>  // Make sure to include this for Serial functions



CJY901 ::CJY901 ()
{
	ucDevAddr =0x50;
}
void CJY901::StartIIC()
{
	ucDevAddr = 0x50;
	Wire.begin();

}
void CJY901::StartIIC(unsigned char ucAddr)
{
	ucDevAddr = ucAddr;
	Wire.begin();
  Wire.setClock(400000);
}

void CJY901::readRegisters(unsigned char deviceAddr, unsigned char addressToRead, unsigned char bytesToRead, char * dest)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); // endTransmission but keep the connection active

  Wire.requestFrom(deviceAddr, bytesToRead); // Ask for bytes, once done, bus is released by default

  unsigned long startTime = millis();
  while(Wire.available() < bytesToRead) {
    if (millis() - startTime > 50) { 
      Serial.println("I2C Timeout, resetting Wire bus...");
      Wire.begin();    // Restart I2C
      return;          // Exit function early
    }
  }

  for(int x = 0; x < bytesToRead; x++)
    dest[x] = Wire.read();    
}

void CJY901::writeRegister(unsigned char deviceAddr, unsigned char addressToWrite, unsigned char bytesToRead, char *dataToWrite)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);

  for(int i = 0; i < bytesToRead; i++) {
    Wire.write(dataToWrite[i]);
  }

  uint8_t result = Wire.endTransmission(); // Returns 0 if successful

  if (result != 0) { // If transmission failed
    Serial.println("I2C Write Error, resetting Wire bus...");
    Wire.begin();    // Restart I2C
  }
}


short CJY901::ReadWord(unsigned char ucAddr)
{
	short sResult;
	readRegisters(ucDevAddr, ucAddr, 2, (char *)&sResult);
	return sResult;
}
void CJY901::WriteWord(unsigned char ucAddr,short sData)
{	
	writeRegister(ucDevAddr, ucAddr, 2, (char *)&sData);
}
void CJY901::ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[])
{
	readRegisters(ucDevAddr, ucAddr, ucLength, chrData);
}

void CJY901::updateTime()
{
	readRegisters(ucDevAddr, 0x30, 8, (char*)&stcTime);	
}


void CJY901::updateAcc()
{
	readRegisters(ucDevAddr, AX, 6, (char *)&stcAcc);

    for (int i = 0; i < 3; i++) {
        // Convert the raw short values to float and store them in the af array
        stcAcc.af[i] = (float)stcAcc.a[i] / 32768.0f * 16.0f;  // 16g range
    }
}


void CJY901::updateGyro()
{
	readRegisters(ucDevAddr, GX, 6, (char *)&stcGyro);

   for (int i = 0; i < 3; i++) {
        // Convert the raw short values to float and store them in the gf array
        stcGyro.gf[i] = (float)stcGyro.g[i] / 32768.0f * 2000.0f; // 2000 deg/sec range
    }
}


void CJY901::updateAngle()
{
	readRegisters(ucDevAddr, Roll, 6, (char *)&stcAngle);

  for (int i = 0; i < 3; i++) {
        // Convert the raw short values to float and store them in the Anglef array
        stcAngle.Anglef[i] = (float)stcAngle.Angle[i] / 32768.0f * 180.0f; // 180 deg range
    }
}


void CJY901::updateMag()
{
	readRegisters(ucDevAddr, HX, 6, (char *)&stcMag);
}


void CJY901::updatePress()
{
	readRegisters(ucDevAddr, PressureL, 8, (char *)&stcPress);
}
void CJY901::updateDStatus()
{
	readRegisters(ucDevAddr, D0Status, 8, (char *)&stcDStatus);
}
void CJY901::updateLonLat()
{
	readRegisters(ucDevAddr, LonL, 8, (char *)&stcLonLat);
}
void CJY901::updateGPSV()
{
	readRegisters(ucDevAddr, GPSHeight, 8, (char *)&stcGPSV);
}

void CJY901::updateAll()
{
    // Update Accelerometer data
    updateAcc();

    // Update Gyroscope data
    updateGyro();

    // Update Angle data
    updateAngle();

    //
    updateMag();
}

// Function to print the magnetometer data on the Serial Monitor
void printMagData(const SMag &mag)
{
    Serial.println("Magnetometer Data:");
    Serial.print("X: ");
    Serial.print(mag.h[0]);
    Serial.print(", Y: ");
    Serial.print(mag.h[1]);
    Serial.print(", Z: ");
    Serial.println(mag.h[2]);

}

// Function to print the accelerometer data on the Serial Monitor
void printAccData(const SAcc &acc)
{
    Serial.println("Acceleration Data:");
    Serial.print("X: ");
    Serial.print(acc.af[0]);
    Serial.print(", Y: ");
    Serial.print(acc.af[1]);
    Serial.print(", Z: ");
    Serial.println(acc.af[2]);
}

// Function to print the gyroscope data on the Serial Monitor
void printGyroData(const SGyro &gyro)
{
    Serial.println("Gyroscope Data:");
    Serial.print("X: ");
    Serial.print(gyro.gf[0]);
    Serial.print(", Y: ");
    Serial.print(gyro.gf[1]);
    Serial.print(", Z: ");
    Serial.println(gyro.gf[2]);
}

// Function to print the angle data on the Serial Monitor
void printAngleData(const SAngle &angle)
{
    Serial.println("Angle Data:");
    Serial.print("X: ");
    Serial.print(angle.Anglef[0]);
    Serial.print(", Y: ");
    Serial.print(angle.Anglef[1]);
    Serial.print(", Z: ");
    Serial.println(angle.Anglef[2]);
}

// Function to print all sensor data together (including magnetometer) on the Serial Monitor
void CJY901::printAllData()
{
    Serial.println("---- All Sensor Data ----");

    // Print Acceleration data
    printAccData(stcAcc);

    // Print Gyroscope data
    printGyroData(stcGyro);

    // Print Angle data
    printAngleData(stcAngle);

    // Print Magnetometer data
    printMagData(stcMag);

    Serial.println("-------------------------");
}

void CJY901::Config()
{
  // factory reset
  //reset();

  unlockChip(); /// UNLOCKING THE CHIP TO ENABLE THE WRITE COMMANDS
  // note the chip is litle indian meaning lsbyte sent first
  
  //----seting the oriantation -----------------
  char writeData[2] = {0x00, 0x00};  // setting to Horizontal orientation (0x0000) 
  writeRegister(ucDevAddr, ORIENT, 2, writeData);  // Set the device orientation to horizontal 0x0000 for hz, 0x0001 for vl.
  
  //--setting the algo to be 6 axis - NO USE OF THE MAGNETOMETER
  //- 9 axis mode 0x0000, for 6 axis 0x0001)
  char axis6Data[2] = {0x01, 0x00}; 
  writeRegister(ucDevAddr, AXIS6, 2, axis6Data);  
  
  save(); // savet the changing aaaccording to the write protocol 

  // CHECKING WRITING RESULTS 
  checkOrientation();
  checkAxis6Mode();

  //--- accelration ranage is being updated automatically! between 2g to 16 g
  /*This parameter cannot be set. The product's
  internal adaptive acceleration range will
  automatically switch to 16g when the acceleration
  exceeds 2g.*/

  //  gyro range 0011(0x03): 2000°/s - fixed and cannot be set

}

void CJY901::calibrate()
{    

  // NO NEED TO DO THE CALIBIRATION ACCELERSTION AS IT LEAD TO SESITIVITIZ LOSS/ BEST PRACTICE LEAVE IT ALONE
  //     // // Step 1: Set the value to 0x01 for automatic accelerometer calibration
  //     char calData1[2] = {0x01, 0x00};
  //     writeRegister(ucDevAddr, CALSW, 2, calData1);
  //     Serial.println("Step 1: Automatic accelerometer calibration started.");


  // Step 2: Set the value to 0x07 for Magnetic Field Calibration (Spherical Fitting) -- THIS IS THE RECOMMENDED MAGNETIC CALIB BZ THE PREDUCER 
  // char calData2[2] = {0x09, 0x00};
  // writeRegister(ucDevAddr, CALSW, 2, calData2);
  // Serial.println("Step 2: Magnetic Field Calibration (Spherical Fitting) started.");


  // Step 3: Set the value to 0x04 to set the heading angle to zero 
  // NOT REQUIRED IN 9 AXIS MODE - SO COMMENT IT IF U PLAINING TO USE THE MAGNETOMETER
  unlockChip();
  char setHeadingAng[2] = {0x04, 0x00};
  writeRegister(ucDevAddr, CALSW, 2, setHeadingAng);
  Serial.println("Setting heading angle to zero.");

  // Step 4: Set the value to 0x08 to set the angle reference
  char calData4[2] = {0x08, 0x00};
  writeRegister(ucDevAddr, CALSW, 2, calData4);
  Serial.println("Setting the angle reference... and we are nearly done!");

  // Final Step: Set the value back to 0x00 for normal working mode
  // char calData5[2] = {0x00, 0x00};
  // writeRegister(ucDevAddr, CALSW, 2, calData5);
  // Serial.println("Calibration is done successfully.");
  save();


}

void CJY901::checkOrientation()
{

    unsigned char buffer[2]; 
    readRegisters(ucDevAddr, ORIENT, 2, (char *)buffer);
    
    // Check if the value in the register is 0x0000 (horizontal orientation)
    if (buffer[0] == 0x00 && buffer[1] == 0x00) {
      Serial.println("Orientation is set to Horizontal.");

    } 
    else if (buffer[0] == 0x01 && buffer[1] == 0x00) {
        Serial.println("Orientation is set to Vertical.");
    }
    else {
      Serial.println("Orientation isUnknown: ");
      Serial.println(buffer[0]);
      Serial.println(buffer[1]);
    }
    delay(2000);  // Wait for 2 seconds
  

}

void CJY901::checkAxis6Mode()
{
    char readData[2];
    readRegisters(ucDevAddr, AXIS6, 2, readData);

    if (readData[0] == 0x01 && readData[1] == 0x00) {
      // Print confirmation if the mode is set to 9-axis
      Serial.println("algorithm is set to 6-axis mode successfully!");
    } 
    else if (readData[0] == 0x00 && readData[1] == 0x00) {
      // Print an error if the value is not as expected
      Serial.println("algorithm is set to 9-axis mode.");
    }
    else{
      Serial.println("algorithm code is unkwon:");
      Serial.println(readData[0]);
      Serial.println(readData[1]);

    }

}



void CJY901::unlockChip()
{
    // Step 1: Unlock the chip by writing to the register ++**-- litle indian--**++
    char unlockCode[2] = { 0x88, 0xB5 };
    writeRegister(ucDevAddr, LOCK, 2, unlockCode);  // 0x88 is the unlock register address


    // char checkData3[2];  // Array to hold the read value
    // readRegisters(ucDevAddr, LOCK, 2, checkData3);  // Read the register value 
    //      Serial.println((int) checkData3[0]);
    //      Serial.println((int) checkData3[1]);


    
}

void CJY901::save()
{
  // saving the changes
  char savingCode[2] = {0x00, 0x00}; 
  writeRegister(ucDevAddr, SAVE, 2, savingCode);  // 0x88 is the unlock register address

}


void CJY901::reset()
{
  unlockChip();

  // factory reset
  char resetCode[2] = {0x01, 0x00}; 
  writeRegister(ucDevAddr, SAVE, 2, resetCode);  // Factory reset: 0x0001

  save();
}

void CJY901::lockChip()
{
    // Step 1: Unlock the chip by writing to the register ++**-- litle indian--**++
    char lockCode[2] = {0x00, 0x00};
    writeRegister(ucDevAddr, LOCK, 2, lockCode);  // 0x88 is the unlock register address

    // saving the changes
        char savingCode[2] = {0x00, 0x00}; 
        writeRegister(ucDevAddr, SAVE, 2, savingCode);  // 0x88 is the unlock register address

    delay(100);  // Wait briefly before proceeding
}


CJY901 JY901 = CJY901();

//----------------------------------------------------------/
  // // tEMPORARY REST
  //       char resetCode[2] = {0x01, 0x00}; 
  //       writeRegister(ucDevAddr, SAVE, 2, resetCode);  
  //       delay(5000);

      // char checkData3[2];  // Array to hold the read value
    // readRegisters(ucDevAddr, BATVAL, 2, checkData3);  // Read the register value 
    //          Serial.println("fddbgdbbb"); 

    //      Serial.println((int) checkData3[0]);
    //      Serial.println((int) checkData3[1]); 


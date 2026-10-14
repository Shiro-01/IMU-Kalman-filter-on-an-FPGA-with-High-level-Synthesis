#include <Wire.h>
#include "SdFat.h"
#include "driver/timer.h"
#include "MPU6050.h"
#include "ICM20948_SPI.h"
#include "IMUTypes.h"
#include "WT901.h"

// --- Configuration --- //
#define FS 1000             // 1 KHz
#define BUFFER_FRAME_COUNT 100
#define TOTAL_BUFFERS 3
#define FILE_SIZE_2H   360000000LL//1080000000//540000000//30000000// //33750000//// 332000000LL   //1073741824L
#define SPI_SPEED SD_SCK_MHZ(10)
#define SD_CS_PIN 5

struct __attribute__((packed)) FullFrame {
  // NOTE: Timestamp is taken before I²C reads. At 1MHz I²C each read takes
  // ~112µs, so imu3 data is captured ~224µs after the recorded timestamp.
  // Acceptable for Allan Variance as all sensors are offset equally per frame.
  uint64_t timestamp;
  IMUData imu1;
  IMUData imu2;
  IMUData imu3;
}; // 50 Bytes in total 

// --- Constants --- //
const size_t BUFFER_SIZE = sizeof(FullFrame) * BUFFER_FRAME_COUNT; // 5000 Bytes

// --- Global Parameter --- //
// --- FreeRTOS Parameters --- //
static QueueHandle_t buffersQueue;
static TaskHandle_t collectorTask;
static TaskHandle_t loggerTask;
static SemaphoreHandle_t bufferSemaphore;

uint32_t totalFramesCollected = 0;
const uint32_t MAX_FRAMES = 7200000;//21600000; //600000; //;
// --- Timer Parameter --- //
hw_timer_t *timer = NULL;

// --- SD Parameters --- //
SdFat sd;
SdFile sdFile;

// --- Frames Buffers --- //
FullFrame buffers[TOTAL_BUFFERS][BUFFER_FRAME_COUNT];
int status = 0;
// ----------------------- ISRs, Tasks & helpers -----------------------  //
// --- Timer Interupt ISR --- //
void IRAM_ATTR onTimer(){
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(collectorTask, &woken);
  portYIELD_FROM_ISR(woken);    // tells the Shedualr to refresh and start the new highest priotity task, ofcourse if woken was true. other wise, it will just continue the previous where it got interuppted 
} 

// --- Read IMUs Function --- //
void readAllIMUs(FullFrame* frame){
  frame -> timestamp = micros();
  MPU6050::readData(&(frame->imu1));
  //MPU6050::readData(&(frame->imu2));
  //MPU6050::readData(&(frame->imu3));
  ICM20948::readData(&(frame->imu2));
  WT901::readData(&(frame->imu3));
}


// --- Collector Task --- // 
void CollectorTask (void* parameters){
  int currentBufferIdx = 0;
  int frameIdx = 0;
  bool bufferReady = false;

  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (frameIdx == 0) { // new buffer should start
      if(xSemaphoreTake(bufferSemaphore, 0) != pdTRUE){ // no delay, the task will stay awake and print the error
        printf("the 3 Buffers are full, logger is slower than the collector \n");
        bufferReady = false;
        // Program will end
      }  else {
        bufferReady = true;
      }
    }

    if(bufferReady == true){
      readAllIMUs(&(buffers[currentBufferIdx][frameIdx]));
      totalFramesCollected++;

      // sending the buffer when is full
      if (frameIdx == BUFFER_FRAME_COUNT - 1){
        FullFrame* BufferStartPtr = &(buffers[currentBufferIdx][0]);
        xQueueSend(buffersQueue, &BufferStartPtr, 0);  //--> this was the problem
      }

      // decideing the next location logic
      if (frameIdx < BUFFER_FRAME_COUNT - 1){
        frameIdx++;
      } else {
        if(currentBufferIdx < TOTAL_BUFFERS - 1){
          currentBufferIdx ++;
          frameIdx = 0;
        } else {
          currentBufferIdx = 0;
          frameIdx = 0;
        }
      }
    }

    if ((totalFramesCollected >= MAX_FRAMES) || !bufferReady) {
      timerStop(timer); // Stop the 1kHz clock immediately
      
      // Send a NULL pointer to signify the END of the test
      FullFrame* endSignal = NULL;
      xQueueSend(buffersQueue, &endSignal, portMAX_DELAY);
      
      Serial.println("End signal sent to Logger.");
      vTaskDelete(NULL); // Collector is now safe to die
  }
  }
}

// --- Logger Task (Consumer) ---
void LoggerTask(void* p) {
  FullFrame* BufferStartPtr; // Receive the index as an int

  while (1) {
    // Wait for the Collector to send a buffer index
    if (xQueueReceive(buffersQueue, &BufferStartPtr, portMAX_DELAY)) {

      // CHECK If Is this the kill signal?
      if (BufferStartPtr == NULL) {
        Serial.println("Logger received end signal. Closing file...\n");
        sdFile.close(); 
        Serial.println("SD Card safe. Task exiting.\n");
        vTaskDelete(NULL); // Logger closes the file and then dies
      }
      // Write the specific 4600-byte row to SD
      size_t written = sdFile.write(BufferStartPtr, BUFFER_SIZE);
      
      if (written != BUFFER_SIZE) {
        Serial.println("SD Write Error!");
      }

      // Return the "permit" so the Collector can use this row again
      xSemaphoreGive(bufferSemaphore);
    }
  }
}

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  Wire.begin();  // SDA = 21, SCL = 22 (default ESP32)
  Wire.setClock(800000);

  status = MPU6050::benchmarkSetup(); // setting the IMU with Benchmarking Parameters
  if(status == 0){
    Serial.println("MPU6050 Initialized For Benchmarking\n");
  } else {
    Serial.println("MPU6050 not found\n");
  }

  status = ICM20948::benchmarkSetup(); // setting the IMU with Benchmarking Parameters
  if(status == 0){
    Serial.println("ICM20948 Initialized For Benchmarking\n");
  } else {
    Serial.println("ICM20948 not found\n");
  }

  status = WT901::benchmarkSetup(); // setting the IMU with Benchmarking Parameters
  if(status == 0){
    Serial.println("WT901 Initialized For Benchmarking\n");
  } else {
    Serial.println("WT901 not found\n");
  }
  // 1. Setup Buffers & IPC
  buffersQueue = xQueueCreate(TOTAL_BUFFERS + 1, sizeof(FullFrame*));  // extra place for the kill signal
  bufferSemaphore = xSemaphoreCreateCounting(TOTAL_BUFFERS, TOTAL_BUFFERS);

  // 2. Setup SD Card with Pre-allocation
  if (!sd.begin(SD_CS_PIN, SPI_SPEED)) {
    Serial.println("SD Init Failed!");
    while(1);
  }

  if (!sdFile.open("DATA35_2H_F_0.BIN", O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("File Open Failed!\n");
    while(1);
  }

  Serial.println("Pre-allocating 3 hours fule size... please wait.\n");
   if (!sdFile.preAllocate(FILE_SIZE_2H)) {
     Serial.println("Pre-allocation Failed!\n");
  }
  Serial.println("Ready.\n");

  // 3. Creating Tasks
  xTaskCreatePinnedToCore(CollectorTask, "Collector", 8192, NULL, 10, &collectorTask, 1);
  xTaskCreatePinnedToCore(LoggerTask, "Logger", 8192, NULL, 5, &loggerTask, 0);

  // 4. Hardware Timer (1 KHz)
  timer = timerBegin(1000000);    // setting clock freq 1 MHz
  timerAttachInterrupt(timer, &onTimer);
  // Set the alarm to trigger every 1000 ticks (1000 ticks * 1us = 1ms = 1 KHz)
  // Parameters: (timer, alarm_value, autoreload, reload_count)
  // reload_count = 0 means it repeats forever
  timerAlarm(timer, 1000, true, 0);
}

void loop() {
// Core 1 and Core 0 are busy. Main loop does nothing.
  vTaskDelay(portMAX_DELAY);
}



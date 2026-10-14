# Firmware

Microcontroller firmware for the project. Everything here targets the ESP32
and is used in the first project phase: benchmarking candidate IMUs before
committing to one for the FPGA design. The FPGA design itself lives in
[`FPGA/`](../FPGA).

---

## Sketches

| Sketch | Purpose |
|---|---|
| [`esp32/imu_benchmark_logger/`](esp32/imu_benchmark_logger) | Reads the MPU6050, ICM-20948 and WT901 in parallel at 1 kHz and logs raw frames to SD card for Allan variance benchmarking |
| [`esp32/icm20948_uart_stream/`](esp32/icm20948_uart_stream) | Reads the ICM-20948 (accel, gyro, magnetometer) in physical units and streams it to a PC over UART |
| [`esp32/sd_card_bench/`](esp32/sd_card_bench) | SD card write benchmark, used to check the card can keep up with the logger |

Each sketch is self-contained: the `.ino` file and the drivers it needs sit in
the same folder, as the Arduino IDE requires. `ICM20948_SPI` and `IMUTypes.h`
exist in two sketches but are **not** identical. The streaming copy is newer
and adds magnetometer setup.

---

## Toolchain

| Item | Version used |
|---|---|
| Board | ESP32 dev board (classic ESP32, run at 240 MHz) |
| Arduino-ESP32 core | 3.3.x (the timer API used needs core **3.x**) |
| [SdFat](https://github.com/greiman/SdFat) library | 2.3.0 (logger and SD benchmark only) |

Install the ESP32 core through the Arduino IDE Boards Manager (`esp32` by
Espressif) and SdFat through the Library Manager.

### Build and flash

**Arduino IDE:** open the sketch's `.ino` file, select your ESP32 board and
port, and click Upload.

**arduino-cli:**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32/imu_benchmark_logger
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/tty.usbserial-XXXX firmware/esp32/imu_benchmark_logger
```

Serial monitor: 115200 baud for all sketches.

---

## Wiring Overview

| Device | Bus | ESP32 pins |
|---|---|---|
| ICM-20948 | SPI (HSPI), 6 MHz | CS 32, SCK 25, MISO 26, MOSI 27 |
| MPU6050 | I²C, 800 kHz | SDA 21, SCL 22 (address 0x68) |
| WT901 | I²C, 800 kHz | SDA 21, SCL 22 (address 0x50) |
| SD card | SPI (VSPI), 10 MHz | CS 5, default VSPI pins |

Pin numbers are set in the drivers (`ICM20948_SPI.h`) and sketches
(`SD_CS_PIN`). Change them there if your wiring differs.

---

## Related Host Tools

- SD logs from `imu_benchmark_logger` are analysed with the Streamlit dashboard
  in [`host/esp32_benchmark/`](../host/esp32_benchmark).

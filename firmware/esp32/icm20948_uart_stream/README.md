# ICM-20948 UART Stream

Reads the ICM-20948 over SPI, including the AK09916 magnetometer through the
ICM's internal I²C master, converts the readings to physical units and sends
them to a PC over UART as framed binary packets.

This was the ESP32 prototype of the streaming path, before the acquisition
moved to the FPGA.

---

## What It Does

1. `ICM20948::runningSetup()` configures the sensor for running mode (±16 g,
   ±2000 dps, AK09916 magnetometer at 100 Hz). If the sensor
   doesn't respond, the sketch prints an error and halts.
2. Every loop, `ICM20948::readStreamData()` reads accel, gyro, temperature and
   magnetometer, and returns them as `IMUPhysical`:

   | Field | Unit |
   |---|---|
   | `accel[3]` | m/s² |
   | `gyro[3]` | rad/s |
   | `mag[3]` | µT |
   | `temp` | °C |
   | `magValid` | fresh magnetometer sample |

3. `sendIMUPacket()` (`UART_Streaming.h/.cpp`) frames the sample and writes it
   to the serial port.

> **Current state:** the loop sends **one packet per second** (`delay(1000)`),
> and the six state-vector fields are sent as `0`. No filter runs on the ESP32.

---

## Packet Format

Serial: 115200 baud, 8N1.

| Field | Size | Content |
|---|---|---|
| SYNC | 2 bytes | `0xAA 0x55` |
| TYPE | 1 byte | `0x01` (IMU + state) |
| LEN | 1 byte | `61` (payload length) |
| PAYLOAD | 61 bytes | 15 × `float32` + 1 × `bool` (little-endian, `<15f?`) |
| CHK | 1 byte | XOR of TYPE, LEN and every payload byte |

Payload order:
`ax, ay, az, gx, gy, gz, mx, my, mz, thetaX, thetaY, thetaZ, gyroBx, gyroBy, gyroBz, magValid`

> This is **not** the FPGA frame format. The FPGA uses 16-bit words, a
> `MODE/LEN` word and a 16-bit checksum. `host/imu_pipeline/uart_reader.py`
> decodes only the FPGA format. The Python reader for this ESP32 format
> (`UART_Stream_reader.py`) is no longer in the working tree; it is in git
> history under `Code/PCSetup/StreamingSetup/`.

---

## Wiring

| Device | ESP32 pins |
|---|---|
| ICM-20948 (SPI, HSPI, 6 MHz) | CS 32, SCK 25, MISO 26, MOSI 27 |

The PC connection is the board's USB serial port.

---

## Files

| File | Content |
|---|---|
| `icm20948_uart_stream.ino` | Setup and stream loop |
| `ICM20948_SPI.h/.cpp` | ICM-20948 + AK09916 driver, register map, physical-unit conversion, read timing benchmark |
| `IMUTypes.h` | `IMUData`, `IMUDataStream`, `IMUSample`, `IMUPhysical`, `BenchResult` |
| `UART_Streaming.h/.cpp` | Packet framing and checksum |

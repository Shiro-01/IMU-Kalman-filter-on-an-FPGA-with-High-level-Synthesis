# IMU Benchmark Logger

Logs three IMUs simultaneously to an SD card at 1 kHz, so they can be compared
by Allan variance under identical conditions. This is the data source for the
IMU selection in the first project phase.

| Slot | Sensor | Driver | Bus |
|---|---|---|---|
| `imu1` | MPU6050 | `MPU6050.h/.cpp` | I²C |
| `imu2` | ICM-20948 | `ICM20948_SPI.h/.cpp` | SPI (HSPI) |
| `imu3` | WitMotion WT901 | `WT901.h/.cpp` | I²C |

Each driver's `benchmarkSetup()` configures its sensor for benchmarking before
logging starts.

---

## How It Works

The sketch uses a FreeRTOS producer/consumer pair, so sampling stays on schedule
while the SD card is being written:

```
1 kHz hardware timer ──ISR──▶ CollectorTask (core 1, prio 10)
                                 reads all 3 IMUs into one frame
                                 fills 3 rotating buffers × 100 frames
                                          │ queue (buffer pointer)
                                          ▼
                              LoggerTask (core 0, prio 5)
                                 writes each full buffer (5000 bytes) to SD
                                 returns the buffer via a counting semaphore
```

- **Triple buffering:** the collector only fails if all three buffers are full,
  meaning the SD card fell behind. It then prints an error and stops.
- **Pre-allocation:** the output file is pre-allocated (`FILE_SIZE_2H`) so the SD
  card doesn't allocate clusters mid-run, which causes write-latency spikes.
- **Clean stop:** after `MAX_FRAMES` frames (7,200,000 = 2 h at 1 kHz), the
  collector stops the timer and sends a NULL pointer, and the logger closes the
  file safely.

---

## Output Format

Binary file of back-to-back 50-byte packed frames (little-endian):

| Offset | Size | Field |
|---|---|---|
| 0 | 8 | `timestamp`: `micros()` at the start of the read (`uint64`) |
| 8 | 14 | `imu1`: 7 × `int16` |
| 22 | 14 | `imu2`: 7 × `int16` |
| 36 | 14 | `imu3`: 7 × `int16` |

Each IMU block is raw register counts in the order
`accel X, accel Y, accel Z, temperature, gyro X, gyro Y, gyro Z` (`IMUData` in
`IMUTypes.h`).

The timestamp is taken before the reads, so later sensors in the frame are
sampled slightly after it. That offset is the same in every frame, so it does
not affect the Allan variance.

---

## Configuration

Set at the top of `imu_benchmark_logger.ino`:

| Constant | Value | Meaning |
|---|---|---|
| `FS` | 1000 | Sample rate (Hz); the timer alarm is set to match |
| `BUFFER_FRAME_COUNT` | 100 | Frames per buffer |
| `TOTAL_BUFFERS` | 3 | Number of rotating buffers |
| `MAX_FRAMES` | 7,200,000 | Frames to record before stopping |
| `FILE_SIZE_2H` | 360,000,000 | Bytes pre-allocated on the card |
| `SD_CS_PIN` | 5 | SD card chip select |
| `SPI_SPEED` | 10 MHz | SD card SPI clock |

The output file name is hard-coded in `setup()` (`sdFile.open("DATA35_2H_F_0.BIN", ...)`).
Change it before each run, or the previous recording is overwritten (`O_TRUNC`).

---

## Wiring

| Device | ESP32 pins |
|---|---|
| ICM-20948 (SPI) | CS 32, SCK 25, MISO 26, MOSI 27 |
| MPU6050, WT901 (I²C, 800 kHz) | SDA 21, SCL 22 |
| SD card (SPI) | CS 5, default VSPI pins |

---

## Analysis

Copy the `.BIN` file from the card and open it in the dashboard in
[`host/esp32_benchmark/`](../../../host/esp32_benchmark):

```bash
cd host/esp32_benchmark/analyzer_dashboard
streamlit run dashboard.py
```

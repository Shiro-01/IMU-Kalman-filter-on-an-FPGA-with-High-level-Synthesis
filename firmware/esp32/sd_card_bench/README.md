# SD Card Benchmark

Measures SD card write throughput and latency on the ESP32. It was used to check
that a card can keep up with
[`imu_benchmark_logger`](../imu_benchmark_logger) before starting long
recordings.

Based on the `bench` example from Bill Greiman's
[SdFat](https://github.com/greiman/SdFat) library (MIT license), with the
settings changed to match the logger's workload.

---

## Changes From the SdFat Example

| Setting | SdFat example | This sketch | Why |
|---|---|---|---|
| `SD_CS_PIN` | `SS` | `5` | Same CS pin as the logger |
| `SPI_CLOCK` | 50 MHz | 10 MHz | Same SD clock as the logger |
| `BUF_SIZE` | 512 bytes | 46 bytes | Small writes, close to one logger frame |
| `FILE_SIZE_MB` | 5 | 1024 | Long run, to expose latency spikes |
| `WRITE_COUNT` | 2 | 4 | More write passes |
| `READ_COUNT` | 2 | 0 | Write performance only |
| Serial baud | 9600 | 115200 | Same as the other sketches |

`PRE_ALLOCATE` stays `true`, as in the logger.

---

## Usage

1. Insert a freshly formatted card (FAT32 or exFAT) and wire it to the ESP32:
   CS 5, default VSPI pins.
2. Flash the sketch and open the serial monitor at 115200 baud.
3. Type any character to start.

The sketch prints the card's details (manufacturer, size, format), then each
write pass's throughput (KB/s) and minimum, maximum and average write latency
(µs).

**How to read it:** the logger writes 5000 bytes every 100 ms. The maximum
latency must stay well below what the logger's three buffers can absorb
(about 200 ms), or frames are lost.

> The benchmark creates `bench.dat` on the card and overwrites it on each run.

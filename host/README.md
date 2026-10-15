# Host Software

PC-side Python tools for the IMU Kalman filter on FPGA project. Everything
here runs on the host computer: it receives the UART stream from the FPGA,
decodes and converts it to physical units, runs the reference software EKF,
visualizes hardware vs software filter state, and provides the calibration
and benchmarking tools used to characterize the sensors noisees.

---

## Directory Structure

```
host/
├── imu_pipeline/            Core pipeline: UART decode → units → EKF
│   ├── uart_reader.py       Frame sync, checksum, mode dispatch
│   ├── frame_types.py       Raw and physical sample types (MODE_0, MODE_1)
│   ├── sample_converter.py  Raw register counts → physical units
│   ├── data_preparer.py     dt, hard/soft-iron correction, filter initialization
│   ├── ekf.py               Software reference EKF
│   ├── app_mode0.py         CLI: raw IMU stream (MODE_0) → software EKF
│   └── app_mode1.py         CLI: hardware EKF (MODE_1) vs software EKF
├── gui/                     Real-time PySide6 dashboard (see gui/README.md)
├── calibration/
│   ├── loggers/             Log FPGA data to file for offline analysis
│   ├── allan_variance/      Streamlit Allan deviation dashboard
│   ├── magnetometer_calibration/   Ellipsoid fit (git submodule)
│   └── data/                Recorded logs (local only, gitignored)
└── esp32_benchmark/
    ├── analyzer_dashboard/  Streamlit dashboard for the ESP32 IMU benchmark
    └── Data/                Recorded DATA*.BIN files (local only, gitignored)
```

---

## Setup

All tools share one environment. From the repository root:

```bash
git submodule update --init
python3 -m venv ekf
source ekf/bin/activate
pip install -r requirements.txt
```

### Serial port

Scripts that talk to the FPGA have a hard-coded `PORT` constant near the top
(`app_mode0.py`, `app_mode1.py`, both loggers, and `gui/constants.py`). Set it
to your USB-UART device before running:

```bash
ls /dev/ | grep -i usb        # macOS / Linux; on Windows use e.g. "COM4"
```

UART settings: 1,152,000 baud, 8N1.

---

## UART Frame Format

The FPGA streams one frame per sample at 1125 Hz. Multi-byte fields are
big-endian.

| Field   | Size     | Content |
|---------|----------|---------|
| SYNC    | 2 bytes  | `0xAA 0x55` |
| MODE/LEN| 2 bytes  | `0x001E` for MODE_0, `0x012E` for MODE_1 (low byte = payload length) |
| PAYLOAD | 30 / 46 bytes | See below |
| CHK     | 2 bytes  | XOR of all 16-bit payload words |

- **MODE_0** (raw IMU, EKF bypassed), `>Q10hH`: 64-bit timestamp, accel xyz,
  gyro xyz, temperature, mag xyz, mag status.
- **MODE_1** (raw IMU + hardware EKF), `>9h7f`: accel, gyro and mag xyz (raw
  counts), `dt` (float32), and the FPGA's state vector
  `[phi, theta, psi, bx, by, bz]` (float32).

`uart_reader.read_packet()` resyncs on the SYNC bytes, validates the checksum
and returns `(mode_word, raw_sample)`, with the sample type chosen from
`MODE_REGISTRY`.

---

## IMU Pipeline

```
FPGA ──UART──▶ uart_reader ──▶ sample_converter ──▶ data_preparer ──▶ ekf
               (RawSample)     (PhysicalSample)     (PreparedSample)   (state)
```

- **`sample_converter.py`** scales raw counts to m/s², rad/s, µT and °C.
  Sensitivities are passed in at construction (ICM-20948 at ±16 g and
  ±2000 dps, AK09916 at 0.15 µT/LSB).
- **`data_preparer.py`** computes per-step `dt`, applies the magnetometer
  hard/soft-iron correction, flags fresh magnetometer samples (100 Hz), and
  computes the initial state and covariance from a stationary window.
- **`ekf.py`** is the software reference filter. State vector:
  `x = [phi, theta, psi, bx, by, bz]` (roll, pitch, yaw in rad; gyro bias in
  rad/s). `step()` runs predict then update; the magnetometer rows are only
  used on fresh mag samples.

Run the command-line apps from the repository root:

```bash
python3 host/imu_pipeline/app_mode0.py   # MODE_0: software EKF on the raw stream
python3 host/imu_pipeline/app_mode1.py   # MODE_1: prints HW and SW state side by side
```

`app_mode1.py` seeds the software EKF from the FPGA's first published state and
then feeds it the same samples and hardware `dt`, so any difference between the
two lines comes from the filter implementations alone.

---

## Real-Time GUI

A PySide6 desktop dashboard that shows hardware and software EKF state, a 3D
orientation view, and their divergence over time. It includes a **Mock** source,
so it runs without any hardware.

*it uses the same files in the IMU pipline directory*

```bash
python3 host/gui/gui_app.py
```

See [`gui/README.md`](gui/README.md) for the design, options and mesh loading.

---

## Calibration

The sensor noise parameters and magnetometer correction used by the EKF come
from these tools. Loggers read MODE_1 frames from the FPGA and write to
`host/calibration/data/`.

### Allan variance (gyro / accel / mag noise)

1. Log a stationary recording. Duration and file name are set by
   `LOGGING_HOURS` and `OUTPUT_PATH` in the script:
   ```bash
   python3 host/calibration/loggers/allan_logger.py
   ```
2. Analyse it. Launch from `host/calibration/` so the 700 MB upload limit in
   `.streamlit/config.toml` applies:
   ```bash
   cd host/calibration
   streamlit run allan_variance/allan_dashboard.py
   ```
   Upload the log to get Allan deviation curves plus ARW / bias instability
   per axis, and the accelerometer offset. See
   [`allan_variance/readme.md`](calibration/allan_variance/readme.md).

### Magnetometer hard/soft-iron

1. Log fresh magnetometer readings while rotating the board through as many
   orientations as possible (Ctrl+C to stop):
   ```bash
   python3 host/calibration/loggers/mag_calibration_logger.py
   ```
2. Fit the ellipsoid with the
   [magnetometer_calibration](https://github.com/nliaudat/magnetometer_calibration)
   submodule (`--field` is the local field strength in µT):
   ```bash
   cd host/calibration
   python3 magnetometer_calibration/calibrate.py -f data/mag_cali.txt --plot --field 50.134 --json
   ```
3. Copy the resulting hard-iron offset and soft-iron matrix into
   `HARD_IRON_OFFSET` / `SOFT_IRON_MATRIX` in the apps and `gui/constants.py`.

---

## ESP32 IMU Benchmark

Tools for the first project phase: comparing the MPU6050, ICM-20948 and WT901
from binary logs recorded by the ESP32 firmware to SD card.

```bash
cd host/esp32_benchmark/analyzer_dashboard
streamlit run dashboard.py
```

Upload a `DATA*.BIN` file to get scaled signals, Allan deviation per sensor and
axis, and ARW / bias instability / rate random walk for each IMU. Launching from
this folder applies the 2 GB upload limit in its `.streamlit/config.toml`.
Processing lives in `data_engine.py`.

`decoder.py` prints the first frames of `Data/DATA31.BIN` as a quick sanity
check of the file format:

```bash
python3 host/esp32_benchmark/analyzer_dashboard/decoder.py
```

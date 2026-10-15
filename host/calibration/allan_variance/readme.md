# IMU Allan Variance Dashboard

Streamlit dashboard that reads an IMU log file and computes Allan deviation
curves (accelerometer, gyroscope, magnetometer) plus ARW/BI metrics per axis.

## Setup

From the repository root:

```bash
pip install -r requirements.txt
```

## Run

Launch it from `host/calibration/` so Streamlit picks up
`host/calibration/.streamlit/config.toml` (raises the upload limit to 700 MB
for long logs):

```bash
cd host/calibration
streamlit run allan_variance/allan_dashboard.py
```

Then open the printed local URL (usually http://localhost:8501) and upload
your `.txt` log file via the sidebar.

## Input format

Comma-separated file, optional header line, columns (fixed order):

```
t, ax, ay, az, gx, gy, gz, mx, my, mz, mag_fresh
```

- Accelerometer/gyroscope columns are sampled at 1125 Hz.
- Magnetometer columns are only valid on rows where `mag_fresh` is `True`
  (100 Hz update rate); all other rows are discarded before the mag
  analysis.

## What it shows

- Three Allan deviation plots (accel, gyro, mag), each with all 3 axes
  overlaid in different colors with a legend.
- A table under each plot with the extracted **ARW** (Angle/Velocity Random
  Walk) and **BI** (Bias Instability) per axis, in the native units of the
  input columns.

See the "Method notes" expander in the app for the exact estimation method.
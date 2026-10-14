# HLS Extended Kalman Filter

C++ source of the hardware EKF, written for AMD Vitis HLS. It is synthesized and
exported as the `ekf` IP core (`xilinx.com:hls:ekf:1.0`) in
[`../IP_repo/ekf`](../IP_repo/ekf), which the block design
[`../block_designs/IMU_ekf.tcl`](../block_designs/IMU_ekf.tcl) places between the
IMU acquisition FIFO and the UART framing.

The filter is the same model as the Python reference
[`host/imu_pipeline/ekf.py`](../../host/imu_pipeline/ekf.py):

- **State:** `x = [phi, theta, psi, bx, by, bz]` (roll, pitch, yaw in rad; gyro
  bias in rad/s)
- **Predict:** gyro-driven attitude kinematics
- **Update:** accelerometer every sample, plus magnetometer when a fresh sample
  (100 Hz) is available

---

## Top Function

```cpp
void ekf_fp(my_stream& A, my_out_stream_t& OUT, bool& gimbal_lock_out);   // src/top.h
```

| Port | Interface | Content |
|---|---|---|
| `A` | AXI-Stream in, 16-bit | One IMU sample per call: 15 words (`READ_STREAM_SIZE`) from the acquisition FIFO |
| `OUT` | AXI-Stream out, 16-bit, `TLAST` | One result frame per call: 23 words (see below) |
| `gimbal_lock_out` | `ap_none` | High while pitch is within 1° of ±90° |
| return | `ap_ctrl_none` | Free-running, no start/done handshake |

The data type is set once in `src/top.h` (`typedef float T;`). An `ap_fixed`
variant is kept there, commented out.

### Per-sample flow

```
convert ─▶ not initialized? ─yes─▶ initialize   (no output frame)
                │
                no
                ▼
             prepare ─▶ update ─▶ output_stream ─▶ predict
```

1. **`convert`** reads the 15 input words, scales the counts to physical units
   (sensor sensitivities, body-frame sign alignment), and computes `dt` from the
   64-bit timestamp (100 MHz ticks).
2. **`initialize`** (first 100 samples, `INIT_AVG_WINDOW_SIZE`) averages a
   stationary window to compute the initial state and covariance. No frames are
   output during this phase.
3. **`prepare`** subtracts the accelerometer bias, flags fresh magnetometer
   samples, and applies the hard/soft-iron correction to fresh ones.
4. **`update`** runs the measurement update: 3 rows (accel) or 6 rows (accel +
   mag), Cholesky-based gain.
5. **`output_stream`** publishes the state right after the update.
6. **`predict`** propagates state and covariance to the next sample.

### Output frame (23 × 16-bit words)

| Words | Content |
|---|---|
| 0–8 | Raw `ax, ay, az, gx, gy, gz, mx, my, mz` (counts) |
| 9–10 | `dt` (float32, lo/hi) |
| 11–22 | State `phi, theta, psi, bx, by, bz` (float32 each, lo/hi) |

`UART_CONTROLLER` sends this as a **MODE_1** frame. The host decodes it as
`RawSampleMode1` in [`host/imu_pipeline`](../../host/imu_pipeline).

---

## Files

| File | Content |
|---|---|
| `src/top.h` | Top function `ekf_fp`, interface and resource-sharing pragmas, `typedef T` |
| `src/converter.h` | `convert`: AXI-Stream input → physical units and `dt` |
| `src/preparer.h` | `initialize` (stationary averaging) and `prepare` (accel bias, mag freshness, mag correction) |
| `src/update.h` | Measurement models (`ha`, `hm`, Jacobians), Cholesky, forward/backward substitution, state and covariance update |
| `src/predict.h` | `G` matrix (with gimbal-lock clamp), `Fd`, `Q`, state and covariance predict |
| `src/output_streamer.h` | `output_stream`: 23-word result frame |
| `src/common_util.h` | Shared arithmetic wrappers (`my_fmul`, `my_fadd`, `my_recip`, `my_sqrt`, `my_sincos`, `tri_computer`, ...) with operator binding for resource sharing |
| `src/constants.h` | All filter constants (see below) |
| `src/types.h` | `converter_out_t`, `ekf_state_t`, `preparer_out` |

### Resource sharing

To fit the Basys 3 (xc7a35t), every arithmetic operation goes through the
wrappers in `common_util.h`. `ekf_fp` limits their instance counts, for example
4× `my_fmul`, 2× `my_fadd` and 1× each of `my_fsub`, `my_recip`, `my_sqrt`,
`tri_computer` and `my_atan2`. The functions then share those units over time.

---

## Constants

`src/constants.h` holds everything that tunes the filter:

- Sensor sensitivities (ICM-20948 ±16 g / ±2000 dps, AK09916 0.15 µT/LSB) and
  body-frame axis signs
- Noise parameters from the Allan variance analysis (`GYRO_ARW`, `GYRO_BI`,
  `ACCEL_DEV`, `MAG_DEV`)
- Magnetometer hard/soft-iron calibration and accelerometer bias
- Local magnetic field (horizontal / vertical components)
- Rates (1125 Hz accel/gyro, 100 Hz mag), initialization window, gimbal-lock
  threshold

> These values are duplicated in the host tools (`host/imu_pipeline/app_mode*.py`,
> `host/gui/constants.py`). When you change one, change both, or the hardware and
> software filters will diverge. [`params/filter_params.yaml`](../../params) is
> meant to become the single source, but its generator script (`gen_params.py`)
> doesn't exist yet.

---

## Testbenches

Each pipeline stage has an isolated C-simulation testbench in `tb/`:

| Testbench | Tests |
|---|---|
| `tb_convert.cpp` | `convert`: two 15-word frames in, scaled values checked |
| `tb_initilalise.cpp` | `initialize`: stationary-window initialization |
| `tb_predict.cpp` | `predict` against golden values from the Python reference model |
| `tb_update.cpp` | `update` against the Python reference (same state and sample in, compare X/P) |
| `tb_trig_bench.cpp` | Trig function benchmark. **Does not compile:** it includes `trig_bench.h`, which isn't in the repository |

`tb_predict.cpp` refers to the golden-vector script `ekf_predict_golden.py`,
which isn't in the repository either.

---

## Building the IP

There is no Vitis HLS project script in the repository yet. To rebuild the IP:

1. Create a Vitis HLS component for part `xc7a35tcpg236-1` with top function
   `ekf_fp`.
2. Add a source file that includes `src/top.h`, and the testbench you want to
   run from `tb/`.
3. Run C simulation, synthesis, and **Export RTL** as a Vivado IP.
4. Replace the contents of `FPGA/IP_repo/ekf` with the exported IP. In Vivado,
   refresh the IP catalog and upgrade the `ekf` instance in the block design.

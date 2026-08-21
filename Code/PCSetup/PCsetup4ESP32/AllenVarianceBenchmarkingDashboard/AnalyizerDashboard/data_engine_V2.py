"""
data_engine.py
==============
Backend processing engine for the IMU Benchmarking Dashboard.

Responsibilities:
    - Binary file parsing
    - Per-sensor scaling (accel, gyro, temperature)
    - 3-sigma outlier removal with local mean replacement
    - WT901 unique sample extraction (200Hz native / 1kHz logged)
    - Allan Variance computation
    - Noise coefficient extraction (ARW, Bias Instability, RRW)
    - ARW normalisation by filter bandwidth

All functions are pure (no Streamlit dependency) and can be
imported and tested independently.

@author  Abdelrahman Hewala
@date    2026
"""

import numpy as np
import pandas as pd
import allantools

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

FRAME_SIZE          = 50        # bytes per frame: uint64 + 21 x int16
FS_DEFAULT          = 1000      # Hz — default logging rate for MPU6050 / ICM20948
FS_WT901            = 200       # Hz — WT901 native output rate
OUTLIER_WINDOW      = 100       # samples — rolling window for local mean replacement
DISPLAY_DOWNSAMPLE  = 200       # show every Nth point in time series plots

# Sensitivity maps
ACCEL_MAP = {2: 16384, 4: 8192, 8: 4096, 16: 2048}         # LSB/g
GYRO_MAP  = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4} # LSB/dps

# Axis / DOF labels
DOF_LABELS   = ["Acc X", "Acc Y", "Acc Z", "Temp", "Gyro X", "Gyro Y", "Gyro Z"]
SENSOR_NAMES = ["IMU 1 (MPU6050)", "IMU 2 (ICM20948)", "IMU 3 (WT901)"]
COLORS       = ["#00C4FF", "#FF6B35", "#00FF9F"]


# ─────────────────────────────────────────────────────────────────────────────
# 1. Binary File Parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_binary_file(file_content: bytes):
    """
    Parse ESP32 binary file into timestamps and raw IMU arrays.

    Frame format (50 bytes):
        uint64  timestamp   — microseconds since boot
        int16   data[21]    — 3 sensors × 7 DOF (ax, ay, az, temp, gx, gy, gz)

    Parameters
    ----------
    file_content : bytes
        Raw binary content of the DATA.BIN file.

    Returns
    -------
    timestamps : np.ndarray (N,)
        Timestamps in microseconds, dtype uint64.
    imu_raw : np.ndarray (N, 3, 7)
        Raw int16 counts for each sensor and DOF. dtype float64.
    """
    dt = np.dtype([
        ('timestamp', '<u8'),
        ('data',      '<i2', 21)
    ])

    num_packets = len(file_content) // FRAME_SIZE
    raw_array   = np.frombuffer(file_content[:num_packets * FRAME_SIZE], dtype=dt)

    timestamps = raw_array['timestamp']
    d          = raw_array['data']   # (N, 21)

    # Split into 3 sensors along axis 1
    imu_raw = np.stack([
        d[:, 0:7],    # IMU 1 — MPU6050
        d[:, 7:14],   # IMU 2 — ICM20948
        d[:, 14:21],  # IMU 3 — WT901
    ], axis=1).astype(np.float64)

    return timestamps, imu_raw


# ─────────────────────────────────────────────────────────────────────────────
# 2. Scaling
# ─────────────────────────────────────────────────────────────────────────────

def scale_sensor(raw: np.ndarray, acc_res: int, gyro_res: int, temp_formula: str) -> np.ndarray:
    """
    Convert raw ADC counts to physical units for a single sensor.

    Parameters
    ----------
    raw : np.ndarray (N, 7)
        Raw counts [ax, ay, az, temp, gx, gy, gz].
    acc_res : int
        Accelerometer full-scale range in g (2, 4, 8, or 16).
    gyro_res : int
        Gyroscope full-scale range in dps (250, 500, 1000, or 2000).
    temp_formula : str
        Temperature conversion formula — "MPU6050", "ICM20948", or "WT901".

    Returns
    -------
    scaled : np.ndarray (N, 7)
        Physical units: g, g, g, °C, dps, dps, dps.
    """
    scaled = raw.copy()

    # Accelerometer: LSB → g
    scaled[:, 0:3] /= ACCEL_MAP[acc_res]

    # Gyroscope: LSB → dps
    scaled[:, 4:7] /= GYRO_MAP[gyro_res]

    # Temperature
    if temp_formula == "MPU6050":
        # MPU6050 datasheet: Temp(°C) = raw/340 + 36.53
        scaled[:, 3] = (raw[:, 3] / 340.0) + 36.53

    elif temp_formula == "ICM20948":
        # ICM20948 datasheet: Temp(°C) = (raw - RoomTemp_Offset) / Temp_Sensitivity + 21
        # Temp_Sensitivity = 333.87 LSB/°C, RoomTemp_Offset = 0
        scaled[:, 3] = (raw[:, 3] / 333.87) + 21.0

    elif temp_formula == "WT901":
        # WT901 protocol: Temp(°C) = raw / 100
        scaled[:, 3] = raw[:, 3] / 100.0

    return scaled


def scale_all_sensors(imu_raw: np.ndarray, sensor_configs: list) -> list:
    """
    Scale all three sensors using their individual configurations.

    Parameters
    ----------
    imu_raw : np.ndarray (N, 3, 7)
        Raw counts from parse_binary_file.
    sensor_configs : list of dict
        Each dict must contain: acc_res, gyro_res, temp_formula.

    Returns
    -------
    scaled : list of np.ndarray (N, 7)
        One array per sensor in physical units.
    """
    return [
        scale_sensor(imu_raw[:, i, :], cfg['acc_res'], cfg['gyro_res'], cfg['temp_formula'])
        for i, cfg in enumerate(sensor_configs)
    ]


# ─────────────────────────────────────────────────────────────────────────────
# 3. WT901 Unique Sample Extraction
# ─────────────────────────────────────────────────────────────────────────────

def extract_wt901_unique(signal: np.ndarray,
                         logged_fs: int = FS_DEFAULT,
                         native_fs: int = FS_WT901) -> np.ndarray:
    """
    Extract unique samples from a WT901 signal.

    The WT901 outputs at 200Hz but is logged at 1kHz — every 5th sample
    is new data; the rest are repeated. Feeding repeated values into
    Allan Variance would artificially reduce the noise floor.

    Parameters
    ----------
    signal : np.ndarray (N,)
        Full signal at logged_fs rate.
    logged_fs : int
        Rate at which the logger sampled the sensor (default 1000Hz).
    native_fs : int
        Native output rate of the WT901 (default 200Hz).

    Returns
    -------
    unique : np.ndarray (N // step,)
        Downsampled signal at native_fs rate.
    """
    step = logged_fs // native_fs  # = 5
    return signal[::step]


# ─────────────────────────────────────────────────────────────────────────────
# 4. Outlier Removal
# ─────────────────────────────────────────────────────────────────────────────

def remove_outliers_3sigma(signal: np.ndarray,
                           window: int = OUTLIER_WINDOW) -> tuple:
    """
    Detect and replace 3-sigma outliers with the local rolling mean.

    Outliers are values that exceed ±3σ of a rolling window centred on
    each sample. They are replaced (not dropped) to preserve uniform
    sample spacing required by Allan Variance.

    Parameters
    ----------
    signal : np.ndarray (N,)
        Input signal in physical units.
    window : int
        Rolling window size in samples for local mean and std.

    Returns
    -------
    clean : np.ndarray (N,)
        Signal with outliers replaced by local mean.
    n_removed : int
        Number of outliers replaced.
    """
    s            = pd.Series(signal)
    rolling_mean = s.rolling(window=window, center=True, min_periods=1).mean()
    rolling_std  = s.rolling(window=window, center=True, min_periods=1).std()

    upper        = rolling_mean + 3 * rolling_std
    lower        = rolling_mean - 3 * rolling_std
    mask         = (s > upper) | (s < lower)

    s_clean      = s.copy()
    s_clean[mask] = rolling_mean[mask]

    return s_clean.values, int(mask.sum())


# ─────────────────────────────────────────────────────────────────────────────
# 5. Allan Variance
# ─────────────────────────────────────────────────────────────────────────────

def compute_allan_variance(signal: np.ndarray,
                           fs: float,
                           data_type: str = "freq") -> tuple:
    """
    Compute overlapping Allan deviation (OADEV) for a signal.

    Parameters
    ----------
    signal : np.ndarray (N,)
        Input signal in physical units (dps for gyro, g for accel).
    fs : float
        Sample rate in Hz.
    data_type : str
        "freq"  — for rate sensors (gyro dps, accel g).
        "phase" — for integrated signals (angle, velocity).

    Returns
    -------
    taus : np.ndarray
        Averaging times in seconds.
    ad : np.ndarray
        Allan deviation values.
    ade : np.ndarray
        Allan deviation error bars (confidence intervals).
    """
    taus, ad, ade, _ = allantools.oadev(
        signal,
        rate=fs,
        data_type=data_type,
        taus='log10'
    )
    return taus, ad, ade


# ─────────────────────────────────────────────────────────────────────────────
# 6. Noise Coefficient Extraction
# ─────────────────────────────────────────────────────────────────────────────

def extract_noise_coefficients(taus: np.ndarray,
                               ad: np.ndarray,
                               bw: float) -> dict:
    """
    Extract IMU noise coefficients from an Allan Deviation curve.

    Extracted parameters
    --------------------
    ARW (raw)        : Allan deviation at τ=1s — angle/velocity random walk.
    ARW (normalised) : ARW / √BW — normalised by filter bandwidth for fair
                       cross-sensor comparison. Accounts for different DLPF
                       settings across sensors (MPU6050 BW≈260Hz vs
                       ICM20948 BW≈246Hz etc.).
    Bias Instability : Minimum of the Allan deviation curve.
    tau_BI           : Averaging time at the bias instability minimum.
    RRW              : Rate random walk coefficient, extracted at τ=3s
                       using σ(τ) = RRW × √τ → RRW = σ(3) / √3.

    Parameters
    ----------
    taus : np.ndarray
        Averaging times from compute_allan_variance.
    ad : np.ndarray
        Allan deviation values from compute_allan_variance.
    bw : float
        Filter bandwidth in Hz used for ARW normalisation.

    Returns
    -------
    coeffs : dict
        Keys: arw, arw_normalised, bias_instab, tau_bi, rrw, bw
    """
    coeffs = {}

    # --- Bias Instability — minimum of curve ---
    min_idx              = int(np.argmin(ad))
    coeffs['bias_instab'] = float(ad[min_idx])
    coeffs['tau_bi']      = float(taus[min_idx])

    # --- ARW — value at τ = 1s (IEEE standard convention) ---
    idx_1s          = int(np.argmin(np.abs(taus - 1.0)))
    coeffs['arw']   = float(ad[idx_1s])

    # --- ARW normalised by bandwidth ---
    # ARW_norm = ARW_measured / √BW
    # This removes the effect of different filter bandwidths so white
    # noise floors are directly comparable across sensors.
    coeffs['arw_normalised'] = float(coeffs['arw'] / np.sqrt(bw)) if bw > 0 else coeffs['arw']
    coeffs['bw']             = float(bw)

    # --- Rate Random Walk — σ(τ) = RRW × √τ → RRW = σ(3) / √3 ---
    idx_3s        = int(np.argmin(np.abs(taus - 3.0)))
    coeffs['rrw'] = float(ad[idx_3s] / np.sqrt(3.0))

    return coeffs


# ─────────────────────────────────────────────────────────────────────────────
# 7. Descriptive Statistics
# ─────────────────────────────────────────────────────────────────────────────

def calculate_stats(signal: np.ndarray) -> dict:
    """
    Compute basic descriptive statistics for a signal.

    Parameters
    ----------
    signal : np.ndarray (N,)

    Returns
    -------
    stats : dict
        Keys: mean, std, min, max, range
    """
    return {
        'mean':  float(np.mean(signal)),
        'std':   float(np.std(signal)),
        'min':   float(np.min(signal)),
        'max':   float(np.max(signal)),
        'range': float(np.max(signal) - np.min(signal))
    }


# ─────────────────────────────────────────────────────────────────────────────
# 8. Full Pipeline Helper
# ─────────────────────────────────────────────────────────────────────────────

def process_signal_for_av(signal: np.ndarray,
                           cfg: dict,
                           dof_idx: int,
                           remove_outliers: bool = True) -> dict:
    """
    Full processing pipeline for a single sensor DOF ready for AV.

    Steps:
        1. Extract DOF signal
        2. WT901 unique sample extraction (if applicable)
        3. Outlier removal (if enabled)
        4. Allan Variance computation
        5. Noise coefficient extraction

    Parameters
    ----------
    signal : np.ndarray (N, 7)
        Scaled sensor data for one sensor.
    cfg : dict
        Sensor configuration with keys:
            is_wt901    : bool
            bw_accel    : float
            bw_gyro     : float
            data_type   : str  ("freq" or "phase")
    dof_idx : int
        Index into DOF_LABELS (0=AccX ... 6=GyroZ).
    remove_outliers : bool
        Whether to apply 3σ outlier removal.

    Returns
    -------
    result : dict
        Keys: signal_clean, taus, ad, ade, coeffs,
              n_outliers, fs_used, n_samples
    """
    sig = signal[:, dof_idx]

    # WT901 — extract unique samples only
    if cfg.get('is_wt901', False):
        sig   = extract_wt901_unique(sig)
        fs    = FS_WT901
    else:
        fs    = FS_DEFAULT

    # Outlier removal
    n_outliers = 0
    if remove_outliers:
        sig, n_outliers = remove_outliers_3sigma(sig)

    # Allan Variance
    data_type = cfg.get('data_type', 'freq')
    taus, ad, ade = compute_allan_variance(sig, fs, data_type)

    # Noise coefficients
    bw     = cfg['bw_gyro'] if dof_idx in [4, 5, 6] else cfg['bw_accel']
    coeffs = extract_noise_coefficients(taus, ad, bw)

    return {
        'signal_clean': sig,
        'taus':         taus,
        'ad':           ad,
        'ade':          ade,
        'coeffs':       coeffs,
        'n_outliers':   n_outliers,
        'fs_used':      fs,
        'n_samples':    len(sig)
    }
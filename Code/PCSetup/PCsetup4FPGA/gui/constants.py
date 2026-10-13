"""
constants.py

Sensor/EKF configuration shared by the acquisition worker (real hardware
path) and the mock data source, so that raw counts synthesized by the mock
decode back to the same physical values through the *unmodified*
SampleConverter/DataPreparer/EKF pipeline as real hardware would produce.

Values below match the filter parameters reported in the thesis
(Section: Filter Parameters).
"""

import serial

# ---- sample_converter.SampleConverter sensitivities (shared by both paths) ----
ACC_LSB_PER_1_G = 2048.0
GYRO_LSB_PER_DPS = 16.4
MAG_LSB = 0.15
TIMESTAMP_CLK_FREQ = 100_000_000.0
TEMP_LSB_PER_C = 333.87
TEMP_OFFSET = 21.0

# ---- Hamburg WMM 2025-2026, used by DataPreparer.initialize()/EKF's mag model ----
MAG_FIELD_HORIZONTAL_UT = 18.225
MAG_FIELD_VERTICAL_UT = 46.710

# ---- default magnetic declination for Hamburg (WMM), degrees, positive = east ----
# Drifts slowly over time (WMM secular variation) -- refresh periodically from
# NOAA's online declination calculator: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
DEFAULT_DECLINATION_DEG_EAST = 4.2

# ---- real-hardware magnetometer calibration (fitted hard/soft iron) ----
HARD_IRON_OFFSET = [
        -26.150471966169953,
        -1.339476869389613,
        24.21988780871615
    ]
SOFT_IRON_MATRIX = [
        [
            1.093901688943135,
            -0.003137558523757345,
            0.005365140656722684
        ],
        [
            -0.0031375585237573265,
            1.0279565161733353,
            -0.049581846704369055
        ],
        [
            0.005365140656722679,
            -0.049581846704369055,
            1.0954591269449934
        ]
    ]

# ---- accelerometer offset o_a (test-rig zeroing; defines resting = level) ----
ACCEL_BIAS_OFFSET = [-0.284029, -0.0266191, 0.131762]

# ---- gyro noise coefficients (Allan variance), per axis ----
GYRO_ARW = [1.313e-4, 1.181e-4, 1.266e-4]     # rad/sqrt(s)
GYRO_BI  = [6.98e-5, 5.89e-5, 6.38e-5]        # rad/s

# ---- measurement uncertainties (R), per axis ----
ACCEL_DEV = [0.1, 0.1, 0.1]                   # m/s^2
MAG_DEV   = [2.0, 2.0, 2.0]                   # uT

# ---- serial defaults ----
DEFAULT_PORT = "/dev/tty.usbserial-210183ACB00B1"
DEFAULT_BAUDRATE = 1152000
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# ---- number of stationary samples used for DataPreparer.initialize() ----
INIT_SAMPLE_COUNT = 100

# ---- mock source: identity mag/accel correction so the model-derived
# physical mag/accel values pass through DataPreparer unchanged ----
MOCK_HARD_IRON_OFFSET = [0.0, 0.0, 0.0]
MOCK_SOFT_IRON_MATRIX = None  # DataPreparer defaults to identity
MOCK_ACCEL_BIAS_OFFSET = [0.0, 0.0, 0.0]

MOCK_SAMPLE_RATE_HZ = 1125.0
""""
 * @file    app_mode1.py
 * @brief   Streams MODE_1 packets (raw IMU + FPGA-computed EKF state) from
 *          the FPGA, runs a parallel software EKF on the raw IMU fields,
 *          and prints hardware vs software state side by side.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""
import serial
from sample_converter import SampleConverter
import sys
from uart_reader import read_packet
from typing import List
from data_preparer import DataPreparer, INT_MIN_WINDOW
from frame_types import PhysicalSampleMode0, PhysicalSampleMode1
from ekf import EKF
import numpy as np

def physical_mode1_as_mode0(s: PhysicalSampleMode1) -> PhysicalSampleMode0:
    """Adapter only -- repackages a MODE_1 sample's raw-sensor fields into
    a PhysicalSampleMode0-shaped object so it can be fed into
    DataPreparer.prepare()/initialize()"""
    return PhysicalSampleMode0(
        t=0, ax=s.ax, ay=s.ay, az=s.az,
        gx=s.gx, gy=s.gy, gz=s.gz,
        temp=float('nan'),
        mx=s.mx, my=s.my, mz=s.mz,
        mag_fresh=s.mag_fresh,
    )

# Constants -- identical to app_mode0.py
ACC_LSB_PER_1_G  = 2048.0
GYRO_LSB_PER_DPS = 16.4
MAG_LSB = 0.15
TIMESTAMP_CLK_FREQ = 100_000_000.0
TEMP_LSB_PER_C = 333.87
TEMP_OFFSET = 21.0

MAG_FIELD_HORIZONZAL_MT = 18.225
MAG_FIELD_VERTICAL_MT = 46.710

PORT     = '/dev/tty.usbserial-210183ACB00B1'
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

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

ACCEL_BIAS_OFFSET = [-0.284029, -0.0266191, 0.131762]

# ---- gyro noise coefficients (Allan variance), per axis ----
GYRO_ARW = [1.313e-4, 1.181e-4, 1.266e-4]     # rad/sqrt(s)
GYRO_BI  = [6.98e-5, 5.89e-5, 6.38e-5]        # rad/s

# ---- measurement uncertainties (R), per axis ----
ACCEL_DEV = [0.1, 0.1, 0.1]                   # m/s^2
MAG_DEV   = [2.0, 2.0, 2.0]  


def main():
    try:
        ser = serial.Serial(
            port=PORT, baudrate=BAUDRATE, bytesize=BYTESIZE,
            parity=PARITY, stopbits=STOPBITS, timeout=1
        )
    except serial.SerialException as e:
        print(f"Could not open serial port {PORT}: {e}")
        print("Check that the FPGA is connected and the PORT constant matches your device.")
        print("Run: ls /dev/ | grep -i usb")
        sys.exit(1)

    sampleConverter = SampleConverter(ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ, MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET)
    dataPreparer = DataPreparer(gyro_ARW=GYRO_ARW, hard_iron_offset=HARD_IRON_OFFSET, soft_iron_matrix=SOFT_IRON_MATRIX,
                                 mag_field_horizontal_uT=MAG_FIELD_HORIZONZAL_MT, mag_field_vertical_uT=MAG_FIELD_VERTICAL_MT,accel_bias_offset=ACCEL_BIAS_OFFSET)

    # ---- seed the software EKF from the FPGA's own state ----
    # The FPGA initialises on its first 100 samples, which it never publishes,
    # so re-running initialize() here would start from a different window
    # (different psi0 and, worse, different bias0 -- the bias converges over
    # tens of seconds and drags psi with it). Instead: take x0 straight from
    # the first published state vector, so the only thing under test is
    # whether SW update/predict tracks HW update/predict.
    # initialize() is still called, but only for P0 and the preparer's dt state.
    samples_init_arr: List[PhysicalSampleMode1] = []
    while len(samples_init_arr) < INT_MIN_WINDOW:
        result = read_packet(ser)
        if result is None:
            continue
        modeWord, raw = result
        samples_init_arr.append(sampleConverter.convert(raw))

    _, p0 = dataPreparer.initialize([physical_mode1_as_mode0(s) for s in samples_init_arr])

    first = samples_init_arr[0]
    x0 = np.array([first.phi, first.theta, first.psi, first.bx, first.by, first.bz])

    ekf = EKF(
        x0=x0, p0=p0,
        gyro_ARW=GYRO_ARW, gyro_bias_instability=GYRO_BI,
        accel_dev=ACCEL_DEV, mag_dev=MAG_DEV,
        mag_field_horizontal_uT=MAG_FIELD_HORIZONZAL_MT,
        mag_field_vertical_uT=MAG_FIELD_VERTICAL_MT
    )

    # The published state is the FPGA's x right AFTER its update() on this
    # packet (see ekf_fp: update -> output_stream -> predict), so only the
    # predict() of packet 1 is still outstanding on the SW side.
    prepared = dataPreparer.prepare(physical_mode1_as_mode0(first))
    prepared.dt = first.dt
    ekf.predict(prepared)

    def run_step(physicalSample1: PhysicalSampleMode1) -> None:
        prepared = dataPreparer.prepare(physical_mode1_as_mode0(physicalSample1))
        prepared.dt = physicalSample1.dt   # use the real hardware-computed dt

        ekf_result = ekf.step(prepared)

        sw_state_deg = ekf_result.X * 180 / np.pi
        hw_state_deg = np.array([
            physicalSample1.phi, physicalSample1.theta, physicalSample1.psi,
            physicalSample1.bx, physicalSample1.by, physicalSample1.bz,
        ]) * 180 / np.pi

        print("HW:", np.array2string(hw_state_deg, precision=2, floatmode='fixed', suppress_small=True))
        print("SW:", np.array2string(sw_state_deg, precision=2, floatmode='fixed', suppress_small=True))
        print()

    # ---- steady-state: run software EKF alongside the hardware's own answer ----
    # first the buffered packets (from packet 2 on), then live
    for physicalSample1 in samples_init_arr[1:]:
        run_step(physicalSample1)

    while True:
        result = read_packet(ser)
        if result is None:
            continue
        modeWord, raw = result
        run_step(sampleConverter.convert(raw))


if __name__ == "__main__":
    main()
""""
 * @file    allan_logger.py
 * @brief   Streams MODE_1 packets (raw IMU + FPGA-computed EKF state) from the
 *          FPGA, converts the raw sensor fields to physical units, and logs
 *          them to a txt file for offline Allan variance analysis by the
 *          Streamlit dashboard. The EKF-state fields carried in MODE_1 are
 *          not logged; only the converted sensor fields and the mag-fresh
 *          flag are written.
 *
 *          Output columns (one line per sample), matching the dashboard's
 *          expected format:
 *            t, ax, ay, az, gx, gy, gz, mx, my, mz, mag_fresh
 *          with accel in m/s^2, gyro in rad/s, mag in uT (scaling only,
 *          no hard/soft-iron correction), mag_fresh a bool.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""

import os
import serial
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))           # host/calibration/loggers
_HOST_DIR = os.path.dirname(os.path.dirname(_THIS_DIR))           # host
_PIPELINE_DIR = os.path.join(_HOST_DIR, "imu_pipeline")
if _PIPELINE_DIR not in sys.path:
    sys.path.insert(0, _PIPELINE_DIR)

from sample_converter import SampleConverter
from uart_reader import read_packet
from frame_types import PhysicalSampleMode1

# Sensor constants
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g,   +-16g
GYRO_LSB_PER_DPS = 16.4               # LSBs/dps, +-2000 dps
MAG_LSB = 0.15                        # uT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # Hz
TEMP_LSB_PER_C = 333.87
TEMP_OFFSET = 21.0

# UART protocol settings
PORT     =  '/dev/tty.usbserial-210183ACB00B1' #'COM4'                     # '/dev/tty.usbserial-10' on macOS
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# Output
OUTPUT_PATH = os.path.join(_THIS_DIR, "..", "data", "accel_off_log_6sec.txt")
LOGGING_HOURS = 0.0016

def main():
    try:
        ser = serial.Serial(
            port=PORT, baudrate=BAUDRATE, bytesize=BYTESIZE,
            parity=PARITY, stopbits=STOPBITS, timeout=1
        )
    except serial.SerialException as e:
        print(f"Could not open serial port {PORT}: {e}")
        print("Check that the FPGA is connected and the PORT constant matches your device.")
        sys.exit(1)

    sampleConverter = SampleConverter(
        ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ,
        MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET
    )

    f = None
    try:
        with open(OUTPUT_PATH, 'w', buffering=1) as f:
            start = time.time()
            print("logging start\n")

            while time.time() - start < LOGGING_HOURS * 60 * 60:
                result = read_packet(ser)
                if result is None:
                    continue
                modeWord, raw = result
                s: PhysicalSampleMode1 = sampleConverter.convert(raw)
                # MODE_1 frames carry dt, not an absolute timestamp; log
                # monotonic elapsed wall-clock time, which is all Allan needs.
                t = time.time() - start
                f.write(
                    f"{t},{s.ax},{s.ay},{s.az},"
                    f"{s.gx},{s.gy},{s.gz},"
                    f"{s.mx},{s.my},{s.mz},{s.mag_fresh}\n"
                )

            print("logging stop\n")
            print("logging period in seconds:", time.time() - start, "\n")

    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        if f is not None and not f.closed:
            f.close()
        ser.close()

if __name__ == "__main__":
    main()
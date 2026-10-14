"""
 * @file    mag_calibration_logger.py
 *
 * @brief   Live logger for magnetometer hard/soft-iron calibration data
 *          collection, using MODE_1 packets only.
 *
 *  @How to use
 *      1) set the port at the PORT Constant, ie: PORT     = '/dev/tty.usbserial-210183ACB00B1'  
 *      2) Run: python3 mag_calibration_logger.py 
 *
 *      To analyse the data: python magnetometer_calibration/calibrate.py -f ../data/mag_cali.txt --plot --field 50.134 --json 
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""
import argparse
import os
import sys
import time
from typing import Optional, Tuple

import serial

# uart_reader, sample_converter
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))           # host/calibration/loggers
_HOST_DIR = os.path.dirname(os.path.dirname(_THIS_DIR))           # host
_PIPELINE_DIR = os.path.join(_HOST_DIR, "imu_pipeline")
if _PIPELINE_DIR not in sys.path:
    sys.path.insert(0, _PIPELINE_DIR)

from uart_reader import read_packet
from sample_converter import SampleConverter

# Constants
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
TEMP_LSB_PER_C =  333.87
TEMP_OFFSET = 21.0
PROGRESS_EVERY_S = 5.0

# UART Protocol Setting CONSTANTS
PORT     = '/dev/tty.usbserial-210183ACB00B1'   # "COM4"    #'/dev/tty.usbserial-10' for MAC
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

MODE_1 = 0x012E   # only MODE_1 packets are logged

# Output
OUTPUT_PATH = os.path.join(_THIS_DIR, "..", "data", "mag_cali.txt")

def run_logger() -> None:
    converter = SampleConverter(ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ, MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET)

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=BYTESIZE,
        parity=PARITY,
        stopbits=STOPBITS,
        timeout=1
    )

    print(f"Logging FRESH magnetometer readings (uT) to {OUTPUT_PATH}")
    print("Rotate the board through as many orientations as possible now.")
    print("Press Ctrl+C to stop early (already-written data stays valid).")

    n_seen = 0
    n_fresh = 0

    try:
        with open(OUTPUT_PATH, 'w', buffering=1) as f:
            while True:
                result = read_packet(ser)
                if result is None:
                    continue
                modeWord, raw = result

                if modeWord != MODE_1:
                    continue  # skip anything that isn't MODE_1

                physical = converter.convert(raw)
                n_seen += 1

                if physical.mag_fresh:
                    f.write(f"{physical.mx},{physical.my},{physical.mz}\n")
                    n_fresh += 1

    except KeyboardInterrupt:
        print(f"\nStopped by user.")
    finally:
        ser.close()

    print()
    print("=== Summary ===")
    print(f"  Total samples seen : {n_seen}")
    print(f"  Fresh mag readings logged : {n_fresh}")
    print(f"  Output file : {OUTPUT_PATH}")


def main():
    run_logger()


if __name__ == '__main__':
    main()

"""
 * @file    mag_calibration_logger.py
 *
 * @brief   Live logger for magnetometer hard/soft-iron calibration data
 *          collection. 
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
 
# UARTFPGA_reader, sample_converter, and frame_types live in a sibling
# directory, not next to this script -- resolve the path relative to this
# file's own location (not the caller's current working directory), so
# this works regardless of where the script is invoked from.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_FPGA_LIB_DIR = os.path.join(_THIS_DIR, '..', 'PCsetup4FPGA')
sys.path.insert(0, _FPGA_LIB_DIR)
 
from UARTFPGA_reader import read_packet
from sample_converter import SampleConverter

# Constants 
# Constants 
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
TEMP_LSB_PER_C =  333.87
TEMP_OFFSET = 21.0

PROGRESS_EVERY_S = 5.0

# UART Protocol Setting CONSTANTS 
PORT     = 'COM4'                                      #'/dev/tty.usbserial-10' for MAC    # change it to whatever it apears in ur machine. try command (grep ('.*usb.*' | '.*USB.*') /dev) in ur terminal to see the nammings
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE      # serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE    # erial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS



class MagFreshnessTracker:
    """Tracks magnetometer reading freshness. Mirrors DataPreparer.prepare()'s
    algorithm exactly, as its own small standalone piece -- deliberately not
    tied to DataPreparer, since that class needs calibration constants this
    collection step doesn't have yet."""

    def __init__(self, mag_fresh_timeout_s: float = 0.010):
        self.mag_fresh_timeout_s = mag_fresh_timeout_s
        self._last_mag_raw: Optional[Tuple[float, float, float]] = None
        self._last_fresh_t: Optional[float] = None

    def is_fresh(self, t: float, mx: float, my: float, mz: float) -> bool:
        raw_mag = (mx, my, mz)
        if self._last_mag_raw is None:
            fresh = True  # first sample ever, nothing to compare against yet
        else:
            changed = raw_mag != self._last_mag_raw
            timed_out = (t - self._last_fresh_t) >= self.mag_fresh_timeout_s
            fresh = changed or timed_out
        if fresh:
            self._last_fresh_t = t
        self._last_mag_raw = raw_mag
        return fresh


def run_logger() -> None:
    converter =  SampleConverter(ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ, MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET)
    tracker = MagFreshnessTracker()

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=BYTESIZE,
        parity=PARITY,
        stopbits=STOPBITS,
        timeout=1
    )

    print(f"Logging FRESH magnetometer readings (uT) to data/")
    print("Rotate the board through as many orientations as possible now.")
    print("Press Ctrl+C to stop early (already-written data stays valid).")

    n_seen = 0
    n_fresh = 0

    try:
        with open("data/mag_cali.txt", 'w', buffering=1) as f:
            while True:
                result = read_packet(ser)
                if result is None:
                    continue
                modeWord, raw = result

                physical = converter.convert(raw)
                n_seen += 1

                if tracker.is_fresh(physical.t, physical.mx, physical.my, physical.mz):
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
    print(f"  Output file : data/")


def main():
    run_logger()


if __name__ == '__main__':
    main()

# python3 mag_calibration_logger.py --port /dev/tty.usbserial-10

#then run the caliv by
# python3 calibrate.py -f ../data/mag_cali.txt --plo

# Results
# hard_iron_offset = [-31.741723, -24.290997, 22.052194]   # uT

# soft_iron_matrix = [
#     [ 1.176901,  0.015028,  0.004831],
#     [ 0.015028,  1.285583, -0.103001],
#     [ 0.004831, -0.103001,  1.311687],
# ]
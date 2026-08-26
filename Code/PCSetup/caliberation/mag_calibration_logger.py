"""
 * @file    mag_calibration_logger.py
 *
 * @brief   Live logger for magnetometer hard/soft-iron calibration data
 *          collection. Reads the UART stream directly, converts each
 *          sample to physical units via sample_converter.SampleConverter,
 *          and writes ONLY the magnetometer axes (x,y,z in uT) to a text
 *          file, in the exact format nliaudat/magnetometer_calibration's
 *          calibrate.py expects (plain "x,y,z" per line, no header).
 *
 *          Only FRESH magnetometer readings are logged, using the same
 *          freshness algorithm as DataPreparer.prepare(): a reading is
 *          fresh if the raw (mx,my,mz) changed since the last sample, or
 *          if mag_fresh_timeout_s has elapsed since the last fresh one.
 *          The magnetometer updates at ~100 Hz while the IMU samples at
 *          ~1125 Hz, so most raw samples would otherwise repeat the same
 *          stale mag reading -- logging those would just waste space and
 *          add duplicate points that don't help the ellipsoid fit.
 *
 *          Deliberately does NOT use DataPreparer directly: that class
 *          needs hard/soft-iron calibration constants as constructor
 *          arguments, which is exactly what this data collection is for
 *          deriving in the first place, and applies bias corrections
 *          that don't belong in raw calibration data.
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
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
ICM20948_TEMP_SENSITIVITY_LSB_PER_C =  333.87
ICM20948_TEMP_OFFSET_C = 21.0

PROGRESS_EVERY_S = 5.0


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


def run_logger(port: str, baudrate: int, duration_s: float, output_path: str,
               serial_timeout_s: float = 1.0) -> None:
    converter = SampleConverter(
        accel_sensitivity_lsb_per_g=ACC_LSBs_PER_1_G,
        gyro_sensitivity_lsb_per_dps=GYRO_LSBs_PER_DPS,
        timestamp_ticks_per_second=1.0 / (TIMESTAMP_CYCLE_PERIOD * 1e-9),
        mag_sensitivity_ut_per_lsb=MAG_LSB,
        temp_sensitivity_lsb_per_c=ICM20948_TEMP_SENSITIVITY_LSB_PER_C,
        temp_offset_c=ICM20948_TEMP_OFFSET_C,
    )
    tracker = MagFreshnessTracker()

    print(f"Opening {port} @ {baudrate} baud ...")
    ser = serial.Serial(port, baudrate, timeout=serial_timeout_s)

    print(f"Logging FRESH magnetometer readings (uT) to {output_path}")
    print("Rotate the board through as many orientations as possible now.")
    print("Press Ctrl+C to stop early (already-written data stays valid).")

    start_time = time.time()
    last_progress_time = start_time
    n_seen = 0
    n_fresh = 0

    try:
        with open(output_path, 'w', buffering=1) as f:
            while True:
                now = time.time()
                elapsed = now - start_time
                if duration_s is not None and elapsed >= duration_s:
                    print(f"\nReached target duration ({duration_s:.0f}s). Stopping.")
                    break

                result = read_packet(ser)
                if result is None:
                    continue

                mode_word, raw = result
                physical = converter.convert(raw)
                n_seen += 1

                if tracker.is_fresh(physical.t, physical.mx, physical.my, physical.mz):
                    f.write(f"{physical.mx},{physical.my},{physical.mz}\n")
                    n_fresh += 1

                if now - last_progress_time >= PROGRESS_EVERY_S:
                    print(f"  [{elapsed:5.1f}s] {n_seen:>7} samples seen, "
                          f"{n_fresh:>5} fresh mag readings logged")
                    last_progress_time = now

    except KeyboardInterrupt:
        print(f"\nStopped early by user.")
    finally:
        ser.close()

    print()
    print("=== Summary ===")
    print(f"  Total samples seen : {n_seen}")
    print(f"  Fresh mag readings logged : {n_fresh}")
    print(f"  Output file : {output_path}")
    print(f"  Next: python calibrate.py -f {output_path} --plot --json --save")


def main():
    parser = argparse.ArgumentParser(
        description="Live magnetometer calibration data logger. Rotate the "
                    "board through many orientations while this runs.")
    parser.add_argument('--port', default='/dev/tty.usbserial-10')
    parser.add_argument('--baud', type=int, default=1152000)
    parser.add_argument('--seconds', type=float, default=None,
                        help="Duration to log, in seconds. Omit to run until Ctrl+C.")
    parser.add_argument('--out', default='mag_out.txt',
                        help="Output path (default: mag_out.txt, calibrate.py's default input name)")
    args = parser.parse_args()

    run_logger(port=args.port, baudrate=args.baud, duration_s=args.seconds, output_path=args.out)


if __name__ == '__main__':
    main()

# python3 mag_calibration_logger.py --port /dev/tty.usbserial-10

"""
 * @file  sample_converter.py

* @briefConverts raw decoded IMU samples (produced by
* UARTFPGA_reader.read_packet) into physical units for EKF consumption.

*This calss usese the RawSampleMode0 and RawSampleMode1 classes as its input
the output is an instantant of either PhysicalSampleMode0 class or PhysicalSampleMode1 depending on the input class. 

* @author  Abdelrahman Hewala
* @note    Supervisor: Prof. Lutz Leutelt
* @date    2026
"""

from dataclasses import dataclass
from typing import Optional
from frame_types import RawSampleMode0, PhysicalSampleMode0 #, RawSampleMode1. # this should be added once done
import time

MAG_LOGGING_FREQ_HZ = 100  # 100 HZ

class SampleConverter:
    """
    Converts RawSample received as in put either mode0 sample or mode1. and convert any to its phyical value. 
    to do so, the sensors sensitivities must be provided as the instantiation.
    """

    def __init__(
        self,
        accel_sensitivity_lsb_per_g  : float,
        gyro_sensitivity_lsb_per_dps : float,
        timestamp_ticks_per_second   : float,
        mag_sensitivity_ut_per_lsb   : float = 0.15,             # µT/LSB
        temp_sensitivity_lsb_per_c   : Optional[float] = None,
        temp_offset_c: float = 0.0,
    ):
        """
        accel_sensitivity_lsb_per_g  : LSB counts per 1g, (e.g. 16384.0 for MPU6050 @ +-2g).
        gyro_sensitivity_lsb_per_dps : LSB counts per 1 deg/s, ge (e.g. 131.0 for MPU6050 @ +-250dps).
        timestamp_ticks_per_second   : tick rate of the FPGA for our FPGA it is 100 MHz
        mag_sensitivity_lsb_per_ut   : uT per 1 LSB , if left to None, it will pass mx/my/mz through as raw counts (cast to float)
        temp_sensitivity_lsb_per_c   : LSB counts per 1 deg C, for linear scale + offset convention. Leave None to pass temp through as raw counts (cast to float).
        temp_offset_c                : offset term for the temp conversion, applied as (raw / temp_sensitivity_lsb_per_c) + temp_offset_c. Ignored if temp_sensitivity_lsb_per_c is None.
        """
        self.accel_sens = accel_sensitivity_lsb_per_g
        self.gyro_sens = gyro_sensitivity_lsb_per_dps
        self.ts_ticks_per_sec = timestamp_ticks_per_second
        self.mag_sens = mag_sensitivity_ut_per_lsb
        self.temp_sens = temp_sensitivity_lsb_per_c
        self.temp_offset = temp_offset_c

        self.last_fresh_mag_t : float = 0.0
        self.last_fresh_mag   : list[float] = [0.0, 0.0, 0.0]

        # Dispatch table: raw sample type -> conversion method. 
        # type is defined in frame_types.py.
        # still need to add RawSampleMode1 -- will be done once ekf is done on the FPGA, but will be added here
        self._dispatch = {
            RawSampleMode0: self._convert_mode0,
        }

    def convert(self, raw):
        """Converts any registered RawSample type to its physical-unit
        counterpart. Dispatches on the raw sample's type. if mode0, will dispatch mode0 converter and same for mode 1"""
        fn = self._dispatch.get(type(raw))
        if fn is None:
            raise ValueError(f"no converter registered for {type(raw).__name__}")
        return fn(raw)

    def _convert_mode0(self, raw: RawSampleMode0) -> PhysicalSampleMode0:
        t = raw.ts / self.ts_ticks_per_sec

        ax = - raw.ax / self.accel_sens * 9.81   # m/s2
        ay = raw.ay / self.accel_sens * 9.81   # m/s2
        az = raw.az / self.accel_sens * 9.81   # m/s2

        gx = raw.gx / self.gyro_sens * (3.14159265/180)  #rads/s
        gy = - raw.gy / self.gyro_sens * (3.14159265/180)  #rads/s
        gz = - raw.gz / self.gyro_sens * (3.14159265/180)  #rads/s

        if self.temp_sens is not None:
            temp = raw.temp / self.temp_sens + self.temp_offset
        else:
            temp = float(raw.temp)

        mx = raw.mx * self.mag_sens
        my = raw.my * self.mag_sens
        mz = raw.mz * self.mag_sens

        # Hnadeling of the Valid flag
        mag_reading = [mx, my, mz]
        mag_changed = mag_reading != self.last_fresh_mag
        timed_out = t - self.last_fresh_mag_t > (1 / MAG_LOGGING_FREQ_HZ )
        mag_fresh = mag_changed or timed_out

        if mag_fresh:
            self.last_fresh_mag_t = t
            self.last_fresh_mag   = mag_reading

        # return
        return PhysicalSampleMode0(
            t=t, ax=ax, ay=ay, az=az,
            gx=gx, gy=gy, gz=gz,
            temp=temp, mx=mx, my=my, mz=mz,
            mag_fresh=mag_fresh,
        )
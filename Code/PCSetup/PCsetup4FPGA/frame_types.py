"""
frame_types.py

Raw sample type definitions, one per MODE. Shared between UARTFPGA_reader and sample_converter (dispatches
conversion logic based on which type it receives). 
"""

from typing import NamedTuple
from dataclasses import dataclass

class RawSampleMode0(NamedTuple):
    """One decoded MODE_0 frame (IMU data, EKF bypassed), in raw register
    counts. matches struct.unpack('>Q10hH', payload)."""
    ts: int
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    temp: int
    mx: int
    my: int
    mz: int
    mag_state: int


@dataclass
class PhysicalSampleMode0:
    """One converted MODE_0 sample, in physical units."""
    t: float           # seconds
    ax: float          # m/s2
    ay: float          # m/s2
    az: float          # m/s2
    gx: float          # rads/s
    gy: float          # rads/s
    gz: float          # rads/s
    temp: float        # deg C
    mx: float          # physical field units (uT)
    my: float          # physical field units (uT)
    mz: float          # physical field units (uT)
    mag_fresh: int     # passed through unconverted -- status flags word.


# Prepared sample For EKF. Only used For Mode 0
@dataclass
class PreparedSample:
    """One sample, ready for the EKF predict/update step."""
    dt: float           # seconds since the previous sample
    gx: float           # rad/s, raw -- bias not yet subtracted, the EKF does that
    gy: float
    gz: float
    ax: float            # m/s^2
    ay: float
    az: float
    mx: float            # uT, hard/soft-iron corrected
    my: float
    mz: float
    temp: float
    mag_fresh: bool      # True if this mag reading should drive an update this step



# RawSampleMode1: ToDo -- MODE_1 payload (IMU data + state vector + performance
# data) is not finalized yet. 
#
# class RawSampleMode1(NamedTuple):
#     ...
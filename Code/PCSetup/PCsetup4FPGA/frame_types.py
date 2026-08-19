"""
frame_types.py

Raw sample type definitions, one per MODE. Shared between UARTFPGA_reader and sample_converter (dispatches
conversion logic based on which type it receives). 
"""

from typing import NamedTuple


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


# RawSampleMode1: ToDo -- MODE_1 payload (IMU data + state vector + performance
# data) is not finalized yet. 
#
# class RawSampleMode1(NamedTuple):
#     ...
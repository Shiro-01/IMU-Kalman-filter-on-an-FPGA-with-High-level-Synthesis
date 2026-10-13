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



class RawSampleMode1(NamedTuple):
    """One decoded MODE_1 frame: raw IMU counts + hardware-computed dt and
    state vector. Matches output_streamer.h's frame layout exactly:
      words 0-8   : raw_data[0..8]  -- ax,ay,az,gx,gy,gz,mx,my,mz (raw counts,
                    same convention as RawSampleMode0's IMU words -- NOT yet
                    scaled to physical units, that's sample_converter's job)
      words 9-10  : dt   -- float32, split lo/hi, already reconstructed here
      words 11-22 : state vector [phi,theta,psi,bx,by,bz] -- float32 each,
                    split lo/hi, already reconstructed here (these are the
                    EKF's own outputs, not raw sensor counts, so there's
                    nothing left for sample_converter to do with them)
    """
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    mx: int
    my: int
    mz: int
    dt: float
    phi: float
    theta: float
    psi: float
    bx: float
    by: float
    bz: float



@dataclass
class PhysicalSampleMode1:
    """One converted MODE_1 sample. ax/ay/az/gx/gy/gz/mx/my/mz, dt and state vector are bypassed
    Mag flag is calcualted."""
    ax: float             # m/s2
    ay: float             # m/s2
    az: float             # m/s2
    gx: float              # rads/s
    gy: float              # rads/s
    gz: float              # rads/s
    mx: float              # physical field units (uT)
    my: float              # physical field units (uT)
    mz: float              # physical field units (uT)
    dt: float              # seconds, hardware-computed, pass-through
    phi: float             # rad, hardware EKF output, pass-through
    theta: float           # rad, hardware EKF output, pass-through
    psi: float             # rad, hardware EKF output, pass-through
    bx: float              # rad/s, hardware EKF output, pass-through
    by: float              # rad/s, hardware EKF output, pass-through
    bz: float              # rad/s, hardware EKF output, pass-through
    mag_fresh: bool         # diagnostic only -- not fed to a second EKF
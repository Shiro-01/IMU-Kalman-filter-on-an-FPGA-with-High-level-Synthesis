# allan_dashboard_core.py -- analysis core used the dashboard
# (Streamlit UI, styling, and plotting omitted; see project repository)

import allantools
import numpy as np

BI_SCALE_FACTOR = 0.664
DEFAULT_BI_TAU_MAX_S = 200.0


def compute_allan_deviation(data, rate_hz):
    data = np.asarray(data, dtype=float)
    data = data[np.isfinite(data)]
    if len(data) < 10:
        return None, None
    try:
        taus, adev, _err, _n = allantools.oadev(
            data, rate=rate_hz, data_type="freq", taus="octave")
    except Exception:
        return None, None
    mask = np.isfinite(taus) & np.isfinite(adev) & (taus > 0) & (adev > 0)
    taus, adev = taus[mask], adev[mask]
    if len(taus) < 3:
        return None, None
    return taus, adev


def estimate_arw_bi(tau, adev, bi_tau_max=DEFAULT_BI_TAU_MAX_S):
    # ARW = sigma(tau=1s), read directly (no assumed slope).
    # BI  = min(sigma over tau <= bi_tau_max) / 0.664, excluding the
    #       unreliable long-tau tail.
    if tau is None or adev is None or len(tau) < 3:
        return float("nan"), float("nan"), None, None
    log_tau, log_adev = np.log10(tau), np.log10(adev)
    if tau.min() <= 1.0 <= tau.max():
        arw = 10 ** np.interp(0.0, log_tau, log_adev)
    else:
        arw = float(adev[int(np.argmin(np.abs(tau - 1.0)))])
    idx_arw = int(np.argmin(np.abs(tau - 1.0)))
    reliable = tau <= bi_tau_max
    if reliable.sum() >= 1:
        idx_bi = int(np.argmin(np.where(reliable, adev, np.inf)))
    else:
        idx_bi = int(np.argmin(adev))
    bi = adev[idx_bi] / BI_SCALE_FACTOR
    return arw, bi, idx_arw, idx_bi


def accel_offset(df, gravity):
    # Offset of a stationary accel recording: mean(a) - (0, 0, g).
    ox = float(df["ax"].mean())
    oy = float(df["ay"].mean())
    oz = float(df["az"].mean()) - gravity
    return ox, oy, oz
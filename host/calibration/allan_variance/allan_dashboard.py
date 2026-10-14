"""
IMU Allan Variance Analysis Dashboard
======================================

Streamlit dashboard for computing and visualizing Allan deviation curves
for accelerometer, gyroscope, and magnetometer data logged from an IMU.

Run with:
    streamlit run allan_dashboard.py

Method summary:
  * Allan deviation via allantools.oadev (overlapping), data_type="freq",
    taus="octave". Every logged column is a rate-like quantity, so "freq"
    is correct.
  * White-noise coefficient (ARW/VRW) = sigma(tau = 1 s), read directly off
    the curve by log-log interpolation at tau = 1 s. No slope is assumed:
    forcing a -1/2 fit would bias the reading where the curve is not purely
    white noise near tau = 1 s.
  * Bias instability (BI) = min(sigma)/0.664, but the minimum is taken over
    the RELIABLE part of the curve only (tau <= a cutoff set in the sidebar).
    At the longest tau the Allan deviation is computed from very few clusters
    and dips spuriously; that tail must be excluded or argmin grabs it.
  * Accelerometer offset (stationary log only) = mean(a) - (0, 0, g), shown
    as a table. g is set in the sidebar.
  * The white-noise coefficient is shown in several equivalent unit spellings
    per sensor. sqrt(Hz) = 1/sqrt(s), so [unit]/sqrt(Hz) and [unit]*sqrt(s)
    are the same quantity; the columns differ only by rad<->deg or
    (m/s^2)<->g conversion, for direct comparison against the datasheet.
"""

import io

import allantools
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

COLUMN_NAMES = ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "mag_fresh"]

# Default sample rates (Hz) -- only defaults; the sidebar value is what is
# actually used, so a different log/device can be re-analyzed by changing it.
DEFAULT_ACCEL_RATE_HZ = 1125.0
DEFAULT_GYRO_RATE_HZ = 1125.0
DEFAULT_MAG_RATE_HZ = 100.0

# Default gravity (m/s^2) used for the accelerometer offset.
DEFAULT_GRAVITY = 9.81

# Default cutoff (s): BI minimum is searched only over tau <= this, to avoid
# the unreliable long-tau tail of the Allan curve.
DEFAULT_BI_TAU_MAX_S = 200.0

AXIS_COLORS = {"x": "#1f77b4", "y": "#d62728", "z": "#2ca02c"}

# Bias instability scale factor (IEEE-STD-952 / Allan deviation flicker floor)
BI_SCALE_FACTOR = 0.664

# Unit conversions for the alternative coefficient spellings
DEG_PER_RAD = 180.0 / np.pi
G_PER_MS2 = 1.0 / 9.81
UG_PER_G = 1e6


# --------------------------------------------------------------------------
# Data loading / parsing
# --------------------------------------------------------------------------

def load_dataframe(uploaded_file) -> pd.DataFrame:
    """Parse the uploaded txt/csv file into a clean DataFrame.

    The file may (or may not) start with a header line such as:
        {physicalSample.t},{physicalSample.ax},...,{physicalSample.mag_fresh}
    Any non-numeric / brace-containing first line is treated as a header and
    skipped; columns are always assigned COLUMN_NAMES regardless.
    """
    raw_bytes = uploaded_file.read()
    text = raw_bytes.decode("utf-8", errors="ignore")
    lines = text.splitlines()

    if not lines:
        raise ValueError("The uploaded file is empty.")

    first_line = lines[0]
    skip_header = ("{" in first_line) or ("physicalSample" in first_line) or (
        not first_line.strip().split(",")[0].strip().lstrip("-").replace(".", "", 1).isdigit()
    )

    data_str = "\n".join(lines[1:]) if skip_header else text

    df = pd.read_csv(
        io.StringIO(data_str),
        header=None,
        names=COLUMN_NAMES,
        skipinitialspace=True,
        engine="python",
    )

    numeric_cols = ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df["mag_fresh"] = (
        df["mag_fresh"]
        .astype(str)
        .str.strip()
        .str.lower()
        .map({"true": True, "false": False, "1": True, "0": False})
    )

    df = df.dropna(subset=["t", "ax", "ay", "az", "gx", "gy", "gz"]).reset_index(drop=True)
    return df


def get_valid_mag(df: pd.DataFrame) -> pd.DataFrame:
    """Return only the valid, de-duplicated magnetometer rows.

    A True `mag_fresh` row is kept only if the immediately following row is
    not also True (i.e. it is the last True in its run, or the last row),
    since only the last fresh read in a run is a genuinely new mag sample.
    """
    mag_fresh = df["mag_fresh"] == True  # noqa: E712
    next_is_fresh = mag_fresh.shift(-1).fillna(False).astype(bool)
    keep = mag_fresh & ~next_is_fresh
    mag_df = df[keep].dropna(subset=["mx", "my", "mz"])
    return mag_df.reset_index(drop=True)


# --------------------------------------------------------------------------
# Allan variance computation
# --------------------------------------------------------------------------

def compute_allan_deviation(data: np.ndarray, rate_hz: float):
    """Overlapping Allan deviation via allantools. Returns (tau, adev) or
    (None, None) if there is not enough data. data_type="freq" is correct:
    every logged column is a rate, not an integrated phase."""
    data = np.asarray(data, dtype=float)
    data = data[np.isfinite(data)]
    if len(data) < 10:
        return None, None
    try:
        taus, adev, _err, _n = allantools.oadev(
            data, rate=rate_hz, data_type="freq", taus="octave"
        )
    except Exception:
        return None, None
    mask = np.isfinite(taus) & np.isfinite(adev) & (taus > 0) & (adev > 0)
    taus, adev = taus[mask], adev[mask]
    if len(taus) < 3:
        return None, None
    return taus, adev


def estimate_arw_bi(tau, adev, bi_tau_max=DEFAULT_BI_TAU_MAX_S):
    """Extract the white-noise coefficient (ARW/VRW) and bias instability (BI).

      ARW = sigma(tau = 1 s), read directly by log-log interpolation at
            tau = 1 s (no assumed slope). If tau = 1 s falls outside the data
            range the nearest sample is used instead of clamping.
      BI  = min(sigma over tau <= bi_tau_max) / 0.664. Restricting the search
            excludes the unreliable long-tau tail, where the Allan deviation
            is computed from very few clusters and dips spuriously.

    Returns (arw, bi, idx_arw, idx_bi).
    """
    if tau is None or adev is None or len(tau) < 3:
        return float("nan"), float("nan"), None, None

    log_tau = np.log10(tau)
    log_adev = np.log10(adev)

    # ARW: curve's own value at tau = 1 s
    if tau.min() <= 1.0 <= tau.max():
        arw = 10 ** np.interp(0.0, log_tau, log_adev)
    else:
        arw = float(adev[int(np.argmin(np.abs(tau - 1.0)))])
    idx_arw = int(np.argmin(np.abs(tau - 1.0)))

    # BI: minimum over the reliable part only
    reliable = tau <= bi_tau_max
    if reliable.sum() >= 1:
        idx_bi = int(np.argmin(np.where(reliable, adev, np.inf)))
    else:
        idx_bi = int(np.argmin(adev))
    bi = adev[idx_bi] / BI_SCALE_FACTOR

    return arw, bi, idx_arw, idx_bi


def arw_alt_views(sensor_name, arw):
    """Equivalent unit spellings of the white-noise coefficient. sqrt(Hz) =
    1/sqrt(s), so [unit]/sqrt(Hz) == [unit]*sqrt(s); the rest is rad<->deg or
    (m/s^2)<->g."""
    if not np.isfinite(arw):
        return []
    if sensor_name == "Gyroscope":
        return [
            ("rad/\u221as  (= rad/s/\u221aHz)", f"{arw:.6g}"),
            ("\u00b0/\u221as  (= \u00b0/s/\u221aHz)", f"{arw * DEG_PER_RAD:.6g}"),
            ("\u00b0/\u221ah", f"{arw * DEG_PER_RAD * 60.0:.6g}"),
        ]
    if sensor_name == "Accelerometer":
        return [
            ("(m/s\u00b2)/\u221aHz  (= m/s\u00b2\u00b7\u221as)", f"{arw:.6g}"),
            ("g/\u221aHz", f"{arw * G_PER_MS2:.6g}"),
            ("\u00b5g/\u221aHz", f"{arw * G_PER_MS2 * UG_PER_G:.6g}"),
        ]
    return [("\u00b5T\u00b7\u221as  (= \u00b5T/\u221aHz)", f"{arw:.6g}")]


def accel_offset(df: pd.DataFrame, gravity: float):
    """Offset of a STATIONARY accelerometer recording: mean(a) - (0, 0, g).
    Returns (ox, oy, oz) in the data unit (m/s^2). Assumes the sensor was
    stationary and roughly level for the whole record."""
    ox = float(df["ax"].mean()) - 0.0
    oy = float(df["ay"].mean()) - 0.0
    oz = float(df["az"].mean()) - gravity
    return ox, oy, oz


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------

def build_allan_figure(title: str, curves: dict, y_unit: str):
    """curves: axis_label -> (tau, adev, color, idx_arw, idx_bi).
    y_unit labels the Allan-deviation axis with the sensor's data unit."""
    fig = go.Figure()
    for axis_label, (tau, adev, color, idx_arw, idx_bi) in curves.items():
        if tau is None:
            continue
        fig.add_trace(
            go.Scatter(
                x=tau, y=adev, mode="lines+markers", name=axis_label,
                line=dict(color=color, width=2), marker=dict(size=4),
            )
        )
    fig.update_xaxes(type="log", title_text="Averaging time, \u03c4 (s)")
    fig.update_yaxes(type="log", title_text=f"Allan deviation \u03c3(\u03c4)  [{y_unit}]")
    fig.update_layout(
        title=title, legend_title_text="Axis", height=420,
        margin=dict(l=70, r=20, t=50, b=50),
        template="plotly_white", hovermode="x unified",
    )
    return fig


# --------------------------------------------------------------------------
# Styling
# --------------------------------------------------------------------------

def inject_sidebar_style():
    st.markdown(
        """
        <style>
        section[data-testid="stSidebar"] { background-color: #1a1a1a; }
        section[data-testid="stSidebar"] * { color: #f0f0f0; }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] {
            background-color: #3a3a3a; border: 1px solid #555555; }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] * {
            color: #f0f0f0 !important; }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] svg {
            fill: #f0f0f0 !important; }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderFile"] {
            background-color: #3a3a3a; border-radius: 4px; }
        section[data-testid="stSidebar"] input[type="number"] {
            background-color: #f0f0f0; color: #111111 !important; }
        section[data-testid="stSidebar"] div[data-baseweb="input"] {
            background-color: #f0f0f0; border-radius: 4px; }
        section[data-testid="stSidebar"] button {
            background-color: #4a4a4a; color: #f0f0f0 !important;
            border: 1px solid #666666; }
        </style>
        """,
        unsafe_allow_html=True,
    )


# --------------------------------------------------------------------------
# Metrics tables
# --------------------------------------------------------------------------

def render_metrics_table(sensor_name: str, metrics: dict, bi_unit: str):
    st.caption(f"**{sensor_name} \u2014 Bias Instability**")
    bi_rows = [
        {"Axis": ax, f"BI ({bi_unit})": (f"{v['BI']:.6g}" if np.isfinite(v["BI"]) else "n/a")}
        for ax, v in metrics.items()
    ]
    st.dataframe(pd.DataFrame(bi_rows), width="stretch", hide_index=True)

    st.caption(f"**{sensor_name} \u2014 White-noise coefficient (ARW / VRW)**")
    arw_rows = []
    for ax, v in metrics.items():
        row = {"Axis": ax}
        for label, val in arw_alt_views(sensor_name, v["ARW"]):
            row[label] = val
        arw_rows.append(row)
    st.dataframe(pd.DataFrame(arw_rows), width="stretch", hide_index=True)


def render_accel_offset_table(offset, gravity):
    ox, oy, oz = offset
    st.caption(f"**Accelerometer \u2014 offset (stationary, g = {gravity:g} m/s\u00b2)**")
    st.caption(
        "Offset = mean(a) \u2212 (0, 0, g). Subtract this from every sample "
        "before the filter. Valid only if this log is stationary and roughly level."
    )
    df = pd.DataFrame([
        {"Axis": "X", "Offset (m/s\u00b2)": f"{ox:.6g}"},
        {"Axis": "Y", "Offset (m/s\u00b2)": f"{oy:.6g}"},
        {"Axis": "Z", "Offset (m/s\u00b2)": f"{oz:.6g}"},
    ])
    st.dataframe(df, width="stretch", hide_index=True)


# --------------------------------------------------------------------------
# Streamlit app
# --------------------------------------------------------------------------

def main():
    st.set_page_config(page_title="IMU Allan Variance Analysis", layout="wide")
    inject_sidebar_style()

    st.title("Allan Variance Analysis for IMU Data")
    st.caption(
        "Upload a formatted IMU log (.txt/.csv) to compute Allan deviation "
        "curves for the accelerometer, gyroscope, and magnetometer, the "
        "extracted white-noise coefficient (ARW/VRW) and bias instability (BI) "
        "per axis, and the accelerometer offset."
    )

    with st.sidebar:
        st.header("Data input")
        uploaded_file = st.file_uploader("Upload IMU log (.txt or .csv)", type=["txt", "csv"])

        st.markdown("---")
        st.header("Sample rates (Hz)")
        st.caption("Set these to match the log you upload.")
        accel_rate_hz = st.number_input(
            "Accelerometer rate", min_value=0.001, value=DEFAULT_ACCEL_RATE_HZ, step=1.0, format="%.3f")
        gyro_rate_hz = st.number_input(
            "Gyroscope rate", min_value=0.001, value=DEFAULT_GYRO_RATE_HZ, step=1.0, format="%.3f")
        mag_rate_hz = st.number_input(
            "Magnetometer rate", min_value=0.001, value=DEFAULT_MAG_RATE_HZ, step=1.0, format="%.3f",
            help="Effective update rate of rows where mag_fresh = True.")

        st.markdown("---")
        st.header("Bias instability")
        st.caption(
            "BI = min(sigma) / 0.664, searched only over tau <= the cutoff "
            "below. Raise/lower it to exclude the unreliable long-tau tail "
            "(the spurious dip at the right edge of the curve)."
        )
        bi_tau_max = st.number_input(
            "BI search cutoff, tau max (s)", min_value=0.001,
            value=DEFAULT_BI_TAU_MAX_S, step=10.0, format="%.3f")

        st.markdown("---")
        st.header("Accelerometer offset")
        st.caption("Gravity used for the offset = mean(a) - (0, 0, g).")
        gravity = st.number_input(
            "g (m/s\u00b2)", min_value=0.0, value=DEFAULT_GRAVITY, step=0.01, format="%.4f")

    if uploaded_file is None:
        st.info("Upload a .txt file to begin analysis.")
        st.markdown(
            "**Expected format** (header optional -- columns are fixed):\n\n"
            "```\n"
            "t, ax, ay, az, gx, gy, gz, mx, my, mz, mag_fresh\n"
            "```\n"
            "- `ax, ay, az`: accelerometer, **m/s\u00b2**\n"
            "- `gx, gy, gz`: gyroscope, **rad/s**\n"
            "- `mx, my, mz`: magnetometer, **\u00b5T** (valid where `mag_fresh = True`)\n"
            "- `mag_fresh`: `True`/`False`"
        )
        return

    try:
        df = load_dataframe(uploaded_file)
    except Exception as e:
        st.error(f"Failed to parse the uploaded file: {e}")
        return

    mag_df = get_valid_mag(df)
    st.success(
        f"Loaded {len(df):,} samples "
        f"({len(mag_df):,} valid magnetometer samples with `mag_fresh = True`).")

    with st.expander("Preview raw data"):
        st.dataframe(df.head(50), width="stretch")

    sensor_specs = {
        "Accelerometer": {"df": df, "cols": {"x": "ax", "y": "ay", "z": "az"},
                          "rate": accel_rate_hz, "bi_unit": "m/s\u00b2", "y_unit": "m/s\u00b2"},
        "Gyroscope": {"df": df, "cols": {"x": "gx", "y": "gy", "z": "gz"},
                      "rate": gyro_rate_hz, "bi_unit": "rad/s", "y_unit": "rad/s"},
        "Magnetometer": {"df": mag_df, "cols": {"x": "mx", "y": "my", "z": "mz"},
                         "rate": mag_rate_hz, "bi_unit": "\u00b5T", "y_unit": "\u00b5T"},
    }

    results = {}
    for sensor_name, spec in sensor_specs.items():
        curves, metrics = {}, {}
        for axis, col in spec["cols"].items():
            tau, adev = compute_allan_deviation(spec["df"][col].values, spec["rate"])
            arw, bi, idx_arw, idx_bi = estimate_arw_bi(tau, adev, bi_tau_max)
            curves[f"{axis.upper()} ({col})"] = (tau, adev, AXIS_COLORS[axis], idx_arw, idx_bi)
            metrics[axis.upper()] = {"ARW": arw, "BI": bi}
        results[sensor_name] = {
            "curves": curves, "metrics": metrics,
            "bi_unit": spec["bi_unit"], "y_unit": spec["y_unit"]}

    # Accel & Gyro side by side
    st.markdown("---")
    top1, top2 = st.columns(2)
    with top1:
        r = results["Accelerometer"]
        st.plotly_chart(build_allan_figure("Accelerometer \u2014 Allan Deviation", r["curves"], r["y_unit"]),
                        width="stretch")
        render_metrics_table("Accelerometer", r["metrics"], r["bi_unit"])
        render_accel_offset_table(accel_offset(df, gravity), gravity)
    with top2:
        r = results["Gyroscope"]
        st.plotly_chart(build_allan_figure("Gyroscope \u2014 Allan Deviation", r["curves"], r["y_unit"]),
                        width="stretch")
        render_metrics_table("Gyroscope", r["metrics"], r["bi_unit"])

    # Mag centered below
    st.markdown("---")
    _, mid, _ = st.columns([1, 2, 1])
    with mid:
        r = results["Magnetometer"]
        st.plotly_chart(build_allan_figure("Magnetometer \u2014 Allan Deviation", r["curves"], r["y_unit"]),
                        width="stretch")
        render_metrics_table("Magnetometer", r["metrics"], r["bi_unit"])

    st.markdown("---")
    with st.expander("Method notes"):
        st.markdown(
            "- Allan deviation: `allantools.oadev`, `data_type=\"freq\"`, "
            "`taus=\"octave\"`.\n"
            "- **ARW / VRW** = \u03c3(\u03c4=1 s), read directly by log-log "
            "interpolation at \u03c4 = 1 s (no assumed slope).\n"
            "- **BI** = \u03c3_min / 0.664, minimum taken only over "
            "\u03c4 \u2264 the sidebar cutoff, to exclude the unreliable "
            "long-\u03c4 tail.\n"
            "- **Accelerometer offset** = mean(a) \u2212 (0, 0, g), valid only "
            "for a stationary, roughly level log.\n"
            "- The white-noise coefficient is shown in equivalent spellings: "
            "\u221aHz = 1/\u221as, so [unit]/\u221aHz and [unit]\u00b7\u221as are "
            "identical; compare the datasheet against the matching column "
            "(gyro \u00b0/s/\u221aHz, accel \u00b5g/\u221aHz).\n"
            "- Magnetometer rows filtered to `mag_fresh = True` only."
        )


if __name__ == "__main__":
    main()
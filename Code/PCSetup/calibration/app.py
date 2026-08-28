"""
IMU Allan Variance Analysis Dashboard
======================================

Streamlit dashboard for computing and visualizing Allan deviation curves
for accelerometer, gyroscope, and magnetometer data logged from an IMU.

Run with:
    streamlit run app.py
"""

import io

import allantools
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from plotly.subplots import make_subplots

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

COLUMN_NAMES = ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "mag_fresh"]

# Default sample rates (Hz) — used to pre-fill the sidebar controls. These
# are only defaults: the actual rate fed into allantools is whatever the
# user has set in the sidebar at analysis time, so a different log file
# (different device / config) can be re-analyzed just by changing the number.
DEFAULT_ACCEL_RATE_HZ = 1125.0
DEFAULT_GYRO_RATE_HZ = 1125.0
DEFAULT_MAG_RATE_HZ = 100.0

AXIS_COLORS = {"x": "#1f77b4", "y": "#d62728", "z": "#2ca02c"}

# Bias instability scale factor (IEEE-STD-952 / Allan deviation flicker-noise floor)
BI_SCALE_FACTOR = 0.664


# --------------------------------------------------------------------------
# Data loading / parsing
# --------------------------------------------------------------------------

def load_dataframe(uploaded_file) -> pd.DataFrame:
    """Parse the uploaded txt/csv file into a clean DataFrame.

    The file may (or may not) start with a header line such as:
        {physicalSample.t},{physicalSample.ax},...,{physicalSample.mag_fresh}
    Any non-numeric / brace-containing first line is treated as a header
    and skipped; columns are always assigned the fixed names in
    COLUMN_NAMES regardless of what the header line actually says.
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

    # Coerce numeric columns
    numeric_cols = ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Normalize mag_fresh to boolean (handles "True"/"False"/" True"/booleans/1/0)
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

    Rows are only meaningful when `mag_fresh` is True. Occasionally two (or
    more) rows in a row are flagged fresh in sequence — in that case only
    the LAST True in the run is a genuinely new mag sample, e.g.:

        1) False
        2) True     <- discarded (superseded by the next fresh read)
        3) True     <- kept (last fresh read in this run)
        4) False

    A True row is kept only if the immediately following row is not also
    True (i.e. it's the last True in its run, or the very last row).
    """
    mag_fresh = df["mag_fresh"] == True  # noqa: E712
    # shift() upcasts a bool Series to object dtype to hold the edge NaN,
    # which breaks `~` (bitwise-not on Python ints instead of boolean
    # negation) unless cast back to bool explicitly.
    next_is_fresh = mag_fresh.shift(-1).fillna(False).astype(bool)
    keep = mag_fresh & ~next_is_fresh

    mag_df = df[keep].dropna(subset=["mx", "my", "mz"])
    return mag_df.reset_index(drop=True)


# --------------------------------------------------------------------------
# Allan variance computation
# --------------------------------------------------------------------------

def compute_allan_deviation(data: np.ndarray, rate_hz: float):
    """Compute overlapping Allan deviation using allantools.

    Returns (tau, adev) or (None, None) if there isn't enough data.
    """
    data = np.asarray(data, dtype=float)
    data = data[np.isfinite(data)]

    if len(data) < 10:
        return None, None

    try:
        taus, adev, _adev_err, _n = allantools.oadev(
            data, rate=rate_hz, data_type="freq", taus="octave"
        )
    except Exception:
        return None, None

    # Keep only strictly positive, finite values (log-log plot requirement)
    mask = np.isfinite(taus) & np.isfinite(adev) & (taus > 0) & (adev > 0)
    taus, adev = taus[mask], adev[mask]

    if len(taus) < 3:
        return None, None

    return taus, adev


def estimate_arw_bi(tau: np.ndarray, adev: np.ndarray):
    """Estimate Angle/Velocity Random Walk (ARW) and Bias Instability (BI)
    from an Allan deviation curve:

      - ARW = the Allan deviation curve's own value AT tau = 1 s, i.e.
        sigma(1). This is read directly off the plotted curve by log-log
        interpolating between the two octave-tau samples bracketing tau=1
        (or the nearest edge sample if tau=1 falls outside the data's tau
        range). By definition, on a pure slope -1/2 (random-walk) region
        sigma(tau) = ARW / sqrt(tau), so sigma(1) = ARW -- reading it
        directly guarantees the number always matches what's on the graph
        at tau=1, instead of extrapolating a -1/2 line from some other,
        possibly distant, point (which amplifies any deviation from a
        perfect -1/2 slope the farther that anchor point is from tau=1).
      - BI is the minimum of the Allan deviation curve, divided by the
        scale factor 0.664 (relates the AD flicker-noise floor to the
        bias instability coefficient).

    Returns (arw, bi, idx_arw, idx_bi):
      - arw, bi: the estimated values (native units of the input data)
      - idx_arw: index of the tau sample nearest to 1 s (informational only,
        not used in the arw calculation itself, which interpolates)
      - idx_bi: index into tau/adev of the minimum-deviation point used for BI

    NOTE on BI: the standard relation is
        BI = sigma_min / 0.664   (DIVIDE by 0.664)
    not sigma_min * 0.664. Multiplying instead of dividing is the most
    common source of a mismatch when cross-checking by hand off the plot.
    """
    if tau is None or adev is None or len(tau) < 3:
        return float("nan"), float("nan"), None, None

    log_tau = np.log10(tau)
    log_adev = np.log10(adev)

    # --- ARW: read the curve's own value at tau = 1 s directly ---
    # np.interp requires increasing xp; tau/log_tau from oadev are already
    # increasing. Outside the data's tau range it clamps to the nearest
    # edge value rather than extrapolating.
    log_arw = np.interp(0.0, log_tau, log_adev)  # log10(tau)=0 <=> tau=1
    arw = 10 ** log_arw
    idx_arw = int(np.argmin(np.abs(tau - 1.0)))  # nearest sample, for reference only

    # --- Bias instability: minimum of the curve (flicker-noise floor) ---
    idx_bi = int(np.argmin(adev))
    bi = adev[idx_bi] / BI_SCALE_FACTOR  # divide, not multiply

    return arw, bi, idx_arw, idx_bi


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------

def build_allan_figure(title: str, curves: dict):
    """curves: dict of axis_label -> (tau, adev, color, idx_arw, idx_bi)

    idx_arw / idx_bi are accepted but not plotted (see estimate_arw_bi's
    docstring for how ARW/BI are derived) — only the Allan deviation curves
    themselves are drawn, one per axis.
    """
    fig = go.Figure()

    for axis_label, (tau, adev, color, idx_arw, idx_bi) in curves.items():
        if tau is None:
            continue
        fig.add_trace(
            go.Scatter(
                x=tau,
                y=adev,
                mode="lines+markers",
                name=axis_label,
                line=dict(color=color, width=2),
                marker=dict(size=4),
            )
        )

    fig.update_xaxes(type="log", title_text="Averaging time, τ (s)")
    fig.update_yaxes(type="log", title_text="Allan deviation σ(τ)")
    fig.update_layout(
        title=title,
        legend_title_text="Axis",
        height=420,
        margin=dict(l=60, r=20, t=50, b=50),
        template="plotly_white",
        hovermode="x unified",
    )
    return fig


# --------------------------------------------------------------------------
# Styling
# --------------------------------------------------------------------------

def inject_sidebar_style():
    """Dark sidebar with readable contrast, incl. the file-upload widget."""
    st.markdown(
        """
        <style>
        /* Sidebar base */
        section[data-testid="stSidebar"] {
            background-color: #1a1a1a;
        }
        section[data-testid="stSidebar"] * {
            color: #f0f0f0;
        }

        /* File uploader dropzone: give it a mid-gray fill (not white-on-white,
           not black-on-black) with dark text/icons for contrast */
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] {
            background-color: #3a3a3a;
            border: 1px solid #555555;
        }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] * {
            color: #f0f0f0 !important;
        }
        section[data-testid="stSidebar"] [data-testid="stFileUploaderDropzone"] svg {
            fill: #f0f0f0 !important;
        }

        /* Uploaded-file chip after a file is selected */
        section[data-testid="stSidebar"] [data-testid="stFileUploaderFile"] {
            background-color: #3a3a3a;
            border-radius: 4px;
        }

        /* Number inputs (sample rate fields): light fill, dark readable text */
        section[data-testid="stSidebar"] input[type="number"] {
            background-color: #f0f0f0;
            color: #111111 !important;
        }
        section[data-testid="stSidebar"] div[data-baseweb="input"] {
            background-color: #f0f0f0;
            border-radius: 4px;
        }

        /* Browse-files button */
        section[data-testid="stSidebar"] button {
            background-color: #4a4a4a;
            color: #f0f0f0 !important;
            border: 1px solid #666666;
        }
        </style>
        """,
        unsafe_allow_html=True,
    )


# --------------------------------------------------------------------------
# Streamlit app
# --------------------------------------------------------------------------

def main():
    st.set_page_config(page_title="IMU Allan Variance Analysis", layout="wide")
    inject_sidebar_style()

    st.title("Allan Variance Analysis for IMU Data")
    st.caption(
        "Upload a formatted IMU log (.txt/.csv) to compute Allan deviation curves "
        "for the accelerometer, gyroscope, and magnetometer, along with the "
        "extracted Angle/Velocity Random Walk (ARW/VRW) and Bias Instability (BI) "
        "for each axis."
    )

    with st.sidebar:
        st.header("Data input")
        uploaded_file = st.file_uploader("Upload IMU log (.txt or .csv)", type=["txt", "csv"])

        st.markdown("---")
        st.header("Sample rates (Hz)")
        st.caption(
            "Set these to match the log you upload — a different file/device "
            "with a different logging rate can be re-analyzed just by changing "
            "the numbers below. They are fed straight into the Allan deviation "
            "computation."
        )
        accel_rate_hz = st.number_input(
            "Accelerometer rate", min_value=0.001, value=DEFAULT_ACCEL_RATE_HZ, step=1.0, format="%.3f"
        )
        gyro_rate_hz = st.number_input(
            "Gyroscope rate", min_value=0.001, value=DEFAULT_GYRO_RATE_HZ, step=1.0, format="%.3f"
        )
        mag_rate_hz = st.number_input(
            "Magnetometer rate", min_value=0.001, value=DEFAULT_MAG_RATE_HZ, step=1.0, format="%.3f",
            help="Effective update rate of rows where mag_fresh = True.",
        )

    if uploaded_file is None:
        st.info("Upload a .txt file to begin analysis.")
        st.markdown(
            "**Expected format** (header optional — columns are fixed regardless of the header text):\n\n"
            "```\n"
            "t [s], ax [m/s^2], ay [m/s^2], az [m/s^2], "
            "gx [rad/s], gy [rad/s], gz [rad/s], "
            "mx [uT], my [uT], mz [uT], mag_fresh [bool]\n"
            "6969.31075067,-0.0143701171875,-0.258662109375,9.94412109375,"
            "-0.015963377286585367,0.004256900609756098,-0.007449576067073172,"
            "-46.5,-24.9,54.75, True\n"
            "6969.31163955,0.0143701171875,-0.27303222656250004,9.953701171875,"
            "-0.02341295335365854,-0.0010642251524390245,-0.005321125762195123,"
            "-46.5,-24.9,54.75, False\n"
            "```\n"
            "- `t`: sample timestamp (s)\n"
            "- `ax, ay, az`: accelerometer, in **m/s²**\n"
            "- `gx, gy, gz`: gyroscope, in **rad/s**\n"
            "- `mx, my, mz`: magnetometer, in **µT** — only valid on rows where `mag_fresh = True`\n"
            "- `mag_fresh`: `True`/`False` flag marking a fresh magnetometer sample"
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
        f"({len(mag_df):,} valid magnetometer samples with `mag_fresh = True`)."
    )

    with st.expander("Preview raw data"):
        st.dataframe(df.head(50), width="stretch")

    # ----------------------------------------------------------------
    # Compute Allan deviation for each sensor / axis
    # ----------------------------------------------------------------
    sensor_specs = {
        "Accelerometer": {
            "df": df,
            "cols": {"x": "ax", "y": "ay", "z": "az"},
            "rate": accel_rate_hz,
            "unit": "m/s²",
        },
        "Gyroscope": {
            "df": df,
            "cols": {"x": "gx", "y": "gy", "z": "gz"},
            "rate": gyro_rate_hz,
            "unit": "rad/s",
        },
        "Magnetometer": {
            "df": mag_df,
            "cols": {"x": "mx", "y": "my", "z": "mz"},
            "rate": mag_rate_hz,
            "unit": "µT",
        },
    }

    results = {}
    for sensor_name, spec in sensor_specs.items():
        curves = {}
        metrics = {}
        for axis, col in spec["cols"].items():
            tau, adev = compute_allan_deviation(spec["df"][col].values, spec["rate"])
            arw, bi, idx_arw, idx_bi = estimate_arw_bi(tau, adev)
            curves[f"{axis.upper()} ({col})"] = (tau, adev, AXIS_COLORS[axis], idx_arw, idx_bi)
            metrics[axis.upper()] = {"ARW": arw, "BI": bi}
        results[sensor_name] = {"curves": curves, "metrics": metrics, "unit": spec["unit"]}

    # ----------------------------------------------------------------
    # Layout: Accel & Gyro side by side on top, Mag centered below
    # ----------------------------------------------------------------
    st.markdown("---")
    top_col1, top_col2 = st.columns(2)

    with top_col1:
        st.plotly_chart(
            build_allan_figure("Accelerometer — Allan Deviation", results["Accelerometer"]["curves"]),
            width="stretch",
        )
        render_metrics_table("Accelerometer", results["Accelerometer"]["metrics"], results["Accelerometer"]["unit"])

    with top_col2:
        st.plotly_chart(
            build_allan_figure("Gyroscope — Allan Deviation", results["Gyroscope"]["curves"]),
            width="stretch",
        )
        render_metrics_table("Gyroscope", results["Gyroscope"]["metrics"], results["Gyroscope"]["unit"])

    st.markdown("---")
    bottom_left, bottom_center, bottom_right = st.columns([1, 2, 1])
    with bottom_center:
        st.plotly_chart(
            build_allan_figure("Magnetometer — Allan Deviation", results["Magnetometer"]["curves"]),
            width="stretch",
        )
        render_metrics_table("Magnetometer", results["Magnetometer"]["metrics"], results["Magnetometer"]["unit"])

    st.markdown("---")
    with st.expander("Method notes"):
        st.markdown(
            "- Allan deviation is computed with `allantools.oadev` (overlapping Allan deviation), "
            "`data_type=\"freq\"`, `taus=\"octave\"`.\n"
            "- **ARW / VRW** (Angle/Velocity Random Walk) = σ(τ=1 s), read directly off each curve "
            "(log-log interpolated between the two nearest τ samples) — so it always matches the "
            "curve's own height at τ=1 s.\n"
            "- **BI** (Bias Instability) = σ_min / 0.664, i.e. the minimum of the Allan deviation "
            "curve **divided** by the scale factor 0.664 (the flicker-noise floor relation from "
            "IEEE-STD-952) — not multiplied.\n"
            "- Magnetometer rows are filtered to `mag_fresh = True` only. Sample rates for all three "
            "sensors are set in the sidebar (defaults: accel/gyro 1125 Hz, mag 100 Hz) and are used "
            "directly by `allantools` — change them for a different log file/device without editing "
            "any code.\n"
            "- Values are reported in the **native units of the input columns** "
            "(no unit conversion is applied)."
        )


def render_metrics_table(sensor_name: str, metrics: dict, unit: str):
    rows = []
    for axis, vals in metrics.items():
        arw = vals["ARW"]
        bi = vals["BI"]
        rows.append(
            {
                "Axis": axis,
                f"ARW ({unit}/√Hz)": f"{arw:.6g}" if np.isfinite(arw) else "n/a",
                f"BI ({unit})": f"{bi:.6g}" if np.isfinite(bi) else "n/a",
            }
        )
    st.caption(f"**{sensor_name} — ARW & Bias Instability**")
    st.dataframe(pd.DataFrame(rows), width="stretch", hide_index=True)


if __name__ == "__main__":
    main()
"""
dashboard.py
============
Frontend for the IMU Benchmarking Dashboard.
All processing is delegated to data_engine.py.

Run with:
    streamlit run dashboard.py

@author  Abdelrahman Hewala
@date    2026
"""

import streamlit as st
import numpy as np
import plotly.graph_objects as go

from Code.PCSetup.PCsetup4ESP32.AllenVarianceBenchmarkingDashboard.AnalyizerDashboard.data_engine_V2 import (
    parse_binary_file,
    scale_all_sensors,
    process_signal_for_av,
    calculate_stats,
    DOF_LABELS,
    SENSOR_NAMES,
    COLORS,
    DISPLAY_DOWNSAMPLE,
    FS_DEFAULT,
    FS_WT901,
)

def hex_to_rgba(hex_color: str, alpha: float = 0.1) -> str:
    """Convert hex color to rgba string for Plotly transparency support."""
    hex_color = hex_color.lstrip('#')
    r, g, b = int(hex_color[0:2], 16), int(hex_color[2:4], 16), int(hex_color[4:6], 16)
    return f"rgba({r},{g},{b},{alpha})"

# ─────────────────────────────────────────────────────────────────────────────
# Page Config
# ─────────────────────────────────────────────────────────────────────────────
st.set_page_config(
    layout="wide",
    page_title="IMU Benchmarking Dashboard",
    page_icon="🛸"
)

st.markdown("""
<style>
    .block-container { padding-top: 1rem; }
    .metric-card {
        background: #161B22;
        border: 1px solid #30363D;
        border-radius: 8px;
        padding: 12px 16px;
        margin: 6px 0;
    }
    .coeff-label { color: #8B949E; font-size: 11px; text-transform: uppercase; letter-spacing: 0.05em; }
    .coeff-value { color: #FAFAFA; font-size: 20px; font-weight: bold; font-family: monospace; }
    .coeff-unit  { color: #8B949E; font-size: 11px; }
    .sensor-header { color: #FAFAFA; font-size: 14px; font-weight: 600; margin-bottom: 8px; }
</style>
""", unsafe_allow_html=True)

# ─────────────────────────────────────────────────────────────────────────────
# Sidebar
# ─────────────────────────────────────────────────────────────────────────────
with st.sidebar:
    st.title("🛸 IMU Dashboard")
    st.caption("Allan Variance Benchmarking Tool")
    st.divider()

    # File upload
    st.header("📁 Data")
    uploaded_file = st.file_uploader("Upload DATA.BIN", type=["bin"])

    st.divider()

    # Per-sensor configuration
    st.header("⚙️ Sensor Configuration")
    sensor_configs = []

    defaults = [
        # acc_res, gyro_res, bw_accel, bw_gyro, temp_formula, is_wt901
        (2,   250,  260.0,  256.0,  "MPU6050",  False),
        (2,   250,  265.0,  229.8,  "ICM20948", False),
        (16, 2000,  256.0,  256.0,  "WT901",    True),
    ]

    for i, (name, defs) in enumerate(zip(SENSOR_NAMES, defaults)):
        with st.expander(name, expanded=(i == 0)):
            acc_res  = st.selectbox("Accel Range",  [2, 4, 8, 16],
                                    index=[2,4,8,16].index(defs[0]),
                                    key=f"acc_{i}",
                                    format_func=lambda x: f"±{x} g")
            gyro_res = st.selectbox("Gyro Range",   [250, 500, 1000, 2000],
                                    index=[250,500,1000,2000].index(defs[1]),
                                    key=f"gyro_{i}",
                                    format_func=lambda x: f"±{x} dps")
            bw_accel = st.number_input("Accel Filter BW (Hz)", value=defs[2],
                                       min_value=1.0, key=f"bwa_{i}")
            bw_gyro  = st.number_input("Gyro Filter BW (Hz)",  value=defs[3],
                                       min_value=1.0, key=f"bwg_{i}")
            temp_formula = st.selectbox("Temp Formula",
                                        ["MPU6050", "ICM20948", "WT901"],
                                        index=["MPU6050","ICM20948","WT901"].index(defs[4]),
                                        key=f"temp_{i}")
            is_wt901 = st.checkbox("WT901 mode (200Hz native / 1kHz logged)",
                                   value=defs[5], key=f"wt_{i}")
            data_type = st.selectbox("AV Data Type", ["freq", "phase"],
                                     index=0, key=f"dt_{i}",
                                     help="'freq' for rate sensors (gyro/accel), 'phase' for integrated")

            sensor_configs.append({
                "acc_res":      acc_res,
                "gyro_res":     gyro_res,
                "bw_accel":     bw_accel,
                "bw_gyro":      bw_gyro,
                "temp_formula": temp_formula,
                "is_wt901":     is_wt901,
                "data_type":    data_type,
            })

    st.divider()
    st.header("🔬 Analysis Settings")
    remove_outliers_flag = st.checkbox("Remove 3σ outliers", value=True,
                                       help="Replace outliers exceeding 3σ with local rolling mean")
    dof_idx = st.selectbox("Signal to Analyse", range(7),
                            format_func=lambda x: DOF_LABELS[x])
    show_confidence = st.checkbox("Show AV confidence bands", value=True)

# ─────────────────────────────────────────────────────────────────────────────
# Guard — no file uploaded
# ─────────────────────────────────────────────────────────────────────────────
if uploaded_file is None:
    st.title("🛸 IMU Benchmarking Dashboard")
    st.info("👈 Upload a **DATA.BIN** file from your SD card to begin analysis.")
    st.stop()

# ─────────────────────────────────────────────────────────────────────────────
# Parse & Scale
# ─────────────────────────────────────────────────────────────────────────────
@st.cache_data(show_spinner="📂 Parsing binary file...")
def load_file(raw: bytes):
    return parse_binary_file(raw)

raw_bytes = uploaded_file.read()
timestamps, imu_raw = load_file(raw_bytes)
scaled = scale_all_sensors(imu_raw, sensor_configs)

n_frames   = len(timestamps)
duration_s = (timestamps[-1] - timestamps[0]) / 1e6
ts_s       = (timestamps - timestamps[0]) / 1e6   # µs → s

# Header summary
st.title("🛸 IMU Benchmarking Dashboard")
c1, c2, c3, c4 = st.columns(4)
c1.metric("Frames",   f"{n_frames:,}")
c2.metric("Duration", f"{duration_s/3600:.2f} h")
c3.metric("File size", f"{len(raw_bytes)/1e6:.1f} MB")
c4.metric("Log rate",  f"{FS_DEFAULT} Hz")

st.divider()

# ─────────────────────────────────────────────────────────────────────────────
# Time Series
# ─────────────────────────────────────────────────────────────────────────────
st.subheader(f"📈 Time Series — {DOF_LABELS[dof_idx]}")

fig_ts = go.Figure()
ts_ds  = ts_s[::DISPLAY_DOWNSAMPLE]

for i, (sig, name, color) in enumerate(zip(scaled, SENSOR_NAMES, COLORS)):
    fig_ts.add_trace(go.Scatter(
        x=ts_ds,
        y=sig[::DISPLAY_DOWNSAMPLE, dof_idx],
        name=name,
        line=dict(color=color, width=1),
        mode='lines'
    ))

fig_ts.update_layout(
    xaxis_title="Time [s]",
    yaxis_title=DOF_LABELS[dof_idx],
    height=320,
    margin=dict(l=40, r=20, t=20, b=40),
    legend=dict(orientation="h", y=1.05),
    plot_bgcolor="#0E1117",
    paper_bgcolor="#0E1117",
    font=dict(color="#FAFAFA")
)
fig_ts.update_xaxes(gridcolor="#2A2A2A")
fig_ts.update_yaxes(gridcolor="#2A2A2A")
st.plotly_chart(fig_ts, use_container_width=True)

st.caption(f"⚡ Display downsampled to every {DISPLAY_DOWNSAMPLE}th point — full resolution used for Allan Variance.")

# ─────────────────────────────────────────────────────────────────────────────
# Statistics
# ─────────────────────────────────────────────────────────────────────────────
st.divider()
st.subheader("📊 Descriptive Statistics")

stat_cols = st.columns(3)
for i, (sig, name, col) in enumerate(zip(scaled, SENSOR_NAMES, stat_cols)):
    stats = calculate_stats(sig[:, dof_idx])
    with col:
        st.markdown(f"**{name}**")
        sc1, sc2 = st.columns(2)
        sc1.metric("Mean",  f"{stats['mean']:.4f}")
        sc2.metric("Std",   f"{stats['std']:.4f}")
        sc1.metric("Min",   f"{stats['min']:.4f}")
        sc2.metric("Max",   f"{stats['max']:.4f}")

# ─────────────────────────────────────────────────────────────────────────────
# Allan Variance
# ─────────────────────────────────────────────────────────────────────────────
st.divider()
st.subheader("📉 Allan Variance — Noise Characterisation")

is_gyro = dof_idx in [4, 5, 6]
unit    = "°/s" if is_gyro else "g"

av_results  = []
coeff_data  = []

with st.spinner("⏳ Computing Allan Variance for all sensors..."):
    for i, (cfg, sig, name, color) in enumerate(zip(sensor_configs, scaled, SENSOR_NAMES, COLORS)):
        result = process_signal_for_av(sig, cfg, dof_idx, remove_outliers=remove_outliers_flag)
        av_results.append({**result, 'name': name, 'color': color})
        coeff_data.append(result['coeffs'])

        # Report outliers and sample count
        if remove_outliers_flag and result['n_outliers'] > 0:
            pct = result['n_outliers'] / result['n_samples'] * 100
            st.caption(f"ℹ️ {name}: replaced **{result['n_outliers']}** outliers ({pct:.2f}%) with local mean")

        if cfg['is_wt901']:
            st.caption(f"ℹ️ {name}: WT901 mode — using **{result['n_samples']:,}** unique samples at {result['fs_used']}Hz")

# ── AV Plot ──────────────────────────────────────────────────────────────────
fig_av = go.Figure()

# Sensor curves
for r in av_results:
    taus, ad, ade = r['taus'], r['ad'], r['ade']
    col  = r['color']
    c    = r['coeffs']

    # Confidence band
    if show_confidence:
        fig_av.add_trace(go.Scatter(
            x=np.concatenate([taus, taus[::-1]]),
            y=np.concatenate([ad + ade, (ad - ade)[::-1]]),
            fill='toself',
            fillcolor=hex_to_rgba(col, 0.1),
            line=dict(width=0),
            showlegend=False,
            hoverinfo='skip'
        ))

    # Main curve
    fig_av.add_trace(go.Scatter(
        x=taus, y=ad,
        name=r['name'],
        line=dict(color=col, width=2),
        mode='lines'
    ))




fig_av.update_xaxes(type="log", title="Averaging Time τ [s]", gridcolor="#2A2A2A")
fig_av.update_yaxes(type="log", title=f"Allan Deviation σ(τ) [{unit}]", gridcolor="#2A2A2A")
fig_av.update_layout(
    height=580,
    margin=dict(l=40, r=20, t=20, b=40),
    legend=dict(orientation="h", y=1.02),
    plot_bgcolor="#0E1117",
    paper_bgcolor="#0E1117",
    font=dict(color="#FAFAFA")
)

fig_av.update_xaxes(
    type="log",
    title="Averaging Time τ [s]",
    range=[-3, 4],   # 10⁻³ to 10⁴ seconds — covers all physically meaningful taus
    gridcolor="#2A2A2A"
)
st.plotly_chart(fig_av, use_container_width=True)

# ─────────────────────────────────────────────────────────────────────────────
# Noise Coefficients Table
# ─────────────────────────────────────────────────────────────────────────────
st.divider()
st.subheader("🔢 Extracted Noise Coefficients")

unit_arw = f"{unit}/√Hz"

coeff_cols = st.columns(3)
for i, (name, color, coeffs) in enumerate(zip(SENSOR_NAMES, COLORS, coeff_data)):
    bw = coeffs['bw']
    with coeff_cols[i]:
        st.markdown(f"<div style='color:{color}; font-weight:600; font-size:15px; margin-bottom:8px'>{name}</div>",
                    unsafe_allow_html=True)
        st.markdown(f"""
        <div class="metric-card">
            <div class="coeff-label">ARW (raw @ τ=1s)</div>
            <div class="coeff-value">{coeffs['arw']:.4e}</div>
            <div class="coeff-unit">{unit} @ τ=1s</div>
        </div>
        <div class="metric-card">
            <div class="coeff-label">Bias Instability (minimum)</div>
            <div class="coeff-value">{coeffs['bias_instab']:.4e}</div>
            <div class="coeff-unit">{unit} @ τ={coeffs['tau_bi']:.1f}s</div>
        </div>
        <div class="metric-card">
            <div class="coeff-label">Rate Random Walk</div>
            <div class="coeff-value">{coeffs['rrw']:.4e}</div>
            <div class="coeff-unit">{unit}/s^½</div>
        </div>
        """, unsafe_allow_html=True)

# ─────────────────────────────────────────────────────────────────────────────
# Guide
# ─────────────────────────────────────────────────────────────────────────────
st.divider()
with st.expander("📖 How to read the Allan Variance plot"):
    st.markdown(f"""
    | Region | Slope | Noise Type | Kalman Filter Parameter |
    |--------|-------|------------|------------------------|
    | Left side (low τ) | **-½** | White noise / ARW | Process noise **Q** (high freq component) |
    | Bottom (minimum) | **0** | Bias Instability / Flicker | Bias drift state in **Q** |
    | Right side (high τ) | **+½** | Rate Random Walk | Long-term drift model |

    **ARW Normalisation:**
    > ARW_normalised = ARW_raw / √BW

    This removes the effect of different filter bandwidths across sensors so white noise floors
    are directly comparable. The MPU6050 DLPF cannot be disabled (BW≈260Hz), while ICM20948
    uses NBW≈229.8Hz. Normalisation makes both directly comparable.

    **WT901 Note:**
    > WT901 is logged at 1kHz but outputs unique data at 200Hz.
    > Only unique samples (every 5th) are used for Allan Variance — feeding repeated
    > values would artificially suppress the noise floor.

    **Bias Instability:**
    > Independent of filter bandwidth. Directly comparable across all sensors without normalisation.
    > Represents the minimum achievable bias error — a fundamental property of the MEMS sensor itself.
    """)
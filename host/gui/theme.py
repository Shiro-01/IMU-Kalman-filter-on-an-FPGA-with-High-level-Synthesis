"""
theme.py

Dark, avionics/HUD-style palette and stylesheet, applied globally so the
dashboard doesn't look like a generic light Qt app.
"""

BG = "#0a0e14"
PANEL_BG = "#10151d"
PANEL_BORDER = "#233042"
TEXT = "#d7e4ee"
TEXT_DIM = "#7a8ba0"
ACCENT = "#00d4ff"       # HUD cyan
ACCENT_WARN = "#ffb000"  # amber, used for the "software" source
ACCENT_BAD = "#ff4d4d"
ACCENT_RMSE = "#b892ff"  # violet, reserved for derived/aggregate RMSE traces
GRID = "#1c2733"

STYLESHEET = f"""
QWidget {{
    background-color: {BG};
    color: {TEXT};
    font-family: "SF Mono", "Menlo", "Consolas", monospace;
    font-size: 12px;
}}
QMainWindow {{
    background-color: {BG};
}}
QGroupBox {{
    background-color: {PANEL_BG};
    border: 1px solid {PANEL_BORDER};
    border-radius: 4px;
    margin-top: 14px;
    font-weight: bold;
    color: {ACCENT};
    letter-spacing: 1px;
    text-transform: uppercase;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}}
QLabel {{
    color: {TEXT};
}}
QLabel[role="value"] {{
    color: {ACCENT};
    font-size: 14px;
    font-weight: bold;
}}
QLabel[role="hint"] {{
    color: {TEXT_DIM};
    font-size: 10px;
}}
QPushButton {{
    background-color: #16202c;
    border: 1px solid {PANEL_BORDER};
    border-radius: 3px;
    padding: 5px 12px;
    color: {TEXT};
}}
QPushButton:hover {{
    border-color: {ACCENT};
    color: {ACCENT};
}}
QPushButton:disabled {{
    color: {TEXT_DIM};
    border-color: {PANEL_BORDER};
}}
QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{
    background-color: #0d131a;
    border: 1px solid {PANEL_BORDER};
    border-radius: 3px;
    padding: 3px 6px;
    color: {TEXT};
}}
QComboBox:focus, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {ACCENT};
}}
QComboBox::drop-down {{
    border: none;
}}
/* The popup list is a separate top-level view (QAbstractItemView) that does
   not inherit QComboBox's own color rules above -- without this block its
   item text renders in the platform's default color, which can be
   invisible against a dark popup background. */
QComboBox QAbstractItemView {{
    background-color: #0d131a;
    border: 1px solid {PANEL_BORDER};
    color: {TEXT};
    selection-background-color: {ACCENT};
    selection-color: #05131a;
    outline: none;
}}
QStatusBar {{
    background-color: {PANEL_BG};
    color: {TEXT_DIM};
    border-top: 1px solid {PANEL_BORDER};
}}
QSplitter::handle {{
    background-color: {PANEL_BORDER};
}}
"""

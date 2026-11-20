"""Rover-only palette, retaining the ROV's established spacing and components."""

from PySide6.QtGui import QColor
from ROV_Control_Station_Integrated import QSS, PadButton

ACCENT = "#f37669"
PLOT_COLORS = tuple(QColor(color) for color in ("#f37669", "#efb16a", "#d49ac8"))
ROVER_QSS = QSS
for original, replacement in (
    ("#5fd0ff", ACCENT), ("#2dd4bf", "#dd665d"), ("#14b8a6", "#b74b48"),
    ("#2563eb", "#a63f42"), ("#60a5fa", "#ff998a"), ("#1d4ed8", "#8a343a"),
    ("#090e14", "#0e1015"), ("#0e1622", "#161218"),
):
    ROVER_QSS = ROVER_QSS.replace(original, replacement)
ROVER_QSS += """
#topbar { background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #211820, stop:1 #141720);
          border-bottom:1px solid #60363c; }
QGroupBox { background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #1a1b24, stop:1 #141720); }
QTabBar::tab:selected { color:#ff9385; border-top:2px solid #bf5754; }
QPushButton#preview:checked { background:#773b36; border-color:#f37669; color:#ffe3d9; }
QPushButton#camtoggle { background:#38252c; border-color:#754047; color:#ffac9e; }
QPushButton#camtoggle:hover { background:#4a2e35; }
QPushButton#camtoggle:checked { background:#81413f; border-color:#f37669; }
QTableWidget { background:#10141c; alternate-background-color:#181b24; gridline-color:#2c303d; }
QHeaderView::section { background:#25222d; padding:8px; border:1px solid #34303b; color:#d9b3ad; }
QPushButton#motorTestKey { background:#38242c; border:1px solid #744047; color:#ffab9e; }
QPushButton#estop:disabled { background:#332329; border-color:#65404a; color:#ab7a82; }
"""


def pad_button(kind, title, subtitle="", checkable=False):
    button = PadButton(kind, title, subtitle, checkable)
    button.accent = QColor("#fb555d" if kind == "stop" else ACCENT)
    return button

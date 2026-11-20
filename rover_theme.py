"""Mars Rover palette, QSS stylesheet rules, and Vehicle Selector theme."""

from PySide6.QtGui import QColor
from ROV_Control_Station_Integrated import QSS, PadButton


ACCENT = "#f37669"
ACCENT_SOFT = "#ff998a"
WARN = "#fbbf24"
OK = "#4ade80"
DANGER = "#ef4444"

PLOT_COLORS = tuple(QColor(color) for color in ("#f37669", "#efb16a", "#d49ac8"))

ROVER_QSS = QSS
for original, replacement in (
    ("#5fd0ff", ACCENT),
    ("#2dd4bf", "#dd665d"),
    ("#14b8a6", "#b74b48"),
    ("#2563eb", "#a63f42"),
    ("#60a5fa", ACCENT_SOFT),
    ("#1d4ed8", "#8a343a"),
    ("#090e14", "#0e1015"),
    ("#0e1622", "#161218"),
):
    ROVER_QSS = ROVER_QSS.replace(original, replacement)

ROVER_QSS += """
#topbar { background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #231820, stop:1 #131720);
          border-bottom:1px solid #60363c; }
QGroupBox { background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #1a1b24, stop:1 #141720); }
QTabBar::tab { padding:8px 12px; }
QTabBar::tab:selected { color:#ff9385; border-top:2px solid #bf5754; }
QPushButton#topbarBtn { padding:6px 10px; font-size:12px; }
QPushButton#mapBtn { padding:5px 9px; font-size:12px; }
QPushButton#mapBtn:checked { background:#773b36; border-color:#f37669; color:#ffe3d9; font-weight:700; }
QPushButton#preview { padding:6px 10px; font-size:12px; }
QPushButton#preview:checked { background:#773b36; border-color:#f37669; color:#ffe3d9; font-weight:700; }
QPushButton#rosTeleop:checked { background:#6b2e2e; border-color:#fbbf24; color:#fef08a; font-weight:700; }
QPushButton#arm { padding:6px 12px; font-size:12px; background:#2a1f28; border:1px solid #6d4047; color:#f1c5bd; font-weight:700; }
QPushButton#arm:hover:enabled { background:#3d2730; border-color:#f37669; }
QPushButton#arm:checked { background:#8f2b24; border:2px solid #fbbf24; color:#fef08a; font-weight:800; }
QPushButton#arm:disabled { background:#1d1d26; border-color:#383644; color:#696a7a; }
QPushButton#tool { padding:6px 10px; font-size:12px; }
QPushButton#estop { padding:6px 12px; font-size:12px; background:#991b1b; border:2px solid #ef4444; color:#fee2e2; font-weight:800; }
QPushButton#estop:hover:enabled { background:#b91c1c; border-color:#fca5a5; }
QPushButton#estop:disabled { background:#332329; border-color:#65404a; color:#ab7a82; }
QPushButton#camtoggle { background:#38252c; border-color:#754047; color:#ffac9e; }
QPushButton#camtoggle:hover { background:#4a2e35; }
QPushButton#camtoggle:checked { background:#81413f; border-color:#f37669; }
QPushButton#filterBtn { padding:3px 10px; font-size:12px; }
QPushButton#filterBtn:checked { background:#5b2d30; border-color:#f37669; color:#ffe3d9; }
QTableWidget { background:#10141c; alternate-background-color:#181b24; gridline-color:#2c303d; }
QHeaderView::section { background:#25222d; padding:6px 8px; border:1px solid #34303b; color:#d9b3ad; }
QPushButton#motorTestKey { background:#38242c; border:1px solid #744047; color:#ffab9e; font-weight:700; }
QPushButton#motorTestKey[active="true"] { background:#8f3a36; border:2px solid #fbbf24; color:#fef08a; }
QLabel#slipAlert { background:#451a1d; border:1px solid #ef4444; border-radius:6px; color:#fca5a5; padding:4px 8px; }
"""

SELECTOR_QSS = """
QWidget#selectorRoot { background:#0e1015; color:#e6edf3; font-size:14px; }
QLabel#selectorTitle { font-size:26px; font-weight:800; color:#f8fafc; }
QLabel#selectorSub { font-size:13px; color:#94a3b8; }
QFrame#roverCard {
    background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #231820, stop:1 #141720);
    border:1px solid #744047; border-radius:12px;
}
QFrame#rovCard {
    background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #14222e, stop:1 #101722);
    border:1px solid #2b4f68; border-radius:12px;
}
QPushButton#launchRover {
    background:#8a343a; border:1px solid #f37669; color:#ffe9e4;
    padding:12px; border-radius:8px; font-weight:700; font-size:14px;
}
QPushButton#launchRover:hover { background:#a63f42; }
QPushButton#launchRov {
    background:#175061; border:1px solid #2dd4bf; color:#e6fffa;
    padding:12px; border-radius:8px; font-weight:700; font-size:14px;
}
QPushButton#launchRov:hover { background:#1d667a; }
QPushButton#launchRov:disabled { background:#18232e; border-color:#2a3a4f; color:#7b8793; }
QLabel#depOk {
    background:#12301d; border:1px solid #1f7a45; color:#4ade80;
    border-radius:10px; padding:3px 9px; font-size:11px; font-weight:700;
}
QLabel#depMiss {
    background:#2a1f16; border:1px solid #7c5819; color:#fbbf24;
    border-radius:10px; padding:3px 9px; font-size:11px; font-weight:700;
}
"""


def pad_button(kind: str, title: str, subtitle: str = "", checkable: bool = False) -> PadButton:
    """Create a PadButton styled with the Rover Mars-red or stop accent."""
    button = PadButton(kind, title, subtitle, checkable)
    button.accent = QColor("#fb555d" if kind == "stop" else ACCENT)
    return button

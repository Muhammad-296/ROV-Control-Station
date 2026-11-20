"""Cross-platform MESEKET vehicle selector for Mars Rover and ROV control stations."""

import importlib.util
from pathlib import Path
import sys

from PySide6.QtCore import QProcess, Qt
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QMessageBox, QPushButton,
    QVBoxLayout, QWidget,
)

from rover_dashboard import RoverDashboard
from rover_theme import ACCENT, SELECTOR_QSS


def probe_dependencies() -> list[tuple[str, bool]]:
    """Detect installed optional runtime modules without side effects."""
    modules = (
        ("PySide6", "PySide6"),
        ("OpenCV", "cv2"),
        ("Pygame", "pygame"),
        ("VTK 3D", "vtk"),
        ("ROS 2", "rclpy"),
    )
    return [(label, importlib.util.find_spec(mod) is not None) for label, mod in modules]


class VehicleSelector(QWidget):
    """Launcher window switching between Mars Rover and ROV dashboards."""

    def __init__(self):
        super().__init__()
        self.setObjectName("selectorRoot")
        self.setStyleSheet(SELECTOR_QSS)
        self.setWindowTitle("MESEKET Control Station")
        self.resize(720, 420)
        self.setMinimumSize(620, 360)
        self.rover: RoverDashboard | None = None
        self.rov_process = QProcess(self)
        self.rov_process.setProcessChannelMode(QProcess.ProcessChannelMode.ForwardedChannels)
        self.rov_process.started.connect(self.hide)
        self.rov_process.finished.connect(self.rov_finished)
        self.rov_process.errorOccurred.connect(self.rov_error)

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 22)
        root.setSpacing(16)

        header = QVBoxLayout()
        header.setSpacing(4)
        title = QLabel(f"MESEKET <span style='color:{ACCENT}'>CONTROL STATION</span>")
        title.setObjectName("selectorTitle")
        sub = QLabel("Select a vehicle station. Each vehicle keeps independent settings and safety interlocks.")
        sub.setObjectName("selectorSub")
        sub.setWordWrap(True)
        header.addWidget(title)
        header.addWidget(sub)
        root.addLayout(header)

        cards = QHBoxLayout()
        cards.setSpacing(16)

        # Mars Rover card
        rover_card = QFrame()
        rover_card.setObjectName("roverCard")
        rl = QVBoxLayout(rover_card)
        rl.setContentsMargins(18, 18, 18, 18)
        rl.setSpacing(10)
        r_title = QLabel(f"<b style='font-size:17px; color:{ACCENT};'>MARS ROVER</b>")
        r_desc = QLabel(
            "• 4WD / 4WS kinematics & input preview\n"
            "• ROS 2 encoders, IMU, slip & diagnostics\n"
            "• 2D SLAM map, Nav2 path & ArUco markers\n"
            "• Gated /cmd_vel + /goal_pose with E-STOP"
        )
        r_desc.setWordWrap(True)
        r_desc.setStyleSheet("color:#cbd5e1; font-size:13px; line-height:1.4;")
        self.rover_button = QPushButton("Launch Mars Rover Station")
        self.rover_button.setObjectName("launchRover")
        self.rover_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rover_button.clicked.connect(self.open_rover)
        rl.addWidget(r_title)
        rl.addWidget(r_desc, 1)
        rl.addWidget(self.rover_button)
        cards.addWidget(rover_card, 1)

        # ROV card
        rov_card = QFrame()
        rov_card.setObjectName("rovCard")
        vl = QVBoxLayout(rov_card)
        vl.setContentsMargins(18, 18, 18, 18)
        vl.setSpacing(10)
        v_title = QLabel("<b style='font-size:17px; color:#2dd4bf;'>UNDERWATER ROV</b>")
        v_desc = QLabel(
            "• Original 6-thruster underwater control\n"
            "• Arduino Serial V5 protocol & heartbeat\n"
            "• Direct RTSP camera & water temperature\n"
            "• Unchanged hardware & protocol behavior"
        )
        v_desc.setWordWrap(True)
        v_desc.setStyleSheet("color:#cbd5e1; font-size:13px; line-height:1.4;")
        self.rov_button = QPushButton("Launch ROV Control Station")
        self.rov_button.setObjectName("launchRov")
        self.rov_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rov_button.clicked.connect(self.open_rov)
        vl.addWidget(v_title)
        vl.addWidget(v_desc, 1)
        vl.addWidget(self.rov_button)
        cards.addWidget(rov_card, 1)

        root.addLayout(cards, 1)

        # Environment dependency badges
        dep_row = QHBoxLayout()
        dep_row.setSpacing(8)
        dep_lbl = QLabel("Environment:")
        dep_lbl.setStyleSheet("color:#94a3b8; font-size:12px; font-weight:700;")
        dep_row.addWidget(dep_lbl)
        for name, ok in probe_dependencies():
            badge = QLabel(f"{'●' if ok else '○'} {name}")
            badge.setObjectName("depOk" if ok else "depMiss")
            badge.setToolTip(f"{name}: {'available' if ok else 'not detected in current Python environment'}")
            dep_row.addWidget(badge)
        dep_row.addStretch()
        root.addLayout(dep_row)

    def open_rover(self):
        if self.rover is None:
            self.rover = RoverDashboard()
            self.rover.closed.connect(self.show)
        self.rover.show()
        self.rover.raise_()
        self.hide()

    def open_rov(self):
        root = Path(__file__).resolve().parent
        self.rov_button.setEnabled(False)
        self.rov_process.setWorkingDirectory(str(root))
        self.rov_process.start(sys.executable, [str(root / "ROV_Control_Station_Integrated.py")])

    def rov_finished(self, exit_code, exit_status):
        self.rov_button.setEnabled(True)
        self.show()
        if exit_code or exit_status == QProcess.ExitStatus.CrashExit:
            QMessageBox.warning(
                self,
                "ROV closed unexpectedly",
                "The ROV application exited with an error. Check the terminal and "
                "the existing ROV error log. Install requirements.txt if dependencies are missing.",
            )

    def rov_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.rov_button.setEnabled(True)
            self.show()
            QMessageBox.warning(self, "Could not start ROV", self.rov_process.errorString())

    def closeEvent(self, event):
        if self.rover is not None:
            self.rover.final_shutdown()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(SELECTOR_QSS)
    selector = VehicleSelector()
    selector.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

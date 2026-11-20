"""Cross-platform vehicle selector. Run with Python and PySide6 installed."""

from pathlib import Path
import sys

from PySide6.QtCore import QProcess
from PySide6.QtWidgets import (
    QApplication, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from rover_dashboard import RoverDashboard


STYLE = """
QWidget { background: #101923; color: #e6edf3; font-size: 14px; }
QLabel#title { font-size: 28px; font-weight: bold; }
QLabel#status { color: #ffbe76; }
QPushButton { background: #22364a; padding: 14px; border: 1px solid #46627c;
              border-radius: 6px; }
QPushButton:hover { background: #304d68; }
QPushButton:disabled { color: #7b8793; background: #18232e; }
QLineEdit, QComboBox, QTableWidget { background: #182735; padding: 6px; }
QHeaderView::section { background: #22364a; padding: 8px; }
QTabBar::tab { background: #22364a; padding: 12px; }
QTabBar::tab:selected { background: #375a77; }
QGroupBox { border: 1px solid #46627c; margin-top: 12px; padding: 16px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; }
"""


class VehicleSelector(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MESEKET Control Station")
        self.resize(520, 340)
        self.rover = None
        self.rov_process = QProcess(self)
        self.rov_process.setProcessChannelMode(QProcess.ProcessChannelMode.ForwardedChannels)
        self.rov_process.started.connect(self.hide)
        self.rov_process.finished.connect(self.rov_finished)
        self.rov_process.errorOccurred.connect(self.rov_error)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(32, 24, 32, 24)
        title = QLabel("Choose your vehicle")
        title.setObjectName("title")
        layout.addWidget(title)
        layout.addWidget(QLabel("MESEKET operator control station"))
        layout.addSpacing(16)
        rover_button = QPushButton("Mars Rover  ·  Navigation and telemetry")
        rover_button.clicked.connect(self.open_rover)
        layout.addWidget(rover_button)
        self.rov_button = QPushButton("ROV  ·  Existing underwater control station")
        self.rov_button.clicked.connect(self.open_rov)
        layout.addWidget(self.rov_button)
        note = QLabel("Rover: offline setup available. ROV: existing hardware controls.")
        note.setWordWrap(True)
        layout.addWidget(note)

    def open_rover(self):
        if self.rover is None:
            self.rover = RoverDashboard()
            self.rover.closed.connect(self.show)
        self.rover.show()
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
                self, "ROV closed unexpectedly",
                "The ROV application exited with an error. Check the terminal and "
                "the existing ROV error log. Install requirements.txt if dependencies are missing.",
            )

    def rov_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.rov_button.setEnabled(True)
            self.show()
            QMessageBox.warning(self, "Could not start ROV", self.rov_process.errorString())


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    selector = VehicleSelector()
    selector.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

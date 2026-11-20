"""Rover RTSP controls using the existing ROV capture and video widgets."""

from pathlib import Path
import time

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)
from ROV_Control_Station_Integrated import (
    CameraThread, CameraView, camera_url, camera_credentials_error, validate_camera_url,
)


class RoverCamera(QWidget):
    frame = Signal(object)
    state = Signal(str, str)

    def __init__(self, settings, owner):
        super().__init__()
        self.settings, self.owner = settings, owner
        self.thread = None
        self.accept_frames = False
        self.last_kind = "idle"
        self.image = None
        self.window = None
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.host = QLineEdit(settings.value("camera/host", ""))
        self.host.setPlaceholderText("Camera IP address")
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(int(settings.value("camera/port", 554)))
        self.path = QLineEdit(settings.value("camera/path", "profile1"))
        self.user = QLineEdit()
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.EchoMode.Password)
        self.tcp = QCheckBox("Use TCP transport")
        self.tcp.setChecked(settings.value("camera/tcp", True, type=bool))
        for title, field in (("Camera IP", self.host), ("Port", self.port), ("Stream path", self.path),
                             ("Username", self.user), ("Password", self.password)):
            form.addRow(title, field)
        form.addRow(self.tcp)
        layout.addLayout(form)
        self.status = QLabel("Camera off · credentials are never saved")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        for title, callback in (("Connect / reconnect", self.start), ("Stop stream", self.stop),
                                ("Open camera window", self.open_window), ("Save snapshot", self.snapshot)):
            button = QPushButton(title)
            button.clicked.connect(callback)
            layout.addWidget(button)
        layout.addStretch()

    def broadcast(self, kind, text):
        self.last_kind = kind
        self.status.setText(text or ("Live stream" if kind == "live" else kind))
        self.state.emit(kind, text)
        if self.window:
            self.detached_view.set_state(kind, text)
        if kind != "live":
            self.image = None

    def start(self):
        if self.thread is not None and self.thread.isRunning():
            self.stop()
            self.status.setText("Stopping previous stream… press Connect again once stopped.")
            return
        url = camera_url(self.host.text(), self.port.value(), self.path.text(),
                         self.user.text(), self.password.text())
        problem = camera_credentials_error(self.user.text(), self.password.text()) or validate_camera_url(url)
        if not self.host.text().strip():
            problem = "Enter the rover camera IP address."
        if problem:
            self.broadcast("idle", problem)
            return
        for key, value in (("host", self.host.text().strip()), ("port", self.port.value()),
                           ("path", self.path.text().strip()), ("tcp", self.tcp.isChecked())):
            self.settings.setValue(f"camera/{key}", value)
        self.accept_frames = True
        thread = CameraThread(url, self.tcp.isChecked())
        self.thread = thread
        thread.frame.connect(self.receive_frame)
        thread.state.connect(self.receive_state)
        thread.finished.connect(lambda: self.finished(thread))
        self.broadcast("connecting", "Connecting to rover camera…")
        thread.start()

    def receive_state(self, kind, text):
        if self.accept_frames:
            self.broadcast(kind, text)

    def receive_frame(self, image):
        if not self.accept_frames:
            return
        self.image = image
        self.frame.emit(image)
        if self.window:
            self.detached_view.set_frame(image)

    def finished(self, thread):
        if self.thread is thread:
            self.thread = None
            self.accept_frames = False
            if self.last_kind != "error":
                self.broadcast("idle", "Camera stopped")
        thread.deleteLater()

    def stop(self):
        self.accept_frames = False
        if self.thread is not None:
            self.thread.requestInterruption()
        self.broadcast("idle", "Camera stopped")

    def open_window(self):
        if self.window is None:
            self.window = QMainWindow(self.owner)
            self.window.setWindowTitle("Mars Rover · Camera monitor")
            self.window.resize(960, 650)
            central = QWidget()
            layout = QVBoxLayout(central)
            row = QHBoxLayout()
            for title, callback in (("Connect", self.start), ("Stop", self.stop),
                                    ("Snapshot", self.snapshot), ("Fullscreen", self.fullscreen)):
                button = QPushButton(title)
                button.clicked.connect(callback)
                row.addWidget(button)
            layout.addLayout(row)
            self.detached_view = CameraView()
            self.detached_view.set_state("idle", "Connect from the Camera settings tab")
            layout.addWidget(self.detached_view, 1)
            self.window.setCentralWidget(central)
        if self.image is not None:
            self.detached_view.set_frame(self.image)
        self.window.show()
        self.window.raise_()

    def fullscreen(self):
        if self.window.isFullScreen():
            self.window.showNormal()
        else:
            self.window.showFullScreen()

    def snapshot(self):
        if self.image is None:
            self.status.setText("No live frame available to save.")
            return
        suggested = Path.home() / ("rover_camera_" + time.strftime("%Y%m%d_%H%M%S") + ".png")
        path, _ = QFileDialog.getSaveFileName(self.owner, "Save rover camera snapshot", str(suggested), "PNG (*.png)")
        if path:
            self.status.setText("Snapshot saved." if self.image.save(path) else "Could not save snapshot.")

    def shutdown(self):
        self.stop()
        if self.thread is not None and self.thread.isRunning():
            return False
        if self.window:
            self.window.close()
        return True

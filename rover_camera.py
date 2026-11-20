"""Unified Rover Camera controller supporting ROS 2 image streams and Direct RTSP."""

from pathlib import Path
import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from ROV_Control_Station_Integrated import (
    CameraThread, CameraView, camera_url, mask_rtsp_url, validate_camera_url,
)
from rover_theme import ROVER_QSS


CAMERA_SOURCES = (
    ("ROS 2 · RGB Camera", "rgb"),
    ("ROS 2 · Depth Heatmap", "depth"),
    ("ROS 2 · ArUco Debug", "aruco_debug"),
    ("ROS 2 · Pose Debug", "pose_debug"),
    ("Direct RTSP Stream", "rtsp"),
)


def rtsp_credentials_error(username: str, password: str) -> str:
    """Allow anonymous RTSP (both blank) or complete login; reject partial credentials."""
    u = username.strip()
    p = password.strip()
    if bool(u) != bool(p):
        return "Enter both username and password, or leave both blank for anonymous RTSP."
    return ""


class RoverCamera(QWidget):
    """Camera source manager, ROS/RTSP switcher, and detached monitor window."""

    frame = Signal(object)
    state = Signal(str, str)
    ros_feed_changed = Signal(object)  # str | None

    def __init__(self, settings, owner):
        super().__init__()
        self.settings, self.owner = settings, owner
        self.thread: CameraThread | None = None
        self.accept_frames = False
        self._pending_restart = False
        self._active_ros_feed: str | None = None
        self._ros_connected = False
        self.last_kind = "idle"
        self.image = None
        self.window: QMainWindow | None = None
        self.detached_view: CameraView | None = None
        self.detached_source: QComboBox | None = None
        self.detached_snapshot_btn: QPushButton | None = None
        self.detached_full_btn: QPushButton | None = None

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.source_combo = QComboBox()
        for label, key in CAMERA_SOURCES:
            self.source_combo.addItem(label, key)
        saved_src = settings.value("camera/source", "rgb")
        idx = self.source_combo.findData(saved_src)
        if idx >= 0:
            self.source_combo.setCurrentIndex(idx)
        self.source_combo.currentIndexChanged.connect(self._on_source_changed)
        form.addRow("Video source", self.source_combo)

        self.host = QLineEdit(settings.value("camera/host", ""))
        self.host.setPlaceholderText("Camera IP (RTSP mode)")
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(int(settings.value("camera/port", 554)))

        self.path = QComboBox()
        self.path.setEditable(True)
        self.path.addItems(["profile1", "profile2", "Streaming/Channels/101"])
        self.path.setCurrentText(settings.value("camera/path", "profile1"))

        self.user = QLineEdit()
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.EchoMode.Password)
        self.reveal = QCheckBox("Show password")
        self.reveal.toggled.connect(
            lambda show: self.password.setEchoMode(
                QLineEdit.EchoMode.Normal if show else QLineEdit.EchoMode.Password
            )
        )

        self.override = QLineEdit(settings.value("camera/override", ""))
        self.override.setPlaceholderText("Optional full rtsp:// URL override")

        self.tcp = QCheckBox("Use TCP transport")
        self.tcp.setChecked(settings.value("camera/tcp", True, type=bool))

        for title, field in (
            ("Camera IP", self.host),
            ("RTSP port", self.port),
            ("Stream path", self.path),
            ("Username", self.user),
            ("Password", self.password),
        ):
            form.addRow(title, field)
        form.addRow("", self.reveal)
        form.addRow("Full URL", self.override)
        form.addRow(self.tcp)
        layout.addLayout(form)

        self.url_preview = QLabel("")
        self.url_preview.setWordWrap(True)
        self.url_preview.setStyleSheet("color:#8b9bb0; font-size:12px;")
        layout.addWidget(self.url_preview)

        self.status = QLabel("Camera off · select ROS or RTSP source and press Connect")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        for title, callback in (
            ("Connect / reconnect", self.start),
            ("Stop stream", self.stop),
            ("Open camera window", self.open_window),
        ):
            button = QPushButton(title)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.clicked.connect(callback)
            layout.addWidget(button)

        self.snapshot_btn = QPushButton("Save snapshot")
        self.snapshot_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.snapshot_btn.setEnabled(False)
        self.snapshot_btn.clicked.connect(self.snapshot)
        layout.addWidget(self.snapshot_btn)
        layout.addStretch()

        for widget in (self.host, self.user, self.password, self.override):
            widget.textChanged.connect(self._update_url_preview)
        self.port.valueChanged.connect(self._update_url_preview)
        self.path.editTextChanged.connect(self._update_url_preview)
        self._update_url_preview()

    def current_source(self) -> str:
        return str(self.source_combo.currentData() or "rgb")

    def set_ros_connected(self, connected: bool) -> None:
        self._ros_connected = bool(connected)
        if not self._ros_connected and self._active_ros_feed is not None:
            self._active_ros_feed = None
            self.ros_feed_changed.emit(None)
            self.broadcast("idle", "ROS 2 disconnected · camera stream stopped")

    def _on_source_changed(self):
        src = self.current_source()
        self.settings.setValue("camera/source", src)
        if self.detached_source is not None:
            idx = self.detached_source.findData(src)
            if idx >= 0 and self.detached_source.currentIndex() != idx:
                self.detached_source.blockSignals(True)
                self.detached_source.setCurrentIndex(idx)
                self.detached_source.blockSignals(False)
        self._update_url_preview()
        # If actively streaming, switch immediately to the new source.
        if self._active_ros_feed is not None or (self.thread is not None and self.thread.isRunning()):
            self.start()

    def _on_detached_source_changed(self):
        if self.detached_source is None:
            return
        src = self.detached_source.currentData()
        idx = self.source_combo.findData(src)
        if idx >= 0 and self.source_combo.currentIndex() != idx:
            self.source_combo.setCurrentIndex(idx)

    def _build_rtsp_url(self, mask: bool = False) -> tuple[str, str]:
        override = self.override.text().strip()
        host = self.host.text().strip()
        if not override and not host:
            return "", "Enter the rover camera IP address or a full RTSP URL."
        cred_err = rtsp_credentials_error(self.user.text(), self.password.text())
        if cred_err:
            return "", cred_err
        url = camera_url(
            host,
            self.port.value(),
            self.path.currentText(),
            self.user.text(),
            self.password.text(),
            override,
            mask=mask,
        )
        # camera_url uses mask_rtsp_url or preserves embedded override info cleanly.
        if mask and override:
            url = mask_rtsp_url(url)
        valid_err = validate_camera_url(url)
        if valid_err:
            return url, valid_err
        return url, ""

    def _update_url_preview(self, *_):
        src = self.current_source()
        rtsp_fields = (self.host, self.port, self.path, self.user, self.password, self.reveal, self.override, self.tcp)
        for w in rtsp_fields:
            w.setEnabled(src == "rtsp")
        if src != "rtsp":
            label = self.source_combo.currentText()
            self.url_preview.setText(f"ROS Image transport active ({label}).")
            return
        url, problem = self._build_rtsp_url(mask=True)
        if problem:
            self.url_preview.setText(problem)
        else:
            self.url_preview.setText(f"Source: {url}")

    def _update_snapshot_buttons(self):
        can_save = self.image is not None and self.last_kind == "live"
        self.snapshot_btn.setEnabled(can_save)
        if self.detached_snapshot_btn is not None:
            self.detached_snapshot_btn.setEnabled(can_save)

    def broadcast(self, kind: str, text: str):
        self.last_kind = kind
        self.status.setText(text or ("Live stream" if kind == "live" else kind))
        self.state.emit(kind, text)
        if self.detached_view is not None:
            self.detached_view.set_state(kind, text)
        if kind != "live":
            self.image = None
        self._update_snapshot_buttons()

    def start(self):
        src = self.current_source()

        # Stop any active RTSP capture first before switching or reconnecting.
        if self.thread is not None and self.thread.isRunning():
            self._pending_restart = True
            self.accept_frames = False
            self.thread.requestInterruption()
            self.broadcast("connecting", "Switching / reconnecting camera stream…")
            return

        self._pending_restart = False

        if src != "rtsp":
            if not self._ros_connected:
                self._active_ros_feed = None
                self.ros_feed_changed.emit(None)
                self.broadcast("idle", "Connect ROS 2 first to view ROS camera topics.")
                return
            self._active_ros_feed = src
            self.accept_frames = True
            self.ros_feed_changed.emit(src)
            self.broadcast("connecting", f"Subscribed to {self.source_combo.currentText()} · waiting for frames…")
            return

        # RTSP mode: ensure ROS camera feed is detached.
        if self._active_ros_feed is not None:
            self._active_ros_feed = None
            self.ros_feed_changed.emit(None)

        url, problem = self._build_rtsp_url(mask=False)
        if problem:
            self.broadcast("idle", problem)
            return

        for key, value in (
            ("host", self.host.text().strip()),
            ("port", self.port.value()),
            ("path", self.path.currentText().strip()),
            ("override", self.override.text().strip()),
            ("tcp", self.tcp.isChecked()),
        ):
            self.settings.setValue(f"camera/{key}", value)

        self.accept_frames = True
        thread = CameraThread(url, self.tcp.isChecked())
        self.thread = thread
        thread.frame.connect(self.receive_frame)
        thread.state.connect(self.receive_state)
        thread.finished.connect(lambda t=thread: self.finished(t))
        self.broadcast("connecting", "Connecting to rover RTSP camera…")
        thread.start()

    def receive_ros_frame(self, feed_key: str, image) -> None:
        if not self.accept_frames or feed_key != self._active_ros_feed:
            return
        self.receive_frame(image)
        if self.last_kind != "live":
            self.last_kind = "live"
            self.status.setText(f"Live · {self.source_combo.currentText()}")
            self.state.emit("live", "")
            self._update_snapshot_buttons()

    def receive_state(self, kind: str, text: str):
        if self.accept_frames and self._active_ros_feed is None:
            self.broadcast(kind, text)

    def receive_frame(self, image):
        if not self.accept_frames:
            return
        self.image = image
        self.last_kind = "live"
        self.frame.emit(image)
        if self.detached_view is not None:
            self.detached_view.set_frame(image)
        self._update_snapshot_buttons()

    def finished(self, thread):
        restart = self._pending_restart
        self._pending_restart = False
        if self.thread is thread:
            self.thread = None
            if not restart:
                self.accept_frames = False
                if self.last_kind != "error":
                    self.broadcast("idle", "Camera stopped")
        thread.deleteLater()
        if restart:
            self.start()

    def stop(self):
        self._pending_restart = False
        self.accept_frames = False
        if self._active_ros_feed is not None:
            self._active_ros_feed = None
            self.ros_feed_changed.emit(None)
        if self.thread is not None:
            self.thread.requestInterruption()
        self.broadcast("idle", "Camera stopped")

    def open_window(self):
        if self.window is None:
            self.window = QMainWindow(None)
            self.window.setStyleSheet(ROVER_QSS)
            self.window.setWindowTitle("Mars Rover · Live Camera Monitor")
            self.window.resize(980, 660)
            self.window.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, False)

            central = QWidget()
            layout = QVBoxLayout(central)
            row = QHBoxLayout()

            self.detached_source = QComboBox()
            for label, key in CAMERA_SOURCES:
                self.detached_source.addItem(label, key)
            self.detached_source.setCurrentIndex(self.source_combo.currentIndex())
            self.detached_source.currentIndexChanged.connect(self._on_detached_source_changed)
            row.addWidget(self.detached_source, 1)

            for title, callback in (("Connect", self.start), ("Stop", self.stop)):
                button = QPushButton(title)
                button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                button.clicked.connect(callback)
                row.addWidget(button)

            self.detached_snapshot_btn = QPushButton("Snapshot")
            self.detached_snapshot_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self.detached_snapshot_btn.clicked.connect(self.snapshot)
            row.addWidget(self.detached_snapshot_btn)

            self.detached_full_btn = QPushButton("Fullscreen  [F11]")
            self.detached_full_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self.detached_full_btn.clicked.connect(self.toggle_fullscreen)
            row.addWidget(self.detached_full_btn)

            layout.addLayout(row)
            self.detached_view = CameraView()
            self.detached_view.set_state(self.last_kind, self.status.text())
            layout.addWidget(self.detached_view, 1)
            self.window.setCentralWidget(central)

            QShortcut(QKeySequence("F11"), self.window).activated.connect(self.toggle_fullscreen)
            QShortcut(QKeySequence("Escape"), self.window).activated.connect(self._exit_fullscreen)

        if self.image is not None and self.last_kind == "live":
            self.detached_view.set_frame(self.image)
        self._update_snapshot_buttons()
        self.window.show()
        self.window.raise_()
        self.window.activateWindow()

    def _exit_fullscreen(self):
        if self.window is not None and self.window.isFullScreen():
            self.window.showNormal()
            if self.detached_full_btn is not None:
                self.detached_full_btn.setText("Fullscreen  [F11]")

    def toggle_fullscreen(self):
        if self.window is None:
            return
        if self.window.isFullScreen():
            self.window.showNormal()
            if self.detached_full_btn is not None:
                self.detached_full_btn.setText("Fullscreen  [F11]")
        else:
            self.window.showFullScreen()
            if self.detached_full_btn is not None:
                self.detached_full_btn.setText("Windowed  [F11]")

    def save_snapshot_to(self, path: str) -> bool:
        if self.image is None:
            return False
        return bool(self.image.save(path))

    def snapshot(self):
        if self.image is None:
            self.status.setText("No live frame available to save.")
            return
        suggested = Path.home() / ("rover_camera_" + time.strftime("%Y%m%d_%H%M%S") + ".png")
        path, _ = QFileDialog.getSaveFileName(self.owner, "Save rover camera snapshot", str(suggested), "PNG (*.png)")
        if path:
            self.status.setText("Snapshot saved." if self.save_snapshot_to(path) else "Could not save snapshot.")

    def shutdown(self, timeout_ms: int = 1000) -> bool:
        self._pending_restart = False
        self.accept_frames = False
        if self._active_ros_feed is not None:
            self._active_ros_feed = None
            self.ros_feed_changed.emit(None)
        if self.thread is not None and self.thread.isRunning():
            self.thread.requestInterruption()
            self.thread.wait(timeout_ms)
        if self.window is not None:
            self._exit_fullscreen()
            self.window.close()
        return self.thread is None or not self.thread.isRunning()

"""Rover dashboard with optional read-only ROS telemetry; no motion commands."""

import math
import time

from PySide6.QtCore import QSettings, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFormLayout, QFrame, QGridLayout, QGroupBox, QHeaderView, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPushButton, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget, QPlainTextEdit, QSlider, QStackedWidget,
)


from ROV_Control_Station_Integrated import (
    AttitudeWidget, AxisBar, CameraView, CompassWidget, PadButton, Pill,
    PlotWidget, PS4Monitor, QSS, STLViewer,
    StatCard, StickWidget, make_scroll, mono_font, setup_fonts,
)
from rover_view import DRIVE_MOTORS, WHEELS, RoverView
from rover_ros import RoverRosLink


# Defaults follow the current Autonmous_Ws simulation and SLAM contracts.
TOPICS = (
    ("joints", "Drive joint states", "/joint_states_gz", "sensor_msgs/msg/JointState"),
    ("ticks", "Drive encoder ticks", "/wheel/ticks", "std_msgs/msg/Int64MultiArray"),
    ("wheel_speeds", "Wheel speeds (m/s)", "/wheel/per_wheel_speeds", "std_msgs/msg/Float64MultiArray"),
    ("map", "Map", "/map", "nav_msgs/msg/OccupancyGrid"),
    ("pose", "Estimated pose / speed", "/odometry/filtered", "nav_msgs/msg/Odometry"),
    ("imu", "Orientation", "/imu/data", "sensor_msgs/msg/Imu"),
    ("rgb", "RGB camera", "/camera/image_raw", "sensor_msgs/msg/Image"),
    ("depth", "Depth camera", "/camera/depth/image_raw", "sensor_msgs/msg/Image"),
    ("path", "Planned route", "/plan", "nav_msgs/msg/Path"),
    ("wheel_odom", "Wheel odometry", "/odom", "nav_msgs/msg/Odometry"),
    ("slip", "Slip flag", "/wheel/slip_detected", "std_msgs/msg/Bool"),
)


def message(text):
    label = QLabel(text)
    label.setWordWrap(True)
    return label


class RoverDashboard(QMainWindow):
    closed = Signal()

    def __init__(self):
        super().__init__()
        self.settings = QSettings("MESEKET", "MarsRoverStation")
        self.readings = {}
        self.ros_link = None
        # Owner contract for the shared, optional STL viewer. No ROV controller is created.
        self.armed = self.ready = False
        self.last_imu = float("-inf")
        self.imu = dict.fromkeys(("roll", "pitch", "yaw"), 0.0)
        self.cmd = self.out_cmd = dict.fromkeys(("surge", "sway", "yaw", "heave", "roll"), 0.0)
        setup_fonts(QApplication.instance())
        self.setStyleSheet(QSS + """
            QTableWidget { background:#0b1119; gridline-color:#243245; }
            QHeaderView::section { background:#182230; padding:8px; border:1px solid #243245; }
        """)
        self.setWindowTitle("MESEKET · Mars Rover Control Station")
        self.resize(1600, 960)
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.topbar())
        body = QWidget()
        columns = QHBoxLayout(body)
        columns.setContentsMargins(14, 12, 14, 8)
        columns.setSpacing(14)
        columns.addWidget(self.manual_panel())
        columns.addWidget(self.center_panel(), 1)
        columns.addWidget(self.settings_panel())
        # Match the ROV's columns, while allowing smaller laptop displays to scroll.
        body.setMinimumWidth(1260)
        body_scroll = make_scroll(body)
        body_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        root.addWidget(body_scroll, 1)
        root.addWidget(self.console_panel())
        self.setCentralWidget(central)
        self.statusBar().showMessage("OFFLINE · 4 drive + 4 steering motors · 4 drive encoders · steering feedback deferred · No commands sent")
        self.freshness_timer = QTimer(self)
        self.freshness_timer.timeout.connect(self.refresh_readings)
        self.freshness_timer.start(250)

    @staticmethod
    def unavailable(button):
        button.setEnabled(False)
        button.setToolTip("Rover command output is not implemented in this version.")
        return button

    def topbar(self):
        bar = QFrame()
        bar.setObjectName("topbar")
        layout = QVBoxLayout(bar)
        layout.setContentsMargins(18, 10, 18, 10)
        row = QHBoxLayout()
        title = QLabel("MARS ROVER  <span style='color:#5fd0ff'>CONTROL STATION</span>")
        title.setObjectName("title")
        row.addWidget(title)
        row.addStretch()
        self.connection_pill = Pill("Disconnected", "idle")
        row.addWidget(self.connection_pill)
        self.safety_pill = Pill("STATE UNKNOWN", "warn")
        row.addWidget(self.safety_pill)
        back = QPushButton("Vehicle selection")
        back.clicked.connect(self.close)
        row.addWidget(back)
        layout.addLayout(row)
        actions = QHBoxLayout()
        actions.addWidget(message("Read-only telemetry · motion controls unavailable"))
        actions.addStretch()
        self.connect_button = QPushButton("CONNECT ROS 2")
        self.connect_button.setObjectName("connect")
        self.connect_button.clicked.connect(self.toggle_ros)
        actions.addWidget(self.connect_button)
        for text, name in (("ARM", "arm"), ("RESET LATCH", "tool"),
                           ("■  E-STOP · UNAVAILABLE", "estop")):
            button = QPushButton(text)
            button.setObjectName(name)
            actions.addWidget(self.unavailable(button))
        layout.addLayout(actions)
        modes = QHBoxLayout()
        modes.addWidget(QLabel("CONTROL MODE"))
        self.mode = QComboBox()
        self.mode.addItems(["Manual drive", "Autonomous navigation", "Motor inspection"])
        self.mode.setEnabled(False)
        modes.addWidget(self.mode)
        inspect = QPushButton("OPEN MOTOR PANEL  [D1–D4 / S1–S4]")
        inspect.clicked.connect(lambda: self.center_tabs.setCurrentIndex(1))
        modes.addWidget(inspect)
        modes.addWidget(message("4 drive motors · 4 steering motors · steering encoders not implemented"), 1)
        layout.addLayout(modes)
        return bar

    def manual_panel(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 4, 0)
        manual = QGroupBox("Manual control · hold to move")
        grid = QGridLayout(manual)
        controls = (("yaw_l", "Turn L", "Q", 0, 0), ("up", "Forward", "W", 0, 1),
                    ("yaw_r", "Turn R", "E", 0, 2), ("left", "Steer L", "A", 1, 0),
                    ("stop", "STOP", "all motors", 1, 1), ("right", "Steer R", "D", 1, 2),
                    ("down", "Reverse", "S", 2, 1))
        for kind, title, key, row, col in controls:
            grid.addWidget(self.unavailable(PadButton(kind, title, key)), row, col)
        keyboard = QCheckBox("Enable keyboard keys")
        keyboard.setEnabled(False)
        grid.addWidget(keyboard, 3, 0, 1, 3)
        layout.addWidget(manual)
        steering = QGroupBox("Four-wheel steering")
        form = QFormLayout(steering)
        self.steering_mode = QComboBox()
        self.steering_mode.addItems(["Coordinated steering", "Crab steering", "Turn in place"])
        self.steering_mode.setEnabled(False)
        form.addRow("Mode", self.steering_mode)
        form.addRow(message("FL / FR / RL / RR each have a drive motor and a steering motor."))
        layout.addWidget(steering)
        power = QGroupBox("Drive and steering limits")
        limits = QFormLayout(power)
        self.speed_limit = self.saved_spin("speed_limit", 0.05, 2.0, 0.5, " m/s")
        self.steer_limit = self.saved_spin("steer_limit", 1, 90, 30, "°")
        limits.addRow("Drive speed", self.speed_limit)
        limits.addRow("Steering angle", self.steer_limit)
        limits.addRow(message("Local preferences only · controller limits not confirmed"))
        layout.addWidget(power)
        layout.addStretch()
        scroll = make_scroll(page)
        scroll.setFixedWidth(320)
        return scroll

    def saved_spin(self, key, minimum, maximum, default, suffix):
        field = QDoubleSpinBox()
        field.setRange(minimum, maximum)
        field.setSuffix(suffix)
        field.setValue(float(self.settings.value(key, default)))
        field.valueChanged.connect(lambda value: self.settings.setValue(key, value))
        return field

    def center_panel(self):
        self.center_tabs = QTabWidget()
        self.center_tabs.addTab(make_scroll(self.drive_page()), "Rover Drive")
        self.center_tabs.addTab(make_scroll(self.motor_page()), "Motor Station")
        sensors = QTabWidget()
        sensors.addTab(make_scroll(self.sensor_page()), "IMU / rover health")
        sensors.addTab(self.encoder_page(), "Drive encoders")
        self.center_tabs.addTab(sensors, "Sensors")
        self.ps4_monitor = PS4Monitor(self.settings)
        self.ps4_monitor.status.setText("OFFLINE · controller monitoring not connected")
        self.center_tabs.addTab(make_scroll(self.ps4_monitor), "PS4 Monitor")
        self.stl_viewer = STLViewer(self)
        self.stl_viewer.mode.removeItem(3)  # ROV motion simulation does not model rover steering.
        self.stl_viewer.mode.setCurrentIndex(0)
        self.stl_viewer.info.setText("Optional rover STL · load your rover model. VTK is required for 3D rendering.")
        self.stl_viewer.status.setText("Manual model view · no live IMU connection")
        self.center_tabs.addTab(self.stl_viewer, "3D STL")
        return self.center_tabs

    def drive_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        vehicle = QGroupBox("Rover · drive and steering motors")
        view_layout = QVBoxLayout(vehicle)
        header = QHBoxLayout()
        self.telemetry_label = QLabel("Telemetry: disconnected")
        header.addWidget(self.telemetry_label, 1)
        self.camera_toggle = QPushButton("▶ Camera stream")
        self.camera_toggle.setObjectName("camtoggle")
        self.camera_toggle.setCheckable(True)
        self.camera_toggle.toggled.connect(self.toggle_camera)
        header.addWidget(self.camera_toggle)
        view_layout.addLayout(header)
        self.view_stack = QStackedWidget()
        self.rover_view = RoverView()
        self.camera_view = CameraView()
        self.camera_view.set_state("idle", "No rover camera connected")
        self.view_stack.addWidget(self.rover_view)
        self.view_stack.addWidget(self.camera_view)
        view_layout.addWidget(self.view_stack, 1)
        view_layout.addWidget(message("D1–D4: wheel drive · S1–S4: steering · FL / FR / RL / RR"))
        layout.addWidget(vehicle, 1)
        axes = QGroupBox("Command axes · offline")
        row = QHBoxLayout(axes)
        row.addWidget(StickWidget("LEFT STICK", "steering ↔  forward / reverse ↕"))
        row.addWidget(StickWidget("RIGHT STICK", "turning ↔"))
        bars = QVBoxLayout()
        for title in ("Forward / reverse", "Steering", "Turning"):
            bars.addWidget(AxisBar(title))
        row.addLayout(bars, 1)
        axes.setEnabled(False)
        layout.addWidget(axes)
        return page

    def toggle_camera(self, show):
        self.view_stack.setCurrentIndex(1 if show else 0)
        self.camera_toggle.setText("◀ Rover view" if show else "▶ Camera stream")

    def motor_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message("Eight independent motor channels · outputs unavailable while disconnected."))
        self.motor_cards = {}
        for prefix, title in (("D", "Movement motors · measured RPM"), ("S", "Steering motors · measured angle")):
            group = QGroupBox(title)
            grid = QGridLayout(group)
            for i, wheel in enumerate(WHEELS):
                motor = f"{prefix}_{wheel}"
                card = StatCard(f"{prefix}{i + 1} · {wheel}", "-- rpm" if prefix == "D" else "Not implemented")
                self.motor_cards[motor] = card
                grid.addWidget(card, i // 2 * 2, i % 2)
                button = QPushButton(f"Inspect {prefix}{i + 1} · {wheel}")
                grid.addWidget(self.unavailable(button), i // 2 * 2 + 1, i % 2)
            layout.addWidget(group)
        layout.addWidget(message("Encoder feedback is shown under Sensors → Drive encoders. Motor commands and measured feedback are separate."))
        layout.addStretch()
        return page

    def encoder_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message("E1–E4: drive encoders · Steering encoders are deferred until their ROS interface is implemented."))
        self.encoder_table = QTableWidget(4, 6)
        self.encoder_table.setHorizontalHeaderLabels(["Encoder", "Motor", "Wheel", "Raw ticks", "RPM", "Status"])
        for row, motor in enumerate(DRIVE_MOTORS):
            for col, text in enumerate((f"E{row + 1}", f"D{row + 1}", WHEELS[row], "--", "--", "No data")):
                self.encoder_table.setItem(row, col, QTableWidgetItem(text))
        self.encoder_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.encoder_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.encoder_table)
        layout.addWidget(message("Joint-state speed is converted from rad/s to RPM. Raw ticks are shown only when received; each field expires after 2 seconds."))
        return page

    def sensor_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        gauges = QWidget()
        instruments = QGridLayout(gauges)
        self.horizon = horizon = AttitudeWidget()
        self.compass = compass = CompassWidget()
        horizon.setEnabled(False)
        compass.setEnabled(False)
        instruments.addWidget(horizon, 0, 0)
        instruments.addWidget(compass, 0, 1)
        self.imu_overlay = no_data = QLabel("NO IMU DATA · instrument positions are placeholders")
        no_data.setAlignment(Qt.AlignmentFlag.AlignCenter)
        no_data.setStyleSheet("background:#101822; color:#fbbf24; padding:8px;")
        instruments.addWidget(no_data, 0, 0, 1, 2, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(gauges)
        cards = QGridLayout()
        self.imu_cards = {}
        for i, caption in enumerate(("ROLL", "PITCH", "HEADING", "ROVER SPEED", "BATTERY", "WHEEL SLIP")):
            card = StatCard(caption, "--")
            cards.addWidget(card, i // 3, i % 3)
            self.imu_cards[caption] = card
        layout.addLayout(cards)
        group = QGroupBox("Subsystem health")
        form = QFormLayout(group)
        for name in ("SLAM / localization", "Nav2 navigation", "Drive controller", "Steering controller", "Camera / perception"):
            form.addRow(name, QLabel("Unknown · disconnected"))
        layout.addWidget(group)
        layout.addWidget(message("IMU updates when ROS data arrives. Battery and navigation connections are pending."))
        self.attitude_plot = PlotWidget("Attitude", "deg", ("roll", "pitch", "yaw"),
                                        time.monotonic, min_span=20)
        layout.addWidget(self.attitude_plot, 1)
        return page

    def settings_panel(self):
        tabs = QTabWidget()
        tabs.setFixedWidth(350)
        drive = QWidget()
        form = QFormLayout(drive)
        for key, title, default in (("expo", "Joystick expo", 30), ("deadzone", "Deadzone", 8)):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 100 if key == "expo" else 40)
            slider.setValue(int(self.settings.value(key, default)))
            slider.valueChanged.connect(lambda value, name=key: self.settings.setValue(name, value))
            form.addRow(title, slider)
        form.addRow(message("Local preferences · no drive configuration has been sent to the rover."))
        form.addRow("Drive channels", QLabel("D1 FL · D2 FR · D3 RL · D4 RR"))
        form.addRow("Steering channels", QLabel("S1 FL · S2 FR · S3 RL · S4 RR"))
        tabs.addTab(make_scroll(drive), "Drive")
        joystick = QWidget()
        joy_layout = QVBoxLayout(joystick)
        joy_layout.addWidget(message("PS4 / joystick input · disconnected"))
        for text in ("Enable joystick", "Calibrate centre"):
            joy_layout.addWidget(self.unavailable(QPushButton(text)))
        joy_layout.addWidget(message("Left stick: forward / steering. Right stick: turning. Input mapping will be confirmed when the rover controller is connected."))
        joy_layout.addStretch()
        tabs.addTab(make_scroll(joystick), "Joystick")
        tabs.addTab(make_scroll(self.setup_page()), "ROS / Camera")
        return tabs

    def console_panel(self):
        group = QGroupBox("Rover log · read-only")
        group.setFixedHeight(142)
        layout = QVBoxLayout(group)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(600)
        self.console.setFont(mono_font(12))
        self.console.appendPlainText("OFFLINE · Rover dashboard ready. No commands sent.")
        self.console.appendPlainText("4 drive encoders available · steering encoder integration deferred.")
        layout.addWidget(self.console)
        clear = QPushButton("Clear log")
        clear.clicked.connect(self.console.clear)
        layout.addWidget(clear, 0, Qt.AlignmentFlag.AlignRight)
        wrap = QWidget()
        outer = QVBoxLayout(wrap)
        outer.setContentsMargins(14, 0, 14, 10)
        outer.addWidget(group)
        return wrap

    def toggle_ros(self):
        if self.ros_link is not None:
            self.ready = False
            self.connect_button.setEnabled(False)
            self.ros_link.requestInterruption()
            return
        topics = {key: field.text().strip() for key, field in self.topic_fields.items()}
        if any(not topic or any(c.isspace() for c in topic) for topic in topics.values()):
            self.console.appendPlainText("Enter valid topic names under ROS / Camera before connecting.")
            return
        self.readings.clear()
        self.last_imu = float("-inf")
        self.attitude_plot.clear()
        self.ros_link = RoverRosLink(topics, self.encoder_source.currentText(),
                                     self.wheel_radius.value(), self)
        self.ros_link.status.connect(self.ros_status)
        self.ros_link.encoders.connect(self.receive_encoder)
        self.ros_link.imu.connect(self.receive_imu)
        self.ros_link.finished.connect(self.ros_finished)
        self.connect_button.setText("DISCONNECT")
        self.console.appendPlainText("Starting read-only ROS 2 telemetry. No motor commands will be sent.")
        self.ros_link.start()

    def ros_status(self, listening, detail):
        self.ready = listening
        self.connection_pill.set_state("ROS listening" if listening else "Disconnected",
                                       "warn" if listening else "idle")
        self.telemetry_label.setText(detail)
        self.console.appendPlainText(detail)

    def ros_finished(self):
        self.ready = False
        self.last_imu = float("-inf")
        self.ros_link.deleteLater()
        self.ros_link = None
        self.connect_button.setText("CONNECT ROS 2")
        self.connect_button.setEnabled(True)
        self.connection_pill.set_state("Disconnected", "idle")
        self.telemetry_label.setText("Telemetry: disconnected")
        self.console.appendPlainText("ROS telemetry stopped.")
        self.refresh_readings()

    def receive_encoder(self, motor, ticks, rpm):
        if self.ready:
            self.update_encoder(motor, ticks, rpm)

    def receive_imu(self, values):
        if not self.ready:
            return
        self.imu = values
        self.last_imu = time.monotonic()
        self.horizon.set_attitude(values["roll"], values["pitch"])
        self.compass.set_yaw(values["yaw"])
        self.attitude_plot.add(self.last_imu, [values[key] for key in ("roll", "pitch", "yaw")])
        for caption, key in (("ROLL", "roll"), ("PITCH", "pitch"), ("HEADING", "yaw")):
            self.imu_cards[caption].set(f'{values[key]:+.1f}°')

    def update_encoder(self, motor, ticks, value):
        """GUI-thread entry point. Raw ticks and measured RPM arrive independently."""
        if motor not in DRIVE_MOTORS:
            raise ValueError(f"Drive encoder not supported: {motor}")
        if ticks is not None and (isinstance(ticks, bool) or not isinstance(ticks, int)):
            raise ValueError("Encoder ticks must be an integer or None")
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
            raise ValueError("Encoder RPM must be a finite number or None")
        reading = self.readings.setdefault(motor, {})
        now = time.monotonic()
        if ticks is not None:
            reading.update(ticks=ticks, ticks_received=now)
        if value is not None:
            reading.update(value=float(value), received=now)
        self.refresh_readings()

    def refresh_readings(self):
        now = time.monotonic()
        fresh = {}
        for row, motor in enumerate(DRIVE_MOTORS):
            reading = self.readings.get(motor, {})
            speed_live = self.ready and now - reading.get("received", float("-inf")) <= 2.0
            ticks_live = self.ready and now - reading.get("ticks_received", float("-inf")) <= 2.0
            ticks = str(reading["ticks"]) if ticks_live else "STALE" if "ticks" in reading else "--"
            rpm = f'{reading["value"]:+.1f} rpm' if speed_live else "STALE" if "value" in reading else "-- rpm"
            self.encoder_table.item(row, 3).setText(ticks)
            self.encoder_table.item(row, 4).setText(rpm)
            status = "Live" if speed_live and ticks_live else "RPM only" if speed_live else "Ticks only" if ticks_live else "Stale" if reading else "No data"
            self.encoder_table.item(row, 5).setText(status)
            self.motor_cards[motor].set(rpm)
            if speed_live:
                fresh[motor] = reading
        self.rover_view.set_readings(fresh)
        imu_live = self.ready and now - self.last_imu <= 2.0
        if self.ready:
            self.telemetry_label.setText(f"Drive RPM: {len(fresh)}/4 channels · steering feedback deferred")
            self.connection_pill.set_state("Telemetry live" if fresh or imu_live else "ROS listening",
                                           "ok" if fresh or imu_live else "warn")
        state = "ROS listening" if self.ready else "OFFLINE"
        self.statusBar().showMessage(f"{state} · 4 drive encoders · steering feedback deferred · No commands sent")
        self.imu_overlay.setVisible(not imu_live)
        self.horizon.setEnabled(imu_live)
        self.compass.setEnabled(imu_live)
        if not imu_live:
            for caption in ("ROLL", "PITCH", "HEADING"):
                self.imu_cards[caption].set("--")

    def setup_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message(
            "Read-only ROS 2 telemetry. Choose an encoder source, then use CONNECT ROS 2. "
            "Topic changes apply on the next connection."
        ))
        form = QFormLayout()
        self.profile = QComboBox()
        self.profile.addItems(["Simulation", "Hardware"])
        form.addRow("Environment", self.profile)
        self.encoder_source = QComboBox()
        self.encoder_source.addItems(["Joint states", "Ticks and wheel speeds"])
        form.addRow("Drive feedback", self.encoder_source)
        self.wheel_radius = QDoubleSpinBox()
        self.wheel_radius.setRange(0.001, 2.0)
        self.wheel_radius.setDecimals(4)
        self.wheel_radius.setSuffix(" m")
        form.addRow("Wheel radius", self.wheel_radius)
        layout.addLayout(form)
        self.topic_fields = {}
        for key, title, default, kind in TOPICS:
            layout.addWidget(QLabel(title))
            field = QLineEdit(default)
            field.setToolTip(kind)
            self.topic_fields[key] = field
            layout.addWidget(field)
        layout.addWidget(message("Four drive encoders are supported. Steering encoder feedback is not implemented. Wheel radius must match the encoder node when converting wheel speeds from m/s to RPM."))
        self.setup_status = message("")
        layout.addWidget(self.setup_status)
        save = QPushButton("Save Rover topic settings")
        save.clicked.connect(self.save_topics)
        layout.addWidget(save)
        layout.addWidget(message(
            "Simulation wheel odometry defaults to /odom; hardware defaults to "
            "/wheel/odom_raw. Camera remappings can vary by driver. "
            "Each environment has separate settings; ROV settings are untouched."
        ))
        self.profile.setCurrentText(self.settings.value("profile", "Simulation"))
        self.profile.currentTextChanged.connect(self.load_topics)
        self.load_topics()
        return page

    def load_topics(self):
        profile = self.profile.currentText()
        for key, _, default, _ in TOPICS:
            if key == "wheel_odom" and profile == "Hardware":
                default = "/wheel/odom_raw"
            if key == "joints" and profile == "Hardware":
                default = "/joint_states"
            self.topic_fields[key].setText(self.settings.value(f"{profile}/{key}", default))
        self.encoder_source.setCurrentText(self.settings.value(f"{profile}/encoder_source", "Joint states" if profile == "Simulation" else "Ticks and wheel speeds"))
        self.wheel_radius.setValue(float(self.settings.value(f"{profile}/wheel_radius", 0.06)))
        self.setup_status.setText("Unsaved edits are discarded when changing environment. ROS 2 must be installed and sourced to connect.")

    def save_topics(self):
        profile = self.profile.currentText()
        for key, field in self.topic_fields.items():
            topic = field.text().strip()
            if not topic or any(character.isspace() for character in topic):
                self.setup_status.setText("Enter non-empty topic names without whitespace before saving.")
                field.setFocus()
                return
        self.settings.setValue("profile", profile)
        self.settings.setValue(f"{profile}/encoder_source", self.encoder_source.currentText())
        self.settings.setValue(f"{profile}/wheel_radius", self.wheel_radius.value())
        for key, field in self.topic_fields.items():
            self.settings.setValue(f"{profile}/{key}", field.text().strip())
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.setup_status.setText("Could not save settings. Check local settings storage permissions.")
            return
        self.setup_status.setText(f"{profile} topic settings saved. Reconnect to apply changes.")

    def closeEvent(self, event):
        if self.ros_link is not None and self.ros_link.isRunning():
            self.ready = False
            self.ros_link.requestInterruption()
            if not self.ros_link.wait(1500):
                self.console.appendPlainText("ROS is still stopping. Try leaving Rover mode again shortly.")
                event.ignore()
                return
        if self.stl_viewer.loader and self.stl_viewer.loader.isRunning():
            self.stl_viewer.info.setText("Wait for the model to finish loading before leaving Rover mode.")
            event.ignore()
            return
        self.closed.emit()
        super().closeEvent(event)

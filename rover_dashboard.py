"""Rover dashboard with optional read-only ROS telemetry; no motion commands."""

import csv
import math
import time
from pathlib import Path

from PySide6.QtCore import QEvent, QEasingCurve, QPropertyAnimation, QSettings, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFormLayout, QFrame, QGridLayout, QGroupBox, QHeaderView, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPushButton, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget, QPlainTextEdit, QSlider, QStackedWidget,
    QFileDialog, QGraphicsOpacityEffect, QSpinBox,
)


from ROV_Control_Station_Integrated import (
    AttitudeWidget, AxisBar, CameraView, CompassWidget, Pill,
    PlotWidget, PS4Monitor, STLViewer, LogoMark, JoystickManager, DEF_AXES,
    StatCard, StickWidget, make_scroll, mono_font, setup_fonts,
)
from rover_view import DRIVE_MOTORS, WHEELS, RoverView
from rover_ros import RoverRosLink
from rover_camera import RoverCamera
from rover_theme import ACCENT, PLOT_COLORS, ROVER_QSS, pad_button


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
        self.keys = set()
        self.test_hold = None
        self.filtered = [0.0, 0.0, 0.0]
        self.preview_controls = []
        self.joy = JoystickManager()
        self.joy_centres = {}
        self.record_file = None
        self.csv_writer = None
        self.last_tick = time.monotonic()
        # Owner contract for the shared, optional STL viewer. No ROV controller is created.
        self.armed = self.ready = False
        self.last_imu = float("-inf")
        self.imu = dict.fromkeys(("roll", "pitch", "yaw"), 0.0)
        self.cmd = self.out_cmd = dict.fromkeys(("surge", "sway", "yaw", "heave", "roll"), 0.0)
        setup_fonts(QApplication.instance())
        self.setStyleSheet(ROVER_QSS)
        self.setWindowTitle("MESEKET · Mars Rover Control Station")
        self.resize(1600, 980)
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
        self.motor_window = QMainWindow(self)
        self.motor_window.setWindowTitle("Mars Rover · Motor Test Station · PREVIEW ONLY")
        self.motor_window.setCentralWidget(make_scroll(self.motor_test_page()))
        self.motor_window.resize(1060, 740)
        self.preview_timer = QTimer(self)
        self.preview_timer.timeout.connect(self.input_tick)
        self.preview_timer.start(20)
        self.refresh_joysticks()
        QApplication.instance().installEventFilter(self)
        self.center_tabs.currentChanged.connect(self.fade_tab)

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
        row.addWidget(LogoMark(ACCENT))
        title = QLabel(f"MARS ROVER  <span style='color:{ACCENT}'>CONTROL STATION</span>")
        title.setObjectName("title")
        row.addWidget(title)
        row.addSpacing(14)
        self.connection_pill = Pill("Disconnected", "idle")
        self.connect_button = QPushButton("CONNECT ROS 2")
        self.connect_button.setObjectName("connect")
        self.connect_button.clicked.connect(self.toggle_ros)
        row.addWidget(self.connect_button)
        row.addWidget(self.connection_pill)
        row.addStretch()
        back = QPushButton("Vehicle selection")
        back.clicked.connect(self.close)
        row.addWidget(back)
        layout.addLayout(row)
        actions = QHBoxLayout()
        actions.addWidget(message("ROS telemetry · 4 drive + 4 steering motors"))
        actions.addStretch()
        camera = QPushButton("CAMERA / WINDOW")
        camera.clicked.connect(lambda: self.camera.open_window())
        actions.addWidget(camera)
        self.preview_button = QPushButton("INPUT PREVIEW")
        self.preview_button.setObjectName("preview")
        self.preview_button.setCheckable(True)
        self.preview_button.toggled.connect(self.set_preview)
        actions.addWidget(self.preview_button)
        self.safety_pill = Pill("OUTPUT DISABLED", "warn")
        actions.addWidget(self.safety_pill)
        for text, name in (("ARM", "arm"), ("RESET LATCH", "tool"), ("■ E-STOP", "estop")):
            button = QPushButton(text)
            button.setObjectName(name)
            actions.addWidget(self.unavailable(button))
        layout.addLayout(actions)
        modes = QHBoxLayout()
        modes.addWidget(QLabel("CONTROL MODE"))
        self.mode = QComboBox()
        self.mode.addItems(["Rover drive", "Motor inspection"])
        self.mode.currentIndexChanged.connect(lambda index: self.center_tabs.setCurrentIndex(index))
        modes.addWidget(self.mode)
        inspect = QPushButton("OPEN TEST WINDOW  [1–8]")
        inspect.clicked.connect(self.open_motor_window)
        modes.addWidget(inspect)
        self.preview_hint = QLabel("Live readings and local previews are shown separately. No motor commands are sent.")
        self.preview_hint.setStyleSheet("color:#9a8992;")
        modes.addWidget(self.preview_hint, 1)
        layout.addLayout(modes)
        return bar

    def manual_panel(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 4, 0)
        manual = QGroupBox("Manual control · hold to preview")
        column = QVBoxLayout(manual)
        grid = QGridLayout()
        grid.setSpacing(7)
        controls = (("yaw_l", "Turn L", "Q", 0, 0), ("up", "Forward", "W", 0, 1),
                    ("yaw_r", "Turn R", "E", 0, 2), ("left", "Steer L", "A", 1, 0),
                    ("stop", "STOP", "Space", 1, 1), ("right", "Steer R", "D", 1, 2),
                    ("down", "Reverse", "S", 2, 1))
        self.manual_buttons = {}
        for kind, title, key, row, col in controls:
            button = pad_button(kind, title, key)
            if key == "Space":
                button.clicked.connect(self.stop_preview)
            else:
                button.pressed.connect(lambda key=key: self.keys.add(key))
                button.released.connect(lambda key=key: self.keys.discard(key))
            self.preview_controls.append(button)
            self.manual_buttons[key] = button
            button.setEnabled(False)
            grid.addWidget(button, row, col)
        column.addLayout(grid)
        self.keyboard_enabled = QCheckBox("Enable keyboard keys in preview")
        self.keyboard_enabled.setChecked(True)
        self.keyboard_enabled.toggled.connect(self.clear_inputs)
        column.addWidget(self.keyboard_enabled)
        layout.addWidget(manual)
        steering = QGroupBox("Four-wheel steering")
        form = QFormLayout(steering)
        self.steering_mode = QComboBox()
        self.steering_mode.addItems(["Coordinated steering", "Crab steering", "Turn in place"])
        form.addRow("Mode", self.steering_mode)
        centre = QPushButton("Centre steering preview")
        centre.clicked.connect(self.clear_inputs)
        form.addRow(centre)
        form.addRow(message("Steering angle feedback is deferred. Preview angles are simulated."))
        layout.addWidget(steering)
        aux = QGroupBox("Auxiliary outputs")
        row = QHBoxLayout(aux)
        for kind, title in (("light", "Lights"), ("valve", "Aux"), ("alert", "Beacon")):
            button = pad_button(kind, title, "unmapped")
            row.addWidget(self.unavailable(button))
        layout.addWidget(aux)
        power = QGroupBox("Power limit · input preview")
        limits = QVBoxLayout(power)
        self.power_slider, row = self.slider_row("power", 1, 100, 50, "%")
        limits.addLayout(row)
        limits.addWidget(message("Scales the preview only. Physical drive limits are unchanged."))
        layout.addWidget(power)
        layout.addStretch()
        scroll = make_scroll(page)
        scroll.setFixedWidth(350)
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
        detach = QPushButton("↗ Open camera window")
        detach.clicked.connect(lambda: self.camera.open_window())
        header.addWidget(detach)
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
        axes = QGroupBox("Command axes · preview only")
        row = QHBoxLayout(axes)
        self.stick_left = StickWidget("LEFT STICK", "steering ↔  forward / reverse ↕", ACCENT)
        self.stick_right = StickWidget("RIGHT STICK", "turning ↔", ACCENT)
        row.addWidget(self.stick_left)
        row.addWidget(self.stick_right)
        bars = QVBoxLayout()
        self.command_bars = []
        for title in ("Drive", "Steer", "Turn"):
            bar = AxisBar(title, ACCENT, "#efb16a")
            bars.addWidget(bar)
            self.command_bars.append(bar)
        row.addLayout(bars, 1)
        layout.addWidget(axes)
        return page

    def toggle_camera(self, show):
        self.view_stack.setCurrentIndex(1 if show else 0)
        self.camera_toggle.setText("◀ Rover view" if show else "▶ Camera stream")
        if show and self.camera.thread is None:
            self.camera.start()

    def motor_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        launch = QPushButton("OPEN MOTOR TEST STATION  [1–8]")
        launch.setMinimumHeight(52)
        launch.clicked.connect(self.open_motor_window)
        layout.addWidget(launch)
        layout.addWidget(message("Measured drive feedback below. The separate station previews all eight channels without sending commands."))
        self.motor_cards = {}
        for prefix, title in (("D", "Movement motors · measured RPM"), ("S", "Steering motors · measured angle")):
            group = QGroupBox(title)
            grid = QGridLayout(group)
            for i, wheel in enumerate(WHEELS):
                motor = f"{prefix}_{wheel}"
                card = StatCard(f"{prefix}{i + 1} · {wheel}", "-- rpm" if prefix == "D" else "Not implemented")
                self.motor_cards[motor] = card
                grid.addWidget(card, i // 2 * 2, i % 2)

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
        self.encoder_table.setAlternatingRowColors(True)
        layout.addWidget(self.encoder_table)
        export = QPushButton("Export encoder readings · CSV")
        export.clicked.connect(self.export_encoders)
        layout.addWidget(export)
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
                                        time.monotonic, min_span=20, colors=PLOT_COLORS)
        row = QHBoxLayout()
        pause = QPushButton("Pause plots")
        pause.setCheckable(True)
        pause.toggled.connect(lambda on: [plot.set_paused(on) for plot in self.imu_plots])
        clear = QPushButton("Clear plots")
        clear.clicked.connect(lambda: [plot.clear() for plot in self.imu_plots])
        self.record_button = QPushButton("● Record CSV")
        self.record_button.setObjectName("rec")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.record_imu)
        window = QComboBox()
        for seconds in (10, 30, 60, 120):
            window.addItem(f"{seconds} s window", seconds)
        window.setCurrentIndex(1)
        window.currentIndexChanged.connect(lambda: [plot.set_window(window.currentData()) for plot in self.imu_plots])
        for button in (pause, clear, self.record_button, window):
            row.addWidget(button)
        layout.addLayout(row)
        self.gyro_plot = PlotWidget("Gyroscope", "deg/s", ("gx", "gy", "gz"), time.monotonic, colors=PLOT_COLORS)
        self.accel_plot = PlotWidget("Accelerometer", "g", ("ax", "ay", "az"), time.monotonic, colors=PLOT_COLORS)
        self.imu_plots = (self.attitude_plot, self.gyro_plot, self.accel_plot)
        for plot in self.imu_plots:
            layout.addWidget(plot, 1)
        return page

    def settings_panel(self):
        tabs = QTabWidget()
        tabs.setFixedWidth(360)
        drive = QWidget()
        layout = QVBoxLayout(drive)
        self.drive_settings = {}
        for key, title, description, minimum, maximum, default, suffix in (
            ("expo", "Joystick expo", "Finer input near the centre", 0, 100, 30, "%"),
            ("deadzone", "Deadzone", "Ignore small stick movement", 0, 40, 8, "%"),
            ("ramp", "Smooth ramp time", "Ease preview command changes", 0, 2000, 350, " ms"),
        ):
            layout.addWidget(QLabel(f"<b>{title}</b>"))
            layout.addWidget(message(description))
            slider, row = self.slider_row(key, minimum, maximum, default, suffix)
            self.drive_settings[key] = slider
            layout.addLayout(row)
            layout.addSpacing(10)
        layout.addWidget(QLabel("<b>Drive and steering limits</b>"))
        form = QFormLayout()
        self.speed_limit = self.saved_spin("speed_limit", .05, 2, .5, " m/s")
        self.steer_limit = self.saved_spin("steer_limit", 1, 90, 30, "°")
        form.addRow("Speed preference", self.speed_limit)
        form.addRow("Steering preview", self.steer_limit)
        layout.addLayout(form)
        layout.addWidget(message("Local input settings. Hardware failsafe, arming and command output are not connected."))
        layout.addStretch()
        reset = QPushButton("⟲ Reset preview settings")
        reset.clicked.connect(self.reset_preview_settings)
        layout.addWidget(reset)
        tabs.addTab(make_scroll(drive), "Drive")
        tabs.addTab(make_scroll(self.joystick_page()), "Joystick")
        self.camera = RoverCamera(self.settings, self)
        self.camera.frame.connect(self.camera_view.set_frame)
        self.camera.state.connect(self.camera_view.set_state)
        tabs.addTab(make_scroll(self.camera), "Camera")
        tabs.addTab(make_scroll(self.setup_page()), "ROS")
        return tabs

    def console_panel(self):
        group = QGroupBox("Rover log · read-only")
        group.setFixedHeight(142)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 0, 6, 0)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(600)
        self.console.setFont(mono_font(12))
        self.console.appendPlainText("OFFLINE · Rover dashboard ready. No commands sent.")
        self.console.appendPlainText("4 drive encoders available · steering encoder integration deferred.")
        layout.addWidget(self.console)
        clear = QPushButton("Clear log")
        clear.setStyleSheet("padding:3px 12px;")
        clear.clicked.connect(self.console.clear)
        layout.addWidget(clear, 0, Qt.AlignmentFlag.AlignRight)
        wrap = QWidget()
        outer = QVBoxLayout(wrap)
        outer.setContentsMargins(14, 0, 14, 10)
        outer.addWidget(group)
        return wrap

    def slider_row(self, key, minimum, maximum, default, suffix):
        row = QHBoxLayout()
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(int(self.settings.value(key, default)))
        value = QLabel(f"{slider.value()}{suffix}")
        value.setMinimumWidth(58)
        slider.valueChanged.connect(lambda number: value.setText(f"{number}{suffix}"))
        slider.valueChanged.connect(lambda number: self.settings.setValue(key, number))
        row.addWidget(slider, 1)
        row.addWidget(value)
        return slider, row

    def joystick_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.joystick_enabled = QCheckBox("Enable joystick input preview")
        self.joystick_enabled.setChecked(True)
        layout.addWidget(self.joystick_enabled)
        row = QHBoxLayout()
        self.joy_combo = QComboBox()
        self.joy_combo.currentIndexChanged.connect(self.select_joystick)
        row.addWidget(self.joy_combo, 1)
        refresh = QPushButton("⟳")
        refresh.clicked.connect(self.refresh_joysticks)
        row.addWidget(refresh)
        layout.addLayout(row)
        calibrate = QPushButton("CALIBRATE CENTRE · sticks at rest")
        calibrate.clicked.connect(self.calibrate_joystick)
        layout.addWidget(calibrate)
        self.centre_label = message("Centre offsets: uncalibrated")
        layout.addWidget(self.centre_label)
        self.axis_map, self.axis_inv = {}, {}
        form = QFormLayout()
        for key, title, original in (("drive", "Forward / reverse", "surge"),
                                     ("steer", "Steering", "sway"), ("turn", "Turning", "yaw")):
            combo = QComboBox()
            combo.addItem("None", -1)
            for index in range(10):
                combo.addItem(f"Axis {index}", index)
            default, inverted = DEF_AXES[original]
            combo.setCurrentIndex(combo.findData(int(self.settings.value(f"joy_axis/{key}", default))))
            invert = QCheckBox("Invert")
            invert.setChecked(self.settings.value(f"joy_inv/{key}", inverted, type=bool))
            combo.currentIndexChanged.connect(lambda _, name=key, widget=combo: self.settings.setValue(f"joy_axis/{name}", widget.currentData()))
            invert.toggled.connect(lambda value, name=key: self.settings.setValue(f"joy_inv/{name}", value))
            row = QHBoxLayout()
            row.addWidget(combo, 1)
            row.addWidget(invert)
            form.addRow(title, row)
            self.axis_map[key], self.axis_inv[key] = combo, invert
        layout.addLayout(form)
        layout.addWidget(message("Live axes and buttons are shown in PS4 Monitor. Enable INPUT PREVIEW to animate local commands."))
        self.joy_raw = message("No controller connected")
        self.joy_raw.setFont(mono_font(11))
        layout.addWidget(self.joy_raw)
        if not self.joy.ok:
            layout.addWidget(message(self.joy.error))
        layout.addStretch()
        return page

    def refresh_joysticks(self):
        self.joy_combo.blockSignals(True)
        self.joy_combo.clear()
        self.joy_combo.addItem("No controller", -1)
        devices = self.joy.devices()
        for index, name in devices:
            self.joy_combo.addItem(name, index)
        if devices:
            self.joy_combo.setCurrentIndex(1)
        self.joy_combo.blockSignals(False)
        self.select_joystick()

    def select_joystick(self, *_):
        self.joy.select(self.joy_combo.currentData())
        self.joy_centres.clear()
        self.clear_inputs() if hasattr(self, "command_bars") else None

    def calibrate_joystick(self):
        axes, _ = self.joy.poll()
        if not axes:
            self.centre_label.setText("Connect a controller first.")
            return
        self.joy_centres = dict(enumerate(axes))
        self.centre_label.setText("Centre offsets captured. Keep sticks neutral when calibrating.")

    def set_preview(self, enabled):
        self.clear_inputs()
        for button in self.preview_controls:
            button.setEnabled(enabled)
        self.preview_hint.setText("PREVIEW ONLY · hold keys or use a controller · Space stops · Esc exits preview" if enabled
                                  else "Live readings and local previews are shown separately. No motor commands are sent.")
        self.console.appendPlainText("Input preview enabled · no motor commands sent." if enabled else "Input preview stopped.")

    def clear_inputs(self):
        self.keys.clear()
        self.test_hold = None
        self.filtered = [0.0, 0.0, 0.0]
        if hasattr(self, "test_pill"):
            self.test_pill.set_state("PREVIEW IDLE", "idle")

    def stop_preview(self):
        self.preview_button.setChecked(False)
        self.clear_inputs()

    def open_motor_window(self):
        self.clear_inputs()
        self.motor_window.show()
        self.motor_window.raise_()
        self.motor_window.activateWindow()

    def motor_test_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        header = QHBoxLayout()
        self.test_pill = Pill("PREVIEW IDLE", "idle")
        header.addWidget(self.test_pill)
        enable = QPushButton("Enable / disable input preview")
        enable.clicked.connect(lambda: self.preview_button.setChecked(not self.preview_button.isChecked()))
        header.addWidget(enable)
        stop = QPushButton("STOP PREVIEW · Space")
        stop.clicked.connect(self.stop_preview)
        header.addWidget(stop)
        layout.addLayout(header)
        layout.addWidget(message("Local animation only. Hold 1–4 for drive, 5–8 for steering; release to stop. Steering encoder feedback is not implemented."))
        self.test_sliders, self.test_inversions = {}, {}
        for offset, prefix, title, maximum, unit in ((0, "D", "Drive motor preview", 100, "%"), (4, "S", "Steering motor preview", 90, "°")):
            group = QGroupBox(title)
            grid = QGridLayout(group)
            for index, wheel in enumerate(WHEELS):
                channel = offset + index
                grid.addWidget(QLabel(f"{channel + 1} · {prefix}{index + 1} · {wheel}"), index, 0)
                slider, row = self.slider_row(f"motor_preview/{channel}", 1, maximum, 25 if prefix == "D" else 30, unit)
                self.test_sliders[channel] = slider
                grid.addLayout(row, index, 1)
                invert = QCheckBox("Invert")
                invert.setChecked(self.settings.value(f"motor_invert/{channel}", False, type=bool))
                invert.toggled.connect(lambda value, channel=channel: self.settings.setValue(f"motor_invert/{channel}", value))
                self.test_inversions[channel] = invert
                grid.addWidget(invert, index, 2)
                for col, sign, text in ((3, -1, "Hold −"), (4, 1, "Hold +")):
                    button = QPushButton(text)
                    button.setObjectName("motorTestKey")
                    button.pressed.connect(lambda channel=channel, sign=sign: self.hold_motor(channel, sign))
                    button.released.connect(lambda channel=channel: self.release_motor(channel))
                    grid.addWidget(button, index, col)
            layout.addWidget(group)
        layout.addStretch()
        return page

    def hold_motor(self, channel, sign):
        if not self.preview_button.isChecked():
            self.test_pill.set_state("ENABLE INPUT PREVIEW", "warn")
            return
        self.clear_inputs()
        self.test_hold = channel, sign
        self.test_pill.set_state(f"PREVIEW · CHANNEL {channel + 1}", "warn", True)

    def release_motor(self, channel):
        if self.test_hold and self.test_hold[0] == channel:
            self.clear_inputs()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn and isinstance(obj, (QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox)):
            self.clear_inputs()
        if event.type() == QEvent.Type.ApplicationDeactivate or (
            event.type() in (QEvent.Type.WindowDeactivate, QEvent.Type.Close)
            and obj in (self, self.motor_window)
        ):
            self.clear_inputs()
        if event.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return super().eventFilter(obj, event)
        if QApplication.activeWindow() not in (self, self.motor_window) or not self.preview_button.isChecked():
            return super().eventFilter(obj, event)
        if isinstance(QApplication.focusWidget(), (QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox)):
            return super().eventFilter(obj, event)
        down = event.type() == QEvent.Type.KeyPress
        if event.isAutoRepeat():
            return False
        key = event.key()
        if key in (Qt.Key.Key_Space, Qt.Key.Key_Escape):
            self.stop_preview()
            return True
        if event.modifiers() & (Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.AltModifier | Qt.KeyboardModifier.MetaModifier):
            return False
        if QApplication.activeWindow() is self.motor_window and Qt.Key.Key_1 <= key <= Qt.Key.Key_8:
            channel = key - Qt.Key.Key_1
            self.hold_motor(channel, 1) if down else self.release_motor(channel)
            return True
        if not self.keyboard_enabled.isChecked():
            return False
        for letter in "WASDQE":
            if key == getattr(Qt.Key, f"Key_{letter}"):
                self.keys.add(letter) if down else self.keys.discard(letter)
                return True
        return super().eventFilter(obj, event)

    def input_tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(.1, now - self.last_tick), now
        if not self.isVisible() and not self.motor_window.isVisible():
            return
        if self.joy.changed():
            self.refresh_joysticks()
        axes, buttons = self.joy.poll()
        self.ps4_monitor.feed(self.joy.name, axes, buttons, self.joy.hats)
        self.joy_raw.setText("  ".join(f"{index}: {value:+.2f}" for index, value in enumerate(axes)) or "No controller connected")
        enabled = self.preview_button.isChecked()
        active = QApplication.activeWindow() in (self, self.motor_window)
        target = [0.0, 0.0, 0.0]
        if enabled and active:
            target = [100.0 * (("W" in self.keys) - ("S" in self.keys)),
                      100.0 * (("D" in self.keys) - ("A" in self.keys)),
                      100.0 * (("E" in self.keys) - ("Q" in self.keys))]
            if self.joystick_enabled.isChecked():
                deadzone = self.drive_settings["deadzone"].value() / 100
                expo = self.drive_settings["expo"].value() / 100
                for index, name in enumerate(("drive", "steer", "turn")):
                    axis = self.axis_map[name].currentData()
                    if axis is None or not 0 <= axis < len(axes):
                        continue
                    value = max(-1, min(1, axes[axis] - self.joy_centres.get(axis, 0)))
                    if self.axis_inv[name].isChecked():
                        value *= -1
                    value = 0 if abs(value) <= deadzone else math.copysign((abs(value) - deadzone) / (1 - deadzone), value)
                    shaped = (1 - expo) * value + expo * value ** 3
                    target[index] = max(-100, min(100, target[index] + shaped * 100))
            target[0] *= self.power_slider.value() / 100
        ramp = self.drive_settings["ramp"].value() / 1000
        blend = 1 if ramp <= 0 else min(1, dt * 4 / ramp)
        self.filtered = [current + (wanted - current) * blend for current, wanted in zip(self.filtered, target)]
        drive, steer, turn = self.filtered
        wheel_drive = dict.fromkeys(WHEELS, drive)
        angle = steer * self.steer_limit.value() / 100
        if self.steering_mode.currentIndex() == 1:
            steering = dict.fromkeys(WHEELS, angle)
        elif self.steering_mode.currentIndex() == 2:
            steering = dict(zip(WHEELS, (-45, 45, 45, -45))) if abs(turn) > 1 else dict.fromkeys(WHEELS, 0)
            wheel_drive = {wheel: turn * (-1 if wheel.endswith("L") else 1) * self.power_slider.value() / 100 for wheel in WHEELS}
        else:
            steering = {wheel: angle * (1 if wheel.startswith("F") else -1) for wheel in WHEELS}
            for wheel in WHEELS:
                wheel_drive[wheel] = max(-100, min(100, drive + turn * (-.5 if wheel.endswith("L") else .5)))
        if enabled and active and self.test_hold:
            channel, sign = self.test_hold
            value = self.test_sliders[channel].value() * sign * (-1 if self.test_inversions[channel].isChecked() else 1)
            unit = "%" if channel < 4 else "°"
            self.test_pill.set_state(f"PREVIEW · CHANNEL {channel + 1} · {value:+.0f}{unit}", "warn", True)
            wheel_drive, steering = dict.fromkeys(WHEELS, 0), dict.fromkeys(WHEELS, 0)
            (wheel_drive if channel < 4 else steering)[WHEELS[channel % 4]] = value
        self.stick_left.set_xy(steer, drive)
        self.stick_right.set_xy(turn, 0)
        for bar, value in zip(self.command_bars, self.filtered):
            bar.set_value(value if enabled else 0)
        for key, button in self.manual_buttons.items():
            button.setDown(key in self.keys and enabled)
        self.rover_view.set_commands(enabled, wheel_drive, steering)

    def reset_preview_settings(self):
        self.preview_button.setChecked(False)
        self.power_slider.setValue(50)
        for key, value in (("expo", 30), ("deadzone", 8), ("ramp", 350)):
            self.drive_settings[key].setValue(value)
        self.speed_limit.setValue(.5)
        self.steer_limit.setValue(30)
        self.steering_mode.setCurrentIndex(0)

    def fade_tab(self, index):
        if hasattr(self, "tab_animation"):
            self.tab_animation.stop()
            self.tab_effect.setOpacity(1)
        widget = self.center_tabs.widget(index)
        self.tab_effect = QGraphicsOpacityEffect(widget)
        widget.setGraphicsEffect(self.tab_effect)
        self.tab_animation = QPropertyAnimation(self.tab_effect, b"opacity", self)
        self.tab_animation.setDuration(180)
        self.tab_animation.setStartValue(.35)
        self.tab_animation.setEndValue(1)
        self.tab_animation.setEasingCurve(QEasingCurve.Type.OutCubic)
        self.tab_animation.start()

    def record_imu(self, enabled):
        if not enabled:
            self.stop_recording()
            return
        path, _ = QFileDialog.getSaveFileName(self, "Record rover IMU", str(Path.home() / "rover_imu.csv"), "CSV (*.csv)")
        if not path:
            self.record_button.setChecked(False)
            return
        try:
            self.record_file = open(path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(self.record_file, fieldnames=("monotonic_s", "roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az"))
            self.csv_writer.writeheader()
            self.console.appendPlainText("Recording received IMU samples to CSV.")
        except OSError as error:
            self.console.appendPlainText(f"Cannot record IMU: {error}")
            self.record_button.setChecked(False)

    def stop_recording(self):
        if self.record_file is not None:
            self.record_file.close()
        self.record_file = self.csv_writer = None
        if hasattr(self, "record_button"):
            self.record_button.blockSignals(True)
            self.record_button.setChecked(False)
            self.record_button.blockSignals(False)

    def export_encoders(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export drive encoder readings", str(Path.home() / "rover_encoders.csv"), "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("encoder", "motor", "wheel", "raw_ticks", "rpm", "status"))
                for row in range(4):
                    writer.writerow(self.encoder_table.item(row, col).text() for col in range(6))
            self.console.appendPlainText("Encoder snapshot exported.")
        except OSError as error:
            self.console.appendPlainText(f"Cannot export encoders: {error}")

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
        [plot.clear() for plot in self.imu_plots]
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
        for plot, keys in ((self.gyro_plot, ("gx", "gy", "gz")), (self.accel_plot, ("ax", "ay", "az"))):
            if all(key in values for key in keys):
                plot.add(self.last_imu, [values[key] for key in keys])
        if self.csv_writer is not None:
            try:
                self.csv_writer.writerow({"monotonic_s": self.last_imu, **values})
            except OSError as error:
                self.console.appendPlainText(f"IMU recording stopped: {error}")
                self.stop_recording()
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
        if not self.camera.shutdown():
            self.console.appendPlainText("Waiting for camera capture to stop…")
            QTimer.singleShot(250, self.close)
            event.ignore()
            return
        self.preview_button.setChecked(False)
        self.motor_window.hide()
        self.stop_recording()
        self.closed.emit()
        super().closeEvent(event)

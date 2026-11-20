"""Mars Rover Control Station dashboard with ROS 2 telemetry, 2D SLAM/Nav2 map, camera, and gated teleop."""

import csv
import math
from pathlib import Path
import time

from PySide6.QtCore import QEasingCurve, QEvent, QPropertyAnimation, QSettings, Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QFrame, QGraphicsOpacityEffect, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QPushButton, QSlider, QSpinBox, QStackedWidget,
    QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from ROV_Control_Station_Integrated import (
    AttitudeWidget, AxisBar, CameraView, CompassWidget, DEF_AXES,
    JoystickManager, LogoMark, Pill, PlotWidget, PS4Monitor, STLViewer,
    StatCard, StickWidget, make_scroll, mono_font, repolish, setup_fonts,
)
from rover_camera import RoverCamera
from rover_kinematics import (
    DEFAULT_WHEEL_RADIUS_M, DRIVE_MOTORS, MODE_COORDINATED,
    OUTPUT_PREVIEW, OUTPUT_ROS_TELEOP, SAFETY_ARMED, SAFETY_DISARMED,
    SAFETY_ESTOP, SAFETY_PREVIEW, SafetyController, WHEELS,
    compute_cmd_vel, compute_wheel_kinematics, shape_axis,
)
from rover_map import RoverMapPanel
from rover_ros import RoverRosLink
from rover_theme import ACCENT, PLOT_COLORS, ROVER_QSS, pad_button
from rover_view import RoverView


# Key, Human label, Sim default, ROS message type, Hardware override (if different)
TOPIC_DEFS = (
    ("joints", "Drive joint states", "/joint_states_gz", "sensor_msgs/msg/JointState", "/joint_states"),
    ("ticks", "Drive encoder ticks", "/wheel/ticks", "std_msgs/msg/Int64MultiArray", "/wheel/ticks"),
    ("wheel_speeds", "Wheel speeds (m/s)", "/wheel/per_wheel_speeds", "std_msgs/msg/Float64MultiArray", "/wheel/per_wheel_speeds"),
    ("imu", "Orientation / IMU", "/imu/data", "sensor_msgs/msg/Imu", "/imu/data"),
    ("pose", "Estimated pose / EKF", "/odometry/filtered", "nav_msgs/msg/Odometry", "/odometry/filtered"),
    ("wheel_odom", "Wheel odometry", "/odom", "nav_msgs/msg/Odometry", "/wheel/odom_raw"),
    ("slip", "Wheel slip flag", "/wheel/slip_detected", "std_msgs/msg/Bool", "/wheel/slip_detected"),
    ("single_slip", "Single-wheel slip", "/wheel/single_wheel_slip", "std_msgs/msg/Bool", "/wheel/single_wheel_slip"),
    ("map", "SLAM occupancy map", "/map", "nav_msgs/msg/OccupancyGrid", "/map"),
    ("path", "Nav2 planned path", "/plan", "nav_msgs/msg/Path", "/plan"),
    ("aruco_pose", "ArUco pose", "/perception/aruco_pose", "geometry_msgs/msg/PoseStamped", "/perception/aruco_pose"),
    ("obstacles", "3D obstacles", "/perception/obstacles_only", "vision_msgs/msg/Detection3DArray", "/perception/obstacles_only"),
    ("rgb", "RGB camera", "/camera/image_raw", "sensor_msgs/msg/Image", "/camera/color/image_raw"),
    ("depth", "Depth camera", "/camera/depth/image_raw", "sensor_msgs/msg/Image", "/camera/depth/filtered"),
    ("aruco_debug", "ArUco debug image", "/marker_detection/debug_image", "sensor_msgs/msg/Image", "/marker_detection/debug_image"),
    ("pose_debug", "Pose debug image", "/marker_detection/pose_debug_image", "sensor_msgs/msg/Image", "/marker_detection/pose_debug_image"),
    ("enc_diag", "Encoder diagnostics", "/slam/encoder_diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "/slam/encoder_diagnostics"),
    ("slip_diag", "Slip diagnostics", "/slam/slip_diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "/slam/slip_diagnostics"),
    ("cmd_vel", "Teleop command", "/cmd_vel", "geometry_msgs/msg/Twist", "/cmd_vel"),
    ("goal_pose", "Nav2 2D goal", "/goal_pose", "geometry_msgs/msg/PoseStamped", "/goal_pose"),
)

GUIDED_CHANNELS = (
    "D1 · FL Drive",
    "D2 · FR Drive",
    "D3 · RL Drive",
    "D4 · RR Drive",
    "S1 · FL Steer (preview)",
    "S2 · FR Steer (preview)",
    "S3 · RL Steer (preview)",
    "S4 · RR Steer (preview)",
)

TELEMETRY_CSV_FIELDS = (
    "monotonic_s", "pose_x", "pose_y", "pose_yaw_deg", "speed_mps",
    "roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az",
    "fl_ticks", "fl_rpm", "fr_ticks", "fr_rpm", "rl_ticks", "rl_rpm", "rr_ticks", "rr_rpm",
    "slip_detected",
)


def message(text: str) -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    return label


def wrap_deg180(angle: float) -> float:
    return ((angle + 180.0) % 360.0) - 180.0


class RoverDashboard(QMainWindow):
    closed = Signal()

    def __init__(self):
        super().__init__()
        self.settings = QSettings("MESEKET", "MarsRoverStation")
        self.safety = SafetyController()
        self.gazebo_teleop = False

        self.readings: dict[str, dict] = {}
        self.ros_link: RoverRosLink | None = None
        self.topic_rates: dict[str, dict] = {}
        self.diag_entries: dict[str, list[dict]] = {}

        self.keys: set[str] = set()
        self.test_hold: tuple[int, int] | None = None
        self.filtered = [0.0, 0.0, 0.0]
        self.preview_controls: list[QPushButton] = []
        self.joy = JoystickManager()
        self.joy_centres: dict[int, float] = {}
        self.last_tick = time.monotonic()

        self.record_file = None
        self.csv_writer = None
        self._last_csv_write = 0.0

        # Owner contract for shared STLViewer and Instruments
        self.armed = False
        self.ready = False
        self.last_imu = float("-inf")
        self.last_odom_time = float("-inf")
        self.last_slip_time = float("-inf")
        self.last_map_time = float("-inf")
        self.last_perception_time = float("-inf")

        self.raw_yaw_deg = 0.0
        self.yaw_offset_deg = 0.0
        self.imu = dict.fromkeys(("roll", "pitch", "yaw"), 0.0)
        self.latest_odom: dict | None = None
        self.slip_detected = False
        self.single_slip_detected = False
        self.cmd = self.out_cmd = dict.fromkeys(("surge", "sway", "yaw", "heave", "roll"), 0.0)

        self._log_entries: list[tuple[str, str, str]] = []
        self._log_filter = "ALL"
        self._guided_idx = 0
        self._guided_results: dict[int, str] = {}
        self._cam_close_attempts = 0
        self._closing_app = False

        self.tab_effect: QGraphicsOpacityEffect | None = None
        self.tab_animation: QPropertyAnimation | None = None

        setup_fonts(QApplication.instance())
        self.setStyleSheet(ROVER_QSS)
        self.setWindowTitle("Control Station · Mars Rover")
        self.resize(1600, 960)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.topbar())

        body = QWidget()
        columns = QHBoxLayout(body)
        columns.setContentsMargins(12, 10, 12, 8)
        columns.setSpacing(12)
        columns.addWidget(self.manual_panel())
        columns.addWidget(self.center_panel(), 1)
        columns.addWidget(self.settings_panel())
        body.setMinimumWidth(1200)
        body_scroll = make_scroll(body)
        body_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        root.addWidget(body_scroll, 1)
        root.addWidget(self.console_panel())
        self.setCentralWidget(central)

        self.statusBar().showMessage(
            "OFFLINE · 4 drive + 4 steering motors · 4 drive encoders · steering feedback deferred"
        )

        self.freshness_timer = QTimer(self)
        self.freshness_timer.timeout.connect(self.refresh_readings)
        self.freshness_timer.start(250)

        self.motor_window = QMainWindow(None)
        self.motor_window.setStyleSheet(ROVER_QSS)
        self.motor_window.setWindowTitle("Mars Rover · Motor Test Station [1–8]")
        self.motor_window.setCentralWidget(make_scroll(self.motor_test_page()))
        self.motor_window.resize(1040, 740)
        self.motor_window.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, False)

        self.preview_timer = QTimer(self)
        self.preview_timer.timeout.connect(self.input_tick)
        self.preview_timer.start(20)

        self.refresh_joysticks()
        QApplication.instance().installEventFilter(self)
        self.center_tabs.currentChanged.connect(self._on_center_tab_changed)
        self._sync_safety_ui()
        self.log_event("INFO", "Rover dashboard ready. Default mode: Preview Only (no commands sent).")
        self.log_event("INFO", "4 drive encoders supported; 4 steering encoders deferred until ROS interface exists.")

    def _btn(self, text: str, obj_name: str | None = None, checkable: bool = False) -> QPushButton:
        b = QPushButton(text)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        if obj_name:
            b.setObjectName(obj_name)
        if checkable:
            b.setCheckable(True)
        return b

    @staticmethod
    def unavailable(button: QPushButton) -> QPushButton:
        button.setEnabled(False)
        button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        button.setToolTip("Auxiliary hardware outputs are unmapped in this rover configuration.")
        return button

    def topbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("topbar")
        layout = QVBoxLayout(bar)
        layout.setContentsMargins(16, 8, 16, 8)
        layout.setSpacing(6)

        row = QHBoxLayout()
        row.addWidget(LogoMark(ACCENT))
        title = QLabel(f"MARS ROVER  <span style='color:{ACCENT}'>CONTROL STATION</span>")
        title.setObjectName("title")
        row.addWidget(title)
        row.addSpacing(12)

        self.connect_button = self._btn("CONNECT ROS 2", "connect")
        self.connect_button.clicked.connect(self.toggle_ros)
        row.addWidget(self.connect_button)

        self.connection_pill = Pill("Disconnected", "idle")
        row.addWidget(self.connection_pill)
        row.addStretch()

        back = self._btn("Vehicle selection", "topbarBtn")
        back.clicked.connect(self.close)
        row.addWidget(back)
        layout.addLayout(row)

        actions = QHBoxLayout()
        actions.setSpacing(6)
        sub = QLabel("4WD+4WS · steering deferred")
        self.drivetrain_label = sub
        sub.setStyleSheet("color:#9aaabe;")
        actions.addWidget(sub)
        actions.addStretch()

        actions.addWidget(QLabel("OUTPUT:"))
        self.output_mode_combo = QComboBox()
        self.output_mode_combo.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.output_mode_combo.addItem("Preview Only (Safe)", OUTPUT_PREVIEW)
        self.output_mode_combo.addItem("ROS 2 Teleop (/cmd_vel)", OUTPUT_ROS_TELEOP)
        self.output_mode_combo.currentIndexChanged.connect(self._on_output_mode_changed)
        actions.addWidget(self.output_mode_combo)

        camera_btn = self._btn("CAMERA WINDOW", "topbarBtn")
        camera_btn.clicked.connect(lambda: self.camera.open_window())
        actions.addWidget(camera_btn)

        self.preview_button = self._btn("INPUT PREVIEW", "preview", checkable=True)
        self.preview_button.toggled.connect(self.set_preview)
        actions.addWidget(self.preview_button)

        self.safety_pill = Pill("PREVIEW ONLY", "idle")
        actions.addWidget(self.safety_pill)

        self.arm_button = self._btn("ARM", "arm", checkable=True)
        self.arm_button.toggled.connect(self.toggle_arm)
        actions.addWidget(self.arm_button)

        self.reset_latch_button = self._btn("RESET LATCH", "tool")
        self.reset_latch_button.clicked.connect(self.reset_estop_latch)
        actions.addWidget(self.reset_latch_button)

        self.estop_button = self._btn("■ E-STOP", "estop")
        self.estop_button.clicked.connect(self.emergency_stop)
        actions.addWidget(self.estop_button)
        layout.addLayout(actions)

        modes = QHBoxLayout()
        modes.addWidget(QLabel("VIEW"))
        self.mode = QComboBox()
        self.mode.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.mode.addItems([
            "Rover Drive",
            "Navigation / Map",
            "Motor Station",
            "Sensors",
            "PS4 Monitor",
            "3D STL",
        ])
        self.mode.currentIndexChanged.connect(self._on_mode_combo_changed)
        modes.addWidget(self.mode)

        inspect = self._btn("OPEN TEST WINDOW  [1–8]")
        inspect.clicked.connect(self.open_motor_window)
        modes.addWidget(inspect)

        self.preview_hint = QLabel("")
        self.preview_hint.setStyleSheet("color:#9a8992;")
        modes.addWidget(self.preview_hint, 1)
        layout.addLayout(modes)
        return bar

    def manual_panel(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 4, 0)
        layout.setSpacing(8)

        manual = QGroupBox("Manual control · hold")
        column = QVBoxLayout(manual)
        grid = QGridLayout()
        grid.setSpacing(6)
        controls = (
            ("yaw_l", "Turn L", "Q", 0, 0),
            ("up", "Forward", "W", 0, 1),
            ("yaw_r", "Turn R", "E", 0, 2),
            ("left", "Steer L", "A", 1, 0),
            ("stop", "STOP", "Space", 1, 1),
            ("right", "Steer R", "D", 1, 2),
            ("down", "Reverse", "S", 2, 1),
        )
        self.manual_buttons = {}
        for kind, title, key, row, col in controls:
            button = pad_button(kind, title, key)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            if key == "Space":
                button.clicked.connect(self.stop_action)
            else:
                button.pressed.connect(lambda k=key: self.keys.add(k))
                button.released.connect(lambda k=key: self.keys.discard(k))
                self.preview_controls.append(button)
                button.setEnabled(False)
            self.manual_buttons[key] = button
            grid.addWidget(button, row, col)
        column.addLayout(grid)

        self.keyboard_enabled = QCheckBox("Enable keyboard (WASDQE)")
        self.keyboard_enabled.setChecked(True)
        self.keyboard_enabled.toggled.connect(self.clear_inputs)
        column.addWidget(self.keyboard_enabled)
        layout.addWidget(manual)

        steering = QGroupBox("Four-wheel steering")
        self.steering_group = steering
        form = QFormLayout(steering)
        self.steering_mode = QComboBox()
        self.steering_mode.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.steering_mode.addItems(["Coordinated steering", "Crab steering", "Turn in place"])
        form.addRow("Mode", self.steering_mode)
        centre = self._btn("Centre steering / zero")
        centre.clicked.connect(self.clear_inputs)
        form.addRow(centre)
        self.steering_note = message("Steering feedback is deferred; wheel angles are simulated.")
        form.addRow(self.steering_note)
        layout.addWidget(steering)

        aux = QGroupBox("Auxiliary outputs")
        row = QHBoxLayout(aux)
        for kind, title in (("light", "Lights"), ("valve", "Aux"), ("alert", "Beacon")):
            button = pad_button(kind, title, "unmapped")
            row.addWidget(self.unavailable(button))
        layout.addWidget(aux)

        power = QGroupBox("Power limit")
        limits = QVBoxLayout(power)
        self.power_slider, row = self.slider_row("power", 1, 100, 50, "%")
        limits.addLayout(row)
        limits.addWidget(message("Scales local preview and gated /cmd_vel output."))
        layout.addWidget(power)
        layout.addStretch()

        scroll = make_scroll(page)
        scroll.setFixedWidth(344)
        return scroll

    def saved_spin(self, key: str, minimum: float, maximum: float, default: float, suffix: str) -> QDoubleSpinBox:
        field = QDoubleSpinBox()
        field.setRange(minimum, maximum)
        field.setSuffix(suffix)
        field.setValue(float(self.settings.value(key, default)))
        field.valueChanged.connect(lambda value: self.settings.setValue(key, value))
        return field

    def center_panel(self) -> QTabWidget:
        self.center_tabs = QTabWidget()
        self.center_tabs.addTab(make_scroll(self.drive_page()), "Rover Drive")
        self.map_panel = RoverMapPanel(self)
        self.map_panel.goal_requested.connect(self.send_nav_goal)
        self.map_panel.log_message.connect(self.log_event)
        self.center_tabs.addTab(self.map_panel, "Navigation / Map")
        self.center_tabs.addTab(make_scroll(self.motor_page()), "Motor Station")

        self.sensor_tabs = QTabWidget()
        self.sensor_tabs.addTab(make_scroll(self.sensor_page()), "IMU · Health")
        self.sensor_tabs.addTab(self.encoder_page(), "Drive Encoders")
        self.sensor_tabs.addTab(make_scroll(self.diagnostics_page()), "ROS Diagnostics")
        self.center_tabs.addTab(self.sensor_tabs, "Sensors")

        self.ps4_monitor = PS4Monitor(self.settings)
        self.ps4_monitor.status.setText("No controller connected")
        self.center_tabs.addTab(make_scroll(self.ps4_monitor), "PS4 Monitor")

        self.stl_viewer = STLViewer(self)
        self.stl_viewer.mode.removeItem(3)
        self.stl_viewer.mode.setCurrentIndex(0)
        self.stl_viewer.info.setText("Optional rover STL · load a model. VTK required for 3D rendering.")
        self.stl_viewer.status.setText("Manual model view · connects to live IMU when available")
        self.center_tabs.addTab(self.stl_viewer, "3D STL")
        return self.center_tabs

    def drive_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(8)

        vehicle = QGroupBox("Rover · drive and steering motors")
        view_layout = QVBoxLayout(vehicle)
        header = QHBoxLayout()
        self.telemetry_label = QLabel("Telemetry: disconnected")
        header.addWidget(self.telemetry_label, 1)

        self.slip_banner = QLabel("SLIP ALERT")
        self.slip_banner.setObjectName("slipAlert")
        self.slip_banner.setVisible(False)
        header.addWidget(self.slip_banner)

        detach = self._btn("↗ Camera window")
        detach.clicked.connect(lambda: self.camera.open_window())
        header.addWidget(detach)

        self.camera_toggle = self._btn("▶ Camera view", "camtoggle", checkable=True)
        self.camera_toggle.toggled.connect(self.toggle_camera_view)
        header.addWidget(self.camera_toggle)
        view_layout.addLayout(header)

        self.view_stack = QStackedWidget()
        self.rover_view = RoverView()
        self.camera_view = CameraView()
        self.camera_view.set_state("idle", "Select ROS or RTSP source under Camera tab and press Connect")
        self.view_stack.addWidget(self.rover_view)
        self.view_stack.addWidget(self.camera_view)
        view_layout.addWidget(self.view_stack, 1)
        view_layout.addWidget(message("D1–D4: wheel drive · S1–S4: steering · FL / FR / RL / RR"))
        layout.addWidget(vehicle, 1)

        axes = QGroupBox("Command axes")
        row = QHBoxLayout(axes)
        self.stick_left = StickWidget("LEFT STICK", "steer ↔  drive ↕", ACCENT)
        self.stick_right = StickWidget("RIGHT STICK", "turn ↔", ACCENT)
        self.stick_left.setMaximumHeight(160)
        self.stick_right.setMaximumHeight(160)
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

    def toggle_camera_view(self, show: bool):
        # Switching the view never auto-starts RTSP or ROS streams.
        self.view_stack.setCurrentIndex(1 if show else 0)
        self.camera_toggle.setText("◀ Rover view" if show else "▶ Camera view")

    def motor_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        launch = self._btn("OPEN MOTOR TEST STATION  [1–8]")
        launch.setMinimumHeight(48)
        launch.clicked.connect(self.open_motor_window)
        layout.addWidget(launch)
        layout.addWidget(
            message("Live measured drive feedback below. Use the Motor Test Station to preview all 8 channels.")
        )

        self.motor_cards = {}
        for prefix, title in (("D", "Movement motors · measured RPM"), ("S", "Steering motors · measured angle")):
            group = QGroupBox(title)
            grid = QGridLayout(group)
            for i, wheel in enumerate(WHEELS):
                motor = f"{prefix}_{wheel}"
                card = StatCard(f"{prefix}{i + 1} · {wheel}", "-- rpm" if prefix == "D" else "Not implemented")
                self.motor_cards[motor] = card
                grid.addWidget(card, i // 2, i % 2)
            layout.addWidget(group)

        # Guided 8-motor inspection checklist (parity with ROV)
        wizard = QGroupBox("Guided 8-motor bench inspection")
        wlay = QVBoxLayout(wizard)
        self.guided_label = QLabel(f"Step 1/8 · Verify {GUIDED_CHANNELS[0]}")
        self.guided_label.setStyleSheet(f"color:{ACCENT}; font-weight:700;")
        wlay.addWidget(self.guided_label)

        btn_row = QHBoxLayout()
        for label, status in (("PASS", "PASS"), ("FAIL", "FAIL"), ("SKIP", "SKIP")):
            b = self._btn(label)
            b.clicked.connect(lambda _=False, s=status: self.mark_guided_step(s))
            btn_row.addWidget(b)
        reset_chk = self._btn("Reset checklist")
        reset_chk.clicked.connect(self.reset_guided_checklist)
        export_chk = self._btn("Export inspection CSV")
        export_chk.clicked.connect(self.export_inspection_dialog)
        btn_row.addWidget(reset_chk)
        btn_row.addWidget(export_chk)
        wlay.addLayout(btn_row)

        self.guided_summary = QLabel("0 / 8 checked")
        self.guided_summary.setStyleSheet("color:#9aaabe;")
        wlay.addWidget(self.guided_summary)
        layout.addWidget(wizard)
        layout.addStretch()
        return page

    def mark_guided_step(self, result: str):
        self._guided_results[self._guided_idx] = result
        self.log_event("INFO", f"Motor inspection [{GUIDED_CHANNELS[self._guided_idx]}]: {result}")
        self._guided_idx = (self._guided_idx + 1) % len(GUIDED_CHANNELS)
        self._refresh_guided_ui()

    def reset_guided_checklist(self):
        self._guided_idx = 0
        self._guided_results.clear()
        self._refresh_guided_ui()

    def _refresh_guided_ui(self):
        self.guided_label.setText(f"Step {self._guided_idx + 1}/8 · Verify {GUIDED_CHANNELS[self._guided_idx]}")
        parts = [f"{GUIDED_CHANNELS[i].split(' ')[0]}:{self._guided_results.get(i, '--')}" for i in range(len(GUIDED_CHANNELS))]
        self.guided_summary.setText(f"{len(self._guided_results)}/8 checked · " + "  ".join(parts))

    def export_inspection_to(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("step", "channel", "result"))
            for i, ch in enumerate(GUIDED_CHANNELS):
                writer.writerow((i + 1, ch, self._guided_results.get(i, "UNCHECKED")))

    def export_inspection_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export motor inspection", str(Path.home() / "rover_motor_inspection.csv"), "CSV (*.csv)")
        if not path:
            return
        try:
            self.export_inspection_to(path)
            self.log_event("INFO", "Motor inspection checklist exported.")
        except OSError as error:
            self.log_event("WARN", f"Cannot export inspection: {error}")

    def encoder_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message("E1–E4: drive encoders · E5–E8: steering encoders deferred until ROS publisher exists."))
        self.encoder_table = QTableWidget(8, 7)
        self.encoder_table.setHorizontalHeaderLabels(
            ["Encoder", "Motor", "Wheel", "Raw ticks", "Speed (m/s)", "RPM / Angle", "Status"]
        )
        for row in range(4):
            for col, text in enumerate((f"E{row + 1}", f"D{row + 1}", WHEELS[row], "--", "--", "--", "No data")):
                self.encoder_table.setItem(row, col, QTableWidgetItem(text))
        for row in range(4, 8):
            idx = row - 4
            for col, text in enumerate((f"E{row + 1}", f"S{idx + 1}", WHEELS[idx], "--", "--", "Deferred", "Not implemented")):
                item = QTableWidgetItem(text)
                item.setForeground(QColor("#6b7280"))
                self.encoder_table.setItem(row, col, item)
        self.encoder_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.encoder_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.encoder_table.setAlternatingRowColors(True)
        layout.addWidget(self.encoder_table)

        export = self._btn("Export encoder readings · CSV")
        export.clicked.connect(self.export_encoders)
        layout.addWidget(export)
        layout.addWidget(message("Joint-state velocities convert rad/s to RPM. Readings expire after 2 seconds."))
        return page

    def sensor_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(8)

        # Top HUD: Attitude + Compass + 3-column numeric IMU HUD
        top = QHBoxLayout()
        self.horizon = AttitudeWidget()
        self.compass = CompassWidget()
        self.horizon.setEnabled(False)
        self.compass.setEnabled(False)
        top.addWidget(self.horizon, 1)
        top.addWidget(self.compass, 1)

        hud_grid = QGridLayout()
        hud_grid.setHorizontalSpacing(12)
        hud_grid.setVerticalSpacing(4)
        self.imu_lbl: dict[str, QLabel] = {}
        mono = mono_font(12)
        groups = (
            ("ATTITUDE °", ("roll", "pitch", "yaw")),
            ("GYRO °/s", ("gx", "gy", "gz")),
            ("ACCEL g", ("ax", "ay", "az")),
        )
        for c, (title, keys) in enumerate(groups):
            hd = QLabel(title)
            hd.setStyleSheet(f"color:{ACCENT}; font-weight:700;")
            hud_grid.addWidget(hd, 0, c)
            for r, k in enumerate(keys, start=1):
                lab = QLabel(f"{k.upper():<5}{0:+7.2f}")
                lab.setFont(mono)
                lab.setMinimumWidth(105)
                hud_grid.addWidget(lab, r, c)
                self.imu_lbl[k] = lab

        self.imu_rate_lbl = QLabel("rate -- Hz")
        self.imu_rate_lbl.setFont(mono)
        self.imu_rate_lbl.setStyleSheet("color:#8aa0b6;")
        hud_grid.addWidget(self.imu_rate_lbl, 4, 0)

        self.imu_pill = Pill("IMU: no data", "idle")
        hud_grid.addWidget(self.imu_pill, 4, 1)

        self.zero_yaw_btn = self._btn("Zero yaw")
        self.zero_yaw_btn.setToolTip("Zero current compass heading display and 3D STL orientation")
        self.zero_yaw_btn.clicked.connect(self.zero_yaw)
        hud_grid.addWidget(self.zero_yaw_btn, 4, 2)

        info_col = QVBoxLayout()
        info_col.addLayout(hud_grid)
        info_col.addStretch()
        top.addLayout(info_col, 2)
        layout.addLayout(top)

        # 6 live rover StatCards
        cards = QGridLayout()
        self.imu_cards = {}
        for i, caption in enumerate(("ROVER SPEED", "POSE (X, Y)", "WHEEL SLIP", "HEADING", "CHASSIS TILT", "BATTERY")):
            card = StatCard(caption, "Pending" if caption == "BATTERY" else "--")
            cards.addWidget(card, i // 3, i % 3)
            self.imu_cards[caption] = card
        layout.addLayout(cards)

        # Live Subsystem Health Matrix
        health_group = QGroupBox("Subsystem health")
        hform = QFormLayout(health_group)
        self.health_labels: dict[str, QLabel] = {}
        for key, label in (
            ("encoders", "Encoders / Wheel Odom"),
            ("imu", "IMU (/imu/data)"),
            ("slam", "SLAM / EKF (/odometry/filtered)"),
            ("nav2", "Nav2 (/map · /plan)"),
            ("perception", "Perception (/aruco · obstacles)"),
            ("slip", "Wheel Slip Monitor"),
            ("steering", "Steering Encoders"),
        ):
            val = QLabel("Deferred · pending ROS interface" if key == "steering" else "Offline")
            val.setStyleSheet("color:#8aa0b6;")
            self.health_labels[key] = val
            hform.addRow(label, val)
        layout.addWidget(health_group)

        # Plot controls
        ctrl_row = QHBoxLayout()
        pause = self._btn("Pause plots", checkable=True)
        pause.toggled.connect(lambda on: [plot.set_paused(on) for plot in self.imu_plots])
        clear = self._btn("Clear plots")
        clear.clicked.connect(lambda: [plot.clear() for plot in self.imu_plots])
        self.record_button = self._btn("● Record Telemetry CSV", "rec", checkable=True)
        self.record_button.toggled.connect(self.record_telemetry_toggled)
        window = QComboBox()
        window.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        for seconds in (10, 30, 60, 120):
            window.addItem(f"{seconds} s window", seconds)
        window.setCurrentIndex(1)
        window.currentIndexChanged.connect(lambda: [plot.set_window(window.currentData()) for plot in self.imu_plots])
        for w in (pause, clear, self.record_button, window):
            ctrl_row.addWidget(w)
        layout.addLayout(ctrl_row)

        self.attitude_plot = PlotWidget("Attitude", "deg", ("roll", "pitch", "yaw"), time.monotonic, min_span=20, colors=PLOT_COLORS)
        self.gyro_plot = PlotWidget("Gyroscope", "deg/s", ("gx", "gy", "gz"), time.monotonic, min_span=20, colors=PLOT_COLORS)
        self.accel_plot = PlotWidget("Accelerometer", "g", ("ax", "ay", "az"), time.monotonic, min_span=0.5, colors=PLOT_COLORS)
        self.imu_plots = (self.attitude_plot, self.gyro_plot, self.accel_plot)
        for plot in self.imu_plots:
            plot.setMinimumHeight(155)
            layout.addWidget(plot, 1)
        return page

    def diagnostics_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message("Live ROS 2 DiagnosticArray messages and subscribed topic rates."))

        diag_group = QGroupBox("Node diagnostics (/slam/*_diagnostics)")
        dlay = QVBoxLayout(diag_group)
        self.diag_table = QTableWidget(0, 4)
        self.diag_table.setHorizontalHeaderLabels(["Source", "Component", "Level", "Message"])
        self.diag_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.diag_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.diag_table.setMinimumHeight(160)
        dlay.addWidget(self.diag_table)
        layout.addWidget(diag_group)

        rates_group = QGroupBox("Subscribed topic rates · ROS graph")
        rlay = QVBoxLayout(rates_group)
        self.rates_table = QTableWidget(len(TOPIC_DEFS), 4)
        self.rates_table.setHorizontalHeaderLabels(["Channel", "Topic", "Rate (Hz)", "State"])
        for row, (key, title, _, _, _) in enumerate(TOPIC_DEFS):
            for col, text in enumerate((title, "--", "--", "Idle")):
                self.rates_table.setItem(row, col, QTableWidgetItem(text))
        self.rates_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.rates_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.rates_table.setMinimumHeight(260)
        rlay.addWidget(self.rates_table)

        scan_btn = self._btn("Refresh active ROS 2 topics")
        scan_btn.clicked.connect(self.scan_ros_topics)
        rlay.addWidget(scan_btn)

        self.discovered_box = QPlainTextEdit()
        self.discovered_box.setReadOnly(True)
        self.discovered_box.setMaximumHeight(130)
        self.discovered_box.setFont(mono_font(11))
        self.discovered_box.setPlaceholderText("Connect ROS 2 and click Refresh to inspect active ROS graph topics.")
        rlay.addWidget(self.discovered_box)
        layout.addWidget(rates_group)
        return page

    def scan_ros_topics(self):
        if self.ros_link is None or not self.ready:
            self.discovered_box.setPlainText("Connect ROS 2 first to inspect active topics.")
            return
        self.ros_link.request_topic_scan()

    def receive_discovered_topics(self, topics: list):
        lines = [f"{name}  [{', '.join(types)}]" for name, types in topics]
        self.discovered_box.setPlainText("\n".join(lines) or "No active topics discovered.")

    def settings_panel(self) -> QTabWidget:
        self.settings_tabs = QTabWidget()
        self.settings_tabs.setFixedWidth(342)

        drive = QWidget()
        layout = QVBoxLayout(drive)
        self.drive_settings = {}
        for key, title, description, minimum, maximum, default, suffix in (
            ("expo", "Joystick expo", "Finer input near centre", 0, 100, 30, "%"),
            ("deadzone", "Deadzone", "Ignore small stick movement", 0, 40, 8, "%"),
            ("ramp", "Smooth ramp time", "Ease command transitions", 0, 2000, 350, " ms"),
        ):
            layout.addWidget(QLabel(f"<b>{title}</b>"))
            layout.addWidget(message(description))
            slider, row = self.slider_row(key, minimum, maximum, default, suffix)
            self.drive_settings[key] = slider
            layout.addLayout(row)
            layout.addSpacing(8)

        layout.addWidget(QLabel("<b>Drive · steering limits</b>"))
        form = QFormLayout()
        self.speed_limit = self.saved_spin("speed_limit", 0.05, 2.0, 0.5, " m/s")
        self.yaw_limit = self.saved_spin("yaw_limit", 0.1, 3.0, 1.0, " rad/s")
        self.steer_limit = self.saved_spin("steer_limit", 1.0, 90.0, 30.0, "°")
        form.addRow("Max speed", self.speed_limit)
        form.addRow("Max yaw rate", self.yaw_limit)
        form.addRow("Steering preview", self.steer_limit)
        layout.addLayout(form)
        layout.addWidget(message("Applies to preview and gated /cmd_vel scaling when armed."))
        layout.addStretch()

        reset = self._btn("⟲ Reset drive settings")
        reset.clicked.connect(self.reset_preview_settings)
        layout.addWidget(reset)

        self.settings_tabs.addTab(make_scroll(drive), "Drive")
        self.settings_tabs.addTab(make_scroll(self.joystick_page()), "Joystick")

        self.camera = RoverCamera(self.settings, self)
        self.camera.frame.connect(self.camera_view.set_frame)
        self.camera.state.connect(self.camera_view.set_state)
        self.camera.ros_feed_changed.connect(self._on_ros_camera_feed_changed)
        self.settings_tabs.addTab(make_scroll(self.camera), "Camera")
        self.settings_tabs.addTab(make_scroll(self.setup_page()), "ROS")
        return self.settings_tabs

    def console_panel(self) -> QWidget:
        group = QGroupBox("Rover log")
        group.setFixedHeight(146)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 0, 6, 4)

        top = QHBoxLayout()
        self.filter_buttons: dict[str, QPushButton] = {}
        for key, label in (("ALL", "All"), ("ROS", "ROS"), ("WARN", "Warnings"), ("CTRL", "Controls")):
            b = self._btn(label, "filterBtn", checkable=True)
            b.clicked.connect(lambda _=False, k=key: self.set_log_filter(k))
            self.filter_buttons[key] = b
            top.addWidget(b)
        self.filter_buttons["ALL"].setChecked(True)
        top.addStretch()

        export_btn = self._btn("Export log")
        export_btn.setStyleSheet("padding:3px 10px;")
        export_btn.clicked.connect(self.export_log_dialog)
        clear = self._btn("Clear log")
        clear.setStyleSheet("padding:3px 10px;")
        clear.clicked.connect(self.clear_log)
        top.addWidget(export_btn)
        top.addWidget(clear)
        layout.addLayout(top)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(600)
        self.console.setFont(mono_font(12))
        layout.addWidget(self.console)

        wrap = QWidget()
        outer = QVBoxLayout(wrap)
        outer.setContentsMargins(12, 0, 12, 8)
        outer.addWidget(group)
        return wrap

    def log_event(self, category: str, text: str):
        stamp = time.strftime("%H:%M:%S")
        cat = category.upper()
        self._log_entries.append((stamp, cat, text))
        if len(self._log_entries) > 600:
            self._log_entries.pop(0)
        if self._log_filter == "ALL" or self._log_filter == cat:
            self.console.appendPlainText(f"[{stamp}] [{cat}] {text}")

    def set_log_filter(self, category: str):
        self._log_filter = category
        for key, btn in self.filter_buttons.items():
            btn.setChecked(key == category)
        lines = [
            f"[{ts}] [{cat}] {msg}"
            for ts, cat, msg in self._log_entries
            if category == "ALL" or cat == category
        ]
        self.console.setPlainText("\n".join(lines))

    def clear_log(self):
        self._log_entries.clear()
        self.console.clear()

    def export_log_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export rover log", str(Path.home() / "rover_log.txt"), "Text (*.txt)")
        if path:
            try:
                Path(path).write_text(
                    "\n".join(f"[{ts}] [{cat}] {msg}" for ts, cat, msg in self._log_entries),
                    encoding="utf-8",
                )
            except OSError as error:
                self.log_event("WARN", f"Cannot export log: {error}")

    def slider_row(self, key: str, minimum: int, maximum: int, default: int, suffix: str):
        row = QHBoxLayout()
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(int(self.settings.value(key, default)))
        value = QLabel(f"{slider.value()}{suffix}")
        value.setMinimumWidth(56)
        slider.valueChanged.connect(lambda number: value.setText(f"{number}{suffix}"))
        slider.valueChanged.connect(lambda number: self.settings.setValue(key, number))
        row.addWidget(slider, 1)
        row.addWidget(value)
        return slider, row

    def joystick_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.joystick_enabled = QCheckBox("Enable joystick input")
        self.joystick_enabled.setChecked(True)
        layout.addWidget(self.joystick_enabled)

        row = QHBoxLayout()
        self.joy_combo = QComboBox()
        self.joy_combo.currentIndexChanged.connect(self.select_joystick)
        row.addWidget(self.joy_combo, 1)
        refresh = self._btn("⟳")
        refresh.clicked.connect(self.refresh_joysticks)
        row.addWidget(refresh)
        layout.addLayout(row)

        calibrate = self._btn("CALIBRATE CENTRE · sticks at rest")
        calibrate.clicked.connect(self.calibrate_joystick)
        layout.addWidget(calibrate)
        self.centre_label = message("Centre offsets: uncalibrated")
        layout.addWidget(self.centre_label)

        self.axis_map, self.axis_inv = {}, {}
        form = QFormLayout()
        for key, title, original in (("drive", "Forward / reverse", "surge"), ("steer", "Steering", "sway"), ("turn", "Turning", "yaw")):
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
            arow = QHBoxLayout()
            arow.addWidget(combo, 1)
            arow.addWidget(invert)
            form.addRow(title, arow)
            self.axis_map[key], self.axis_inv[key] = combo, invert
        layout.addLayout(form)
        layout.addWidget(message("Live axes and buttons appear in PS4 Monitor."))
        self.joy_raw = message("No controller connected")
        self.joy_raw.setFont(mono_font(11))
        layout.addWidget(self.joy_raw)
        if not self.joy.ok:
            layout.addWidget(message(f"Gamepad unavailable: {self.joy.error}"))
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
        if hasattr(self, "command_bars"):
            self.clear_inputs()

    def calibrate_joystick(self):
        axes, _ = self.joy.poll()
        if not axes:
            self.centre_label.setText("Connect a controller first.")
            return
        self.joy_centres = dict(enumerate(axes))
        self.centre_label.setText("Centre offsets captured. Keep sticks neutral.")

    # ---------------- Safety & Mode Transitions ----------------

    def _on_output_mode_changed(self):
        mode = str(self.output_mode_combo.currentData() or OUTPUT_PREVIEW)
        self.safety.set_output_mode(mode)
        self.clear_inputs()
        if self.ros_link is not None:
            self.ros_link.publish_stop_burst()
        self._sync_safety_ui()
        self.log_event("CTRL", f"Output mode switched to {'ROS 2 Teleop (/cmd_vel)' if mode == OUTPUT_ROS_TELEOP else 'Preview Only (Safe)'}.")

    def set_preview(self, enabled: bool):
        self.safety.set_preview_enabled(enabled)
        self.clear_inputs()
        self._sync_safety_ui()
        self.log_event("CTRL", "Input preview enabled · no motor commands sent." if enabled else "Input preview stopped.")

    def toggle_arm(self, desired: bool):
        ok = self.safety.set_armed(desired)
        if desired and not ok:
            self.log_event("WARN", "Cannot ARM: switch to ROS 2 Teleop, connect ROS 2, and clear any E-STOP latch.")
        else:
            if not ok and self.ros_link is not None:
                self.ros_link.publish_stop_burst()
            self.log_event("CTRL", "Rover ARMED · live /cmd_vel & /goal_pose active!" if ok else "Rover DISARMED · outputs zeroed.")
        self.clear_inputs()
        self._sync_safety_ui()

    def emergency_stop(self):
        self.safety.trigger_estop()
        self.clear_inputs()
        self.filtered = [0.0, 0.0, 0.0]
        if self.ros_link is not None:
            self.ros_link.publish_stop_burst()
        if hasattr(self, "map_panel"):
            self.map_panel.cancel_mission()
        self._sync_safety_ui()
        self.log_event("WARN", "E-STOP triggered · outputs forced to zero and latched.")

    def reset_estop_latch(self):
        self.safety.reset_latch()
        self._sync_safety_ui()
        self.log_event("INFO", "E-STOP latch cleared · rover remains DISARMED.")

    def stop_action(self):
        if self.safety.can_publish_ros():
            self.emergency_stop()
        else:
            self.safety.set_preview_enabled(False)
            self.clear_inputs()
            self._sync_safety_ui()
            self.log_event("CTRL", "Preview stopped.")

    def _sync_safety_ui(self):
        st = self.safety.state()
        self.armed = self.safety.can_publish_ros()

        self.preview_button.blockSignals(True)
        self.preview_button.setEnabled(self.safety.output_mode == OUTPUT_PREVIEW and not self.safety.estop_latched)
        self.preview_button.setChecked(self.safety.preview_enabled)
        self.preview_button.blockSignals(False)

        can_arm = (
            self.safety.output_mode == OUTPUT_ROS_TELEOP
            and self.safety.ros_connected
            and not self.safety.estop_latched
        )
        self.arm_button.blockSignals(True)
        self.arm_button.setEnabled(can_arm)
        self.arm_button.setChecked(self.safety.armed)
        self.arm_button.setToolTip(
            "Toggle live /cmd_vel + /goal_pose output"
            if can_arm
            else "Switch Output to ROS 2 Teleop, connect ROS 2, and clear E-STOP latch to ARM"
        )
        self.arm_button.blockSignals(False)
        self.reset_latch_button.setEnabled(self.safety.estop_latched)
        self.estop_button.setEnabled(True)

        if st == SAFETY_ESTOP:
            self.safety_pill.set_state("E-STOP LATCHED", "bad", True)
            self.preview_hint.setText("E-STOP LATCHED · press RESET LATCH to unlock.")
        elif st == SAFETY_ARMED:
            self.safety_pill.set_state("ARMED · /cmd_vel", "ok", True)
            self.preview_hint.setText("ARMED · hold WASDQE / sticks to drive · Space E-STOPs immediately.")
        elif st == SAFETY_DISARMED:
            self.safety_pill.set_state("DISARMED", "warn", False)
            self.preview_hint.setText(
                "Connect ROS 2 and press ARM to enable /cmd_vel and Nav2 goals."
                if not self.safety.ros_connected
                else "ROS connected · press ARM to enable live /cmd_vel & Nav2 goals."
            )
        else:
            if self.safety.preview_enabled:
                self.safety_pill.set_state("PREVIEW ACTIVE", "warn", True)
                self.preview_hint.setText("PREVIEW ONLY · no commands sent to ROS · Space stops.")
            else:
                self.safety_pill.set_state("PREVIEW ONLY", "idle", False)
                self.preview_hint.setText("Safe preview mode · enable INPUT PREVIEW or switch to ROS 2 Teleop.")

        controls_active = self.safety.preview_enabled or self.safety.can_publish_ros()
        for button in self.preview_controls:
            button.setEnabled(controls_active)

        if hasattr(self, "map_panel"):
            self.map_panel.set_goal_allowed(self.safety.can_publish_ros() and not self.gazebo_teleop)

    def clear_inputs(self):
        self.keys.clear()
        self.test_hold = None
        self.filtered = [0.0, 0.0, 0.0]
        if hasattr(self, "test_pill"):
            self.test_pill.set_state("TEST IDLE", "idle")
        if hasattr(self, "test_hold_buttons"):
            for btn in self.test_hold_buttons:
                if btn.property("active"):
                    btn.setProperty("active", False)
                    repolish(btn)

    # ---------------- Motor Test Station Window ----------------

    def open_motor_window(self):
        self.clear_inputs()
        self.motor_window.show()
        self.motor_window.raise_()
        self.motor_window.activateWindow()

    def motor_test_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        header = QHBoxLayout()
        self.test_pill = Pill("TEST IDLE", "idle")
        header.addWidget(self.test_pill)

        enable = self._btn("Enable / disable input preview")
        enable.clicked.connect(self._toggle_preview_from_test_window)
        header.addWidget(enable)

        stop = self._btn("STOP  [Space]", "estop")
        stop.clicked.connect(self.stop_action)
        header.addWidget(stop)
        layout.addLayout(header)

        layout.addWidget(
            message("Hold keys 1–4 (drive) or 5–8 (steering) or Hold buttons. Releasing stops immediately. Steering encoder feedback is deferred.")
        )

        # Common speed / steering / reverse controls (parity with ROV)
        common = QGroupBox("Common test levels · direction")
        ch = QHBoxLayout(common)
        ch.addWidget(QLabel("Drive %:"))
        self.common_drive_slider = QSlider(Qt.Orientation.Horizontal)
        self.common_drive_slider.setRange(5, 100)
        self.common_drive_slider.setValue(25)
        self.common_drive_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        ch.addWidget(self.common_drive_slider, 1)
        common_d_lbl = QLabel("25%")
        self.common_drive_slider.valueChanged.connect(lambda v: common_d_lbl.setText(f"{v}%"))
        ch.addWidget(common_d_lbl)

        ch.addSpacing(12)
        ch.addWidget(QLabel("Steer °:"))
        self.common_steer_slider = QSlider(Qt.Orientation.Horizontal)
        self.common_steer_slider.setRange(5, 90)
        self.common_steer_slider.setValue(30)
        self.common_steer_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        ch.addWidget(self.common_steer_slider, 1)
        common_s_lbl = QLabel("30°")
        self.common_steer_slider.valueChanged.connect(lambda v: common_s_lbl.setText(f"{v}°"))
        ch.addWidget(common_s_lbl)

        self.test_reverse_all = QCheckBox("Reverse all")
        self.test_reverse_all.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        ch.addWidget(self.test_reverse_all)
        layout.addWidget(common)

        self.test_sliders: dict[int, QSlider] = {}
        self.test_inversions: dict[int, QCheckBox] = {}
        self.test_live_labels: dict[int, QLabel] = {}
        self.test_hold_buttons: list[QPushButton] = []
        self._channel_buttons: dict[tuple[int, int], QPushButton] = {}

        for offset, prefix, title, maximum, unit in (
            (0, "D", "Drive channels [1–4]", 100, "%"),
            (4, "S", "Steering channels [5–8] · preview only", 90, "°"),
        ):
            group = QGroupBox(title)
            grid = QGridLayout(group)
            for index, wheel in enumerate(WHEELS):
                channel = offset + index
                grid.addWidget(QLabel(f"[{channel + 1}] {prefix}{index + 1} · {wheel}"), index, 0)
                slider, row = self.slider_row(f"motor_preview/{channel}", 1, maximum, 25 if prefix == "D" else 30, unit)
                self.test_sliders[channel] = slider
                grid.addLayout(row, index, 1)

                invert = QCheckBox("Invert")
                invert.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                invert.setChecked(self.settings.value(f"motor_invert/{channel}", False, type=bool))
                invert.toggled.connect(lambda value, ch_id=channel: self.settings.setValue(f"motor_invert/{ch_id}", value))
                self.test_inversions[channel] = invert
                grid.addWidget(invert, index, 2)

                for col, sign, text in ((3, -1, "Hold −"), (4, 1, "Hold +")):
                    button = QPushButton(text)
                    button.setObjectName("motorTestKey")
                    button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                    button.pressed.connect(lambda ch_id=channel, s=sign: self.hold_motor(ch_id, s))
                    button.released.connect(lambda ch_id=channel: self.release_motor(ch_id))
                    grid.addWidget(button, index, col)
                    self.test_hold_buttons.append(button)
                    self._channel_buttons[(channel, sign)] = button

                live_lbl = QLabel("-- rpm" if prefix == "D" else "Deferred")
                live_lbl.setFont(mono_font(11))
                live_lbl.setMinimumWidth(88)
                self.test_live_labels[channel] = live_lbl
                grid.addWidget(live_lbl, index, 5)
            layout.addWidget(group)

        self.common_drive_slider.valueChanged.connect(
            lambda v: [self.test_sliders[i].setValue(v) for i in range(4)]
        )
        self.common_steer_slider.valueChanged.connect(
            lambda v: [self.test_sliders[i].setValue(v) for i in range(4, 8)]
        )
        layout.addStretch()
        return page

    def _toggle_preview_from_test_window(self):
        if self.safety.output_mode != OUTPUT_PREVIEW:
            self.output_mode_combo.setCurrentIndex(0)
        self.preview_button.setChecked(not self.preview_button.isChecked())

    def hold_motor(self, channel: int, sign: int):
        if not (self.safety.preview_enabled or self.safety.can_publish_ros()):
            self.test_pill.set_state("ENABLE PREVIEW OR ARM", "warn")
            return
        self.clear_inputs()
        effective_sign = -sign if self.test_reverse_all.isChecked() else sign
        self.test_hold = channel, effective_sign
        btn = self._channel_buttons.get((channel, sign))
        if btn is not None:
            btn.setProperty("active", True)
            repolish(btn)
        self.test_pill.set_state(f"CH {channel + 1} ACTIVE", "warn", True)

    def release_motor(self, channel: int):
        if self.test_hold and self.test_hold[0] == channel:
            self.clear_inputs()

    # ---------------- Event Filter & Input Loop ----------------

    def eventFilter(self, obj, event):
        etype = event.type()
        if etype == QEvent.Type.FocusIn and isinstance(obj, (QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox)):
            self.clear_inputs()
        if etype == QEvent.Type.ApplicationDeactivate or (
            etype in (QEvent.Type.WindowDeactivate, QEvent.Type.Close)
            and obj in (self, self.motor_window)
        ):
            self.clear_inputs()
            if self.ros_link is not None and self.safety.can_publish_ros():
                self.ros_link.publish_stop_burst()
            return super().eventFilter(obj, event)

        if etype not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return super().eventFilter(obj, event)
        if QApplication.activeWindow() not in (self, self.motor_window):
            return super().eventFilter(obj, event)
        if isinstance(QApplication.focusWidget(), (QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox)):
            return super().eventFilter(obj, event)
        if event.isAutoRepeat():
            return False

        key = event.key()
        down = etype == QEvent.Type.KeyPress

        # Space / Esc ALWAYS stops cleanly regardless of mode or preview toggle.
        if key in (Qt.Key.Key_Space, Qt.Key.Key_Escape):
            if down:
                self.stop_action()
            return True

        if not (self.safety.preview_enabled or self.safety.can_publish_ros()):
            return super().eventFilter(obj, event)

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
        dt, self.last_tick = min(0.1, now - self.last_tick), now
        if not self.isVisible() and not self.motor_window.isVisible():
            return

        if self.joy.changed():
            self.refresh_joysticks()
        axes, buttons = self.joy.poll()

        # Feed PS4Monitor only when its tab or Joystick settings tab is active.
        ps4_visible = (
            self.center_tabs.currentWidget() and self.center_tabs.currentIndex() == 4
        ) or self.settings_tabs.currentIndex() == 1
        if ps4_visible:
            if not self.joy.ok:
                self.ps4_monitor.status.setText("pygame not installed — install pygame for gamepad support")
            else:
                self.ps4_monitor.feed(self.joy.name, axes, buttons, self.joy.hats)
        if self.settings_tabs.currentIndex() == 1:
            self.joy_raw.setText(
                "  ".join(f"{index}: {value:+.2f}" for index, value in enumerate(axes))
                or ("pygame not installed" if not self.joy.ok else "No controller connected")
            )

        active_output = self.safety.preview_enabled or self.safety.can_publish_ros()
        active_win = QApplication.activeWindow() in (self, self.motor_window)
        target = [0.0, 0.0, 0.0]

        if active_output and active_win:
            target = [
                100.0 * (("W" in self.keys) - ("S" in self.keys)),
                100.0 * (("D" in self.keys) - ("A" in self.keys)),
                100.0 * (("E" in self.keys) - ("Q" in self.keys)),
            ]
            if self.joystick_enabled.isChecked() and axes:
                dz = self.drive_settings["deadzone"].value()
                expo = self.drive_settings["expo"].value()
                for index, name in enumerate(("drive", "steer", "turn")):
                    axis = self.axis_map[name].currentData()
                    if axis is None or not 0 <= axis < len(axes):
                        continue
                    centered = axes[axis] - self.joy_centres.get(axis, 0.0)
                    shaped = shape_axis(centered, dz, expo, self.axis_inv[name].isChecked())
                    target[index] = max(-100.0, min(100.0, target[index] + shaped))
            target[0] *= self.power_slider.value() / 100.0

        ramp = self.drive_settings["ramp"].value() / 1000.0
        blend = 1.0 if ramp <= 0 else min(1.0, dt * 4.0 / ramp)
        self.filtered = [cur + (want - cur) * blend for cur, want in zip(self.filtered, target)]
        drive, steer, turn = self.filtered

        mode = self.steering_mode.currentIndex()
        power_pct = float(self.power_slider.value())
        wheel_drive, steering, turning_radius = compute_wheel_kinematics(
            drive, steer, turn, mode, self.steer_limit.value(), power_pct
        )

        # Handle single-channel hold in Motor Test Window
        if active_output and active_win and self.test_hold is not None:
            channel, sign = self.test_hold
            inv = -1 if self.test_inversions[channel].isChecked() else 1
            value = self.test_sliders[channel].value() * sign * inv
            unit = "%" if channel < 4 else "°"
            self.test_pill.set_state(f"CH {channel + 1} · {value:+.0f}{unit}", "warn", True)
            wheel_drive = dict.fromkeys(WHEELS, 0.0)
            steering = dict.fromkeys(WHEELS, 0.0)
            (wheel_drive if channel < 4 else steering)[WHEELS[channel % 4]] = value
            turning_radius = None
            # When holding single drive channel while armed, yield zero overall chassis Twist unless differential intended:
            # Keep /cmd_vel strictly zero during individual test-window holds for bench safety.
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = compute_cmd_vel(
                drive, steer, turn, mode, self.speed_limit.value(), self.yaw_limit.value(), power_pct
            )

        if self.gazebo_teleop:
            # Gazebo currently has fixed steering and differential wheel speeds.
            steering = dict.fromkeys(WHEELS, 0.0)
            turning_radius = vx / wz if abs(wz) > 1e-6 else None
            left, right = vx - wz * 0.49 / 2, vx + wz * 0.49 / 2
            scale = max(self.speed_limit.value(), abs(left), abs(right), 0.001)
            wheel_drive = {w: 100 * (left if w.endswith("L") else right) / scale for w in WHEELS}

        # Gated ROS /cmd_vel publish
        if self.ros_link is not None:
            if self.safety.can_publish_ros():
                self.ros_link.set_cmd_vel(vx, vy, wz, True)
            else:
                self.ros_link.set_cmd_vel(0.0, 0.0, 0.0, False)

        # Keep STLViewer MotionPreview struct updated
        self.cmd = {"surge": drive, "sway": steer, "yaw": turn, "heave": 0.0, "roll": 0.0}
        self.out_cmd = dict(self.cmd) if active_output else dict.fromkeys(self.cmd, 0.0)

        self.stick_left.set_xy(steer, drive)
        self.stick_right.set_xy(turn, 0.0)
        for bar, value in zip(self.command_bars, self.filtered):
            bar.set_value(value if active_output else 0.0)
        for key, button in self.manual_buttons.items():
            button.setDown(key in self.keys and active_output)

        self.rover_view.set_commands(
            active_output,
            wheel_drive,
            steering,
            steering_mode=mode,
            safety_state=self.safety.state(),
            drive_axis=drive,
            steer_axis=steer,
            turn_axis=turn,
            turning_radius_m=turning_radius,
        )

    def reset_preview_settings(self):
        self.safety.set_preview_enabled(False)
        self.power_slider.setValue(50)
        for key, value in (("expo", 30), ("deadzone", 8), ("ramp", 350)):
            self.drive_settings[key].setValue(value)
        self.speed_limit.setValue(0.5)
        self.yaw_limit.setValue(1.0)
        self.steer_limit.setValue(30.0)
        self.steering_mode.setCurrentIndex(MODE_COORDINATED)
        self._sync_safety_ui()

    # ---------------- Tab Sync & Safe Fade ----------------

    def _on_mode_combo_changed(self, index: int):
        if 0 <= index < self.center_tabs.count() and self.center_tabs.currentIndex() != index:
            self.center_tabs.setCurrentIndex(index)

    def _on_center_tab_changed(self, index: int):
        if 0 <= index < self.mode.count() and self.mode.currentIndex() != index:
            self.mode.blockSignals(True)
            self.mode.setCurrentIndex(index)
            self.mode.blockSignals(False)
        self.fade_tab(index)

    def fade_tab(self, index: int):
        if self.tab_animation is not None:
            self.tab_animation.stop()
            self.tab_animation = None
        if self.tab_effect is not None:
            parent_w = self.tab_effect.parent()
            if isinstance(parent_w, QWidget):
                parent_w.setGraphicsEffect(None)
            self.tab_effect = None

        widget = self.center_tabs.widget(index)
        # Never attach QGraphicsOpacityEffect to the OpenGL VTK STL tab.
        if widget is None or widget is self.stl_viewer:
            return

        effect = QGraphicsOpacityEffect(widget)
        widget.setGraphicsEffect(effect)
        self.tab_effect = effect
        anim = QPropertyAnimation(effect, b"opacity", self)
        anim.setDuration(150)
        anim.setStartValue(0.45)
        anim.setEndValue(1.0)
        anim.setEasingCurve(QEasingCurve.Type.OutCubic)

        def cleanup():
            if self.tab_effect is effect:
                widget.setGraphicsEffect(None)
                self.tab_effect = None

        anim.finished.connect(cleanup)
        self.tab_animation = anim
        anim.start()

    # ---------------- Unified Telemetry CSV & Export ----------------

    def start_recording_to(self, path: str) -> None:
        self.stop_recording()
        self.record_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.record_file, fieldnames=TELEMETRY_CSV_FIELDS)
        self.csv_writer.writeheader()
        self._last_csv_write = 0.0

    def record_telemetry_toggled(self, enabled: bool):
        if not enabled:
            self.stop_recording()
            self.log_event("INFO", "Telemetry CSV recording stopped.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Record rover telemetry", str(Path.home() / "rover_telemetry.csv"), "CSV (*.csv)"
        )
        if not path:
            self.record_button.blockSignals(True)
            self.record_button.setChecked(False)
            self.record_button.blockSignals(False)
            return
        try:
            self.start_recording_to(path)
            self.log_event("INFO", f"Recording unified telemetry CSV to {Path(path).name}.")
        except OSError as error:
            self.log_event("WARN", f"Cannot record telemetry: {error}")
            self.stop_recording()

    def write_telemetry_row(self):
        if self.csv_writer is None:
            return
        now = time.monotonic()
        if now - self._last_csv_write < 0.04:
            return
        self._last_csv_write = now
        row = {
            "monotonic_s": f"{now:.4f}",
            "pose_x": f"{self.latest_odom['x']:.4f}" if self.latest_odom else "",
            "pose_y": f"{self.latest_odom['y']:.4f}" if self.latest_odom else "",
            "pose_yaw_deg": f"{self.latest_odom['yaw_deg']:.2f}" if self.latest_odom else "",
            "speed_mps": f"{self.latest_odom['speed_mps']:.4f}" if self.latest_odom else "",
            "roll": f"{self.imu.get('roll', ''):.3f}" if "roll" in self.imu else "",
            "pitch": f"{self.imu.get('pitch', ''):.3f}" if "pitch" in self.imu else "",
            "yaw": f"{self.imu.get('yaw', ''):.3f}" if "yaw" in self.imu else "",
            "gx": f"{self.imu.get('gx', ''):.3f}" if "gx" in self.imu else "",
            "gy": f"{self.imu.get('gy', ''):.3f}" if "gy" in self.imu else "",
            "gz": f"{self.imu.get('gz', ''):.3f}" if "gz" in self.imu else "",
            "ax": f"{self.imu.get('ax', ''):.4f}" if "ax" in self.imu else "",
            "ay": f"{self.imu.get('ay', ''):.4f}" if "ay" in self.imu else "",
            "az": f"{self.imu.get('az', ''):.4f}" if "az" in self.imu else "",
            "slip_detected": int(self.slip_detected),
        }
        for w in WHEELS:
            r = self.readings.get(f"D_{w}", {})
            lw = w.lower()
            row[f"{lw}_ticks"] = r.get("ticks", "")
            row[f"{lw}_rpm"] = f"{r['value']:.2f}" if "value" in r else ""
        try:
            self.csv_writer.writerow(row)
        except OSError as error:
            self.log_event("WARN", f"Telemetry CSV stopped: {error}")
            self.stop_recording()

    def stop_recording(self):
        if self.record_file is not None:
            try:
                self.record_file.close()
            except OSError:
                pass
        self.record_file = self.csv_writer = None
        if hasattr(self, "record_button"):
            self.record_button.blockSignals(True)
            self.record_button.setChecked(False)
            self.record_button.blockSignals(False)

    def export_encoders_to(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("encoder", "motor", "wheel", "raw_ticks", "speed_mps", "rpm_or_angle", "status"))
            for row in range(8):
                writer.writerow(self.encoder_table.item(row, col).text() for col in range(7))

    def export_encoders(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export drive encoder readings", str(Path.home() / "rover_encoders.csv"), "CSV (*.csv)")
        if not path:
            return
        try:
            self.export_encoders_to(path)
            self.log_event("INFO", "Encoder snapshot exported.")
        except OSError as error:
            self.log_event("WARN", f"Cannot export encoders: {error}")

    # ---------------- ROS Connection & Handlers ----------------

    def _on_ros_camera_feed_changed(self, feed_key):
        if self.ros_link is not None and self.ready:
            self.ros_link.set_active_camera_feed(feed_key)

    def send_nav_goal(self, x: float, y: float, yaw_rad: float):
        if self.gazebo_teleop:
            self.log_event("WARN", "Standalone Gazebo teleop does not run Nav2. Navigation goals are disabled.")
            return
        if not self.safety.can_publish_ros() or self.ros_link is None:
            self.log_event("WARN", "Ignored Nav2 goal: rover must be connected, in ROS 2 Teleop mode, and ARMED.")
            return
        self.ros_link.publish_goal_pose(x, y, yaw_rad, "map")

    def toggle_ros(self):
        if self.ros_link is not None:
            self.ready = False
            self.safety.on_ros_connected(False)
            self.camera.set_ros_connected(False)
            self._sync_safety_ui()
            self.connect_button.setEnabled(False)
            self.ros_link.requestInterruption()
            return

        topics = {key: field.text().strip() for key, field in self.topic_fields.items()}
        if any(any(c.isspace() for c in topic) for topic in topics.values()):
            self.log_event("WARN", "Topic names cannot contain spaces. Fix under the ROS tab before connecting.")
            return
        if not any(topics.get(k) for k in ("joints", "ticks", "wheel_speeds", "imu", "pose", "map")):
            self.log_event("WARN", "Enter at least one telemetry topic under the ROS tab before connecting.")
            return

        self.readings.clear()
        self.topic_rates.clear()
        self.diag_entries.clear()
        self.last_imu = float("-inf")
        self.last_odom_time = float("-inf")
        self.last_slip_time = float("-inf")
        self.last_map_time = float("-inf")
        self.last_perception_time = float("-inf")
        for plot in self.imu_plots:
            plot.clear()

        self.ros_link = RoverRosLink(topics, self.encoder_source.currentText(), self.wheel_radius.value(), self,
                                     use_sim_time=self.profile.currentText() == "Simulation")
        self.ros_link.status.connect(self.ros_status)
        self.ros_link.encoders.connect(self.receive_encoder)
        self.ros_link.imu.connect(self.receive_imu)
        self.ros_link.odometry.connect(self.receive_odometry)
        self.ros_link.slip.connect(self.receive_slip)
        self.ros_link.map_data.connect(self.receive_map)
        self.ros_link.path_data.connect(self.map_panel.set_path)
        self.ros_link.aruco_pose.connect(self.receive_aruco)
        self.ros_link.obstacles.connect(self.receive_obstacles)
        self.ros_link.diagnostics.connect(self.receive_diagnostics)
        self.ros_link.camera_frame.connect(self.camera.receive_ros_frame)
        self.ros_link.topic_rates.connect(self.receive_topic_rates)
        self.ros_link.discovered_topics.connect(self.receive_discovered_topics)
        self.ros_link.finished.connect(self.ros_finished)

        self.connect_button.setText("DISCONNECT")
        self.log_event("ROS", "Starting ROS 2 link (read-only telemetry until ARMED)…")
        self.ros_link.start()

    def ros_status(self, listening: bool, detail: str):
        self.ready = listening
        self.safety.on_ros_connected(listening)
        self.camera.set_ros_connected(listening)
        self.connection_pill.set_state("ROS listening" if listening else "Disconnected", "warn" if listening else "idle")
        self.telemetry_label.setText(detail)
        self._sync_safety_ui()
        self.log_event("ROS" if listening else "WARN", detail)
        if listening and self.ros_link is not None and self.camera._active_ros_feed:
            self.ros_link.set_active_camera_feed(self.camera._active_ros_feed)

    def ros_finished(self):
        self.ready = False
        self.safety.on_ros_connected(False)
        self.camera.set_ros_connected(False)
        self.last_imu = float("-inf")
        if self.ros_link is not None:
            self.ros_link.deleteLater()
            self.ros_link = None
        self.connect_button.setText("CONNECT ROS 2")
        self.connect_button.setEnabled(True)
        self.connection_pill.set_state("Disconnected", "idle")
        self.telemetry_label.setText("Telemetry: disconnected")
        self._sync_safety_ui()
        self.log_event("ROS", "ROS 2 connection stopped.")
        self.refresh_readings()

    def zero_yaw(self):
        self.yaw_offset_deg = self.raw_yaw_deg
        self.stl_viewer.zero_view()
        self.log_event("INFO", f"Compass heading zeroed at raw yaw {self.raw_yaw_deg:+.1f}°.")

    def receive_encoder(self, motor: str, ticks, rpm, speed_mps=None):
        if self.ready:
            self.update_encoder(motor, ticks, rpm, speed_mps)

    def receive_imu(self, values: dict):
        if not self.ready:
            return
        self.last_imu = time.monotonic()
        if "yaw" in values:
            self.raw_yaw_deg = float(values["yaw"])
        adjusted = dict(values)
        if "yaw" in adjusted:
            adjusted["yaw"] = wrap_deg180(adjusted["yaw"] - self.yaw_offset_deg)
        self.imu.update(adjusted)

        roll = self.imu.get("roll", 0.0)
        pitch = self.imu.get("pitch", 0.0)
        yaw = self.imu.get("yaw", 0.0)

        if all(k in values for k in ("roll", "pitch")):
            self.horizon.set_attitude(roll, pitch)
        if "yaw" in adjusted:
            self.compass.set_yaw(yaw)
        if all(k in adjusted for k in ("roll", "pitch", "yaw")):
            self.attitude_plot.add(self.last_imu, [roll, pitch, yaw])
        for plot, keys in ((self.gyro_plot, ("gx", "gy", "gz")), (self.accel_plot, ("ax", "ay", "az"))):
            if all(k in adjusted for k in keys):
                plot.add(self.last_imu, [adjusted[k] for k in keys])

        for key in ("roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az"):
            if key in self.imu:
                self.imu_lbl[key].setText(f"{key.upper():<5}{self.imu[key]:+7.2f}")

        self.imu_cards["HEADING"].set(f"{yaw:+.1f}°")
        self.imu_cards["CHASSIS TILT"].set(f"{max(abs(roll), abs(pitch)):.1f}°")
        self.rover_view.set_alerts(self.slip_detected, roll, pitch)
        self.write_telemetry_row()

    def receive_odometry(self, odom: dict):
        if not self.ready:
            return
        self.last_odom_time = time.monotonic()
        self.latest_odom = dict(odom)
        self.map_panel.set_odometry(odom)
        self.imu_cards["ROVER SPEED"].set(f"{odom['speed_mps']:.2f} m/s")
        self.imu_cards["POSE (X, Y)"].set(f"{odom['x']:+.1f}, {odom['y']:+.1f}")
        self.write_telemetry_row()

    def receive_slip(self, slip_detected: bool, single_slip: bool):
        if not self.ready:
            return
        self.last_slip_time = time.monotonic()
        self.slip_detected = bool(slip_detected)
        self.single_slip_detected = bool(single_slip)
        any_slip = self.slip_detected or self.single_slip_detected
        self.slip_banner.setVisible(any_slip)
        self.imu_cards["WHEEL SLIP"].set("SLIP!" if any_slip else "OK")
        self.rover_view.set_alerts(any_slip, self.imu.get("roll", 0.0), self.imu.get("pitch", 0.0))

    def receive_map(self, info: dict):
        if not self.ready:
            return
        self.last_map_time = time.monotonic()
        self.map_panel.set_map(info)

    def receive_aruco(self, pose: dict):
        if not self.ready:
            return
        self.last_perception_time = time.monotonic()
        self.map_panel.set_aruco(pose)

    def receive_obstacles(self, dets: list):
        if not self.ready:
            return
        self.last_perception_time = time.monotonic()
        self.map_panel.set_obstacles(dets)

    def receive_diagnostics(self, source: str, entries: list):
        self.diag_entries[source] = entries
        rows = []
        lvl_names = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
        for src, items in self.diag_entries.items():
            for e in items:
                rows.append((src, e["name"], lvl_names.get(e["level"], str(e["level"])), e["message"]))
        self.diag_table.setRowCount(len(rows))
        for r, cols in enumerate(rows):
            for c, val in enumerate(cols):
                self.diag_table.setItem(r, c, QTableWidgetItem(val))

    def receive_topic_rates(self, rates: dict):
        self.topic_rates = dict(rates)
        now = time.monotonic()
        for row, (key, title, _, _, _) in enumerate(TOPIC_DEFS):
            topic_txt = self.topic_fields[key].text().strip() if hasattr(self, "topic_fields") else "--"
            info = rates.get(key)
            self.rates_table.item(row, 1).setText(topic_txt or "(disabled)")
            if info and now - info["last_seen"] <= 2.5:
                self.rates_table.item(row, 2).setText(f"{info['hz']:.1f}")
                self.rates_table.item(row, 3).setText("Live")
            else:
                self.rates_table.item(row, 2).setText("--")
                self.rates_table.item(row, 3).setText("Disabled" if not topic_txt else "Waiting")

    def update_encoder(self, motor: str, ticks, value, speed_mps=None):
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
        if speed_mps is not None and math.isfinite(speed_mps):
            reading.update(speed_mps=float(speed_mps))
        elif value is not None and hasattr(self, "wheel_radius"):
            r = self.wheel_radius.value()
            reading.update(speed_mps=float(value) * (2.0 * math.pi * r) / 60.0)
        self.refresh_readings()

    def refresh_readings(self):
        now = time.monotonic()
        fresh = {}
        for row, motor in enumerate(DRIVE_MOTORS):
            reading = self.readings.get(motor, {})
            speed_live = self.ready and now - reading.get("received", float("-inf")) <= 2.0
            ticks_live = self.ready and now - reading.get("ticks_received", float("-inf")) <= 2.0
            ticks = str(reading["ticks"]) if ticks_live else "STALE" if "ticks" in reading else "--"
            rpm = f"{reading['value']:+.1f} rpm" if speed_live else "STALE" if "value" in reading else "-- rpm"
            mps = f"{reading['speed_mps']:+.2f}" if speed_live and "speed_mps" in reading else "--"
            self.encoder_table.item(row, 3).setText(ticks)
            self.encoder_table.item(row, 4).setText(mps)
            self.encoder_table.item(row, 5).setText(rpm)
            status = "Live" if speed_live and ticks_live else "RPM only" if speed_live else "Ticks only" if ticks_live else "Stale" if reading else "No data"
            self.encoder_table.item(row, 6).setText(status)
            self.motor_cards[motor].set(f"{rpm}" + (f" ({mps} m/s)" if mps != "--" else ""))
            if hasattr(self, "test_live_labels") and row in self.test_live_labels:
                self.test_live_labels[row].setText(rpm)
            if speed_live:
                fresh[motor] = reading

        self.rover_view.set_readings(fresh)

        imu_live = self.ready and now - self.last_imu <= 2.0
        odom_live = self.ready and now - self.last_odom_time <= 2.0
        slip_live = self.ready and now - self.last_slip_time <= 3.0
        map_live = self.ready and (now - self.last_map_time <= 10.0 or self.map_panel.canvas.map_info is not None)
        perc_live = self.ready and now - self.last_perception_time <= 3.0

        self.horizon.setEnabled(imu_live)
        self.compass.setEnabled(imu_live)
        if imu_live:
            hz = self.topic_rates.get("imu", {}).get("hz")
            self.imu_rate_lbl.setText(f"rate {hz:.1f} Hz" if hz else "rate -- Hz")
            self.imu_pill.set_state("IMU: LIVE", "ok", True)
        else:
            self.imu_rate_lbl.setText("rate -- Hz")
            self.imu_pill.set_state("IMU: no data" if not self.ready else "IMU: stale", "idle" if not self.ready else "warn")

        if not odom_live and self.ready:
            self.imu_cards["ROVER SPEED"].set("STALE" if self.latest_odom else "--")

        # Update Subsystem Health Matrix
        def set_health(key: str, live: bool, rate_key: str | None = None, note_off: str = "Offline", note_wait: str = "Waiting"):
            lbl = self.health_labels[key]
            if not self.ready:
                lbl.setText(note_off)
                lbl.setStyleSheet("color:#8aa0b6;")
            elif live:
                hz = self.topic_rates.get(rate_key, {}).get("hz") if rate_key else None
                lbl.setText(f"LIVE · {hz:.1f} Hz" if hz else "LIVE")
                lbl.setStyleSheet("color:#4ade80; font-weight:700;")
            else:
                lbl.setText(note_wait)
                lbl.setStyleSheet("color:#fbbf24;")

        set_health("encoders", len(fresh) > 0, "joints" if self.encoder_source.currentIndex() == 0 else "wheel_speeds")
        set_health("imu", imu_live, "imu")
        set_health("slam", odom_live, "pose")
        set_health("nav2", map_live, "path")
        set_health("perception", perc_live or (self.camera.last_kind == "live"), "aruco_pose")
        set_health("slip", slip_live, "slip")

        if self.ready:
            any_live = bool(fresh or imu_live or odom_live)
            self.telemetry_label.setText(f"Drive RPM: {len(fresh)}/4 · IMU: {'OK' if imu_live else '--'} · steering deferred")
            self.connection_pill.set_state("Telemetry live" if any_live else "ROS listening", "ok" if any_live else "warn")

        state = "ROS listening" if self.ready else "OFFLINE"
        safety_tag = self.safety.state().upper()
        self.statusBar().showMessage(f"{state} · {safety_tag} · 4 drive encoders · steering feedback deferred")

    # ---------------- ROS Topic Settings ----------------

    def configure_gazebo_teleop(self):
        """Apply a session-only standalone profile without changing saved topics."""
        self.gazebo_teleop = True
        self.profile.setCurrentText("Simulation")
        for key, _title, sim_default, _kind, _hardware in TOPIC_DEFS:
            self.topic_fields[key].setText(sim_default)
        for key in ("ticks", "wheel_speeds", "slip", "single_slip", "map", "path",
                    "aruco_pose", "obstacles", "aruco_debug", "pose_debug", "enc_diag",
                    "slip_diag", "goal_pose"):
            self.topic_fields[key].setText("")
        self.topic_fields["pose"].setText("/odom")
        self.topic_fields["cmd_vel"].setText("/cmd_vel_manual")
        self.encoder_source.setCurrentText("Joint states")
        self.wheel_radius.setValue(0.06)
        self.profile.setEnabled(False)
        self.steering_mode.setCurrentIndex(MODE_COORDINATED)
        self.steering_mode.setItemText(MODE_COORDINATED, "Skid steer (Gazebo)")
        self.steering_mode.setEnabled(False)
        self.drivetrain_label.setText("Gazebo · 4WD skid steer")
        self.steering_group.setTitle("Skid-steer drive")
        self.steering_note.setText("Gazebo wheels are fixed. Turning uses different left/right wheel speeds.")
        self.rover_view.fixed_steering = True
        self.output_mode_combo.setItemText(1, "ROS 2 Teleop (/cmd_vel_manual)")
        self.setWindowTitle("Control Station · Gazebo skid-steer teleop")
        self.setup_status.setText("Standalone Gazebo: /odom, fixed steering, manual commands via timeout guard. Nav2 is off.")
        self._sync_safety_ui()
        self.log_event("INFO", "Gazebo teleop profile loaded. CONNECT ROS 2, select ROS Teleop, then ARM. Camera: select ROS RGB or Depth and Connect.")

    def setup_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(
            message("Choose environment & encoder source, then click CONNECT ROS 2. Leave unused topics blank to skip.")
        )
        form = QFormLayout()
        self.profile = QComboBox()
        self.profile.addItems(["Simulation", "Hardware"])
        form.addRow("Environment", self.profile)

        self.encoder_source = QComboBox()
        self.encoder_source.addItems(["Joint states", "Ticks and wheel speeds", "Individual tick topics"])
        form.addRow("Drive feedback", self.encoder_source)

        self.wheel_radius = QDoubleSpinBox()
        self.wheel_radius.setRange(0.001, 2.0)
        self.wheel_radius.setDecimals(4)
        self.wheel_radius.setSuffix(" m")
        form.addRow("Wheel radius", self.wheel_radius)
        layout.addLayout(form)

        self.topic_fields: dict[str, QLineEdit] = {}
        for key, title, sim_default, kind, _hw_default in TOPIC_DEFS:
            layout.addWidget(QLabel(title))
            field = QLineEdit(sim_default)
            field.setToolTip(f"{kind} (leave blank to disable)")
            self.topic_fields[key] = field
            layout.addWidget(field)

        self.setup_status = message("")
        layout.addWidget(self.setup_status)
        save = self._btn("Save Rover topic settings")
        save.clicked.connect(self.save_topics)
        layout.addWidget(save)

        reset_topics = self._btn("Restore profile defaults")
        reset_topics.clicked.connect(self.restore_profile_defaults)
        layout.addWidget(reset_topics)

        self.profile.setCurrentText(self.settings.value("profile", "Simulation"))
        self.profile.currentTextChanged.connect(self.load_topics)
        self.load_topics()
        return page

    def load_topics(self):
        profile = self.profile.currentText()
        for key, _title, sim_default, _kind, hw_default in TOPIC_DEFS:
            default = hw_default if profile == "Hardware" else sim_default
            self.topic_fields[key].setText(self.settings.value(f"{profile}/{key}", default))
        self.encoder_source.setCurrentText(
            self.settings.value(f"{profile}/encoder_source", "Joint states" if profile == "Simulation" else "Ticks and wheel speeds")
        )
        self.wheel_radius.setValue(float(self.settings.value(f"{profile}/wheel_radius", DEFAULT_WHEEL_RADIUS_M)))
        self.setup_status.setText(f"{profile} profile loaded. Reconnect ROS 2 to apply edits.")

    def restore_profile_defaults(self):
        if self.gazebo_teleop:
            self.configure_gazebo_teleop()
            return
        profile = self.profile.currentText()
        for key, _title, sim_default, _kind, hw_default in TOPIC_DEFS:
            default = hw_default if profile == "Hardware" else sim_default
            self.topic_fields[key].setText(default)
        self.wheel_radius.setValue(DEFAULT_WHEEL_RADIUS_M)
        self.setup_status.setText(f"Restored {profile} default topics (click Save to persist).")

    def save_topics(self):
        if self.gazebo_teleop:
            self.setup_status.setText("Standalone Gazebo settings are session-only; saved profiles are unchanged.")
            return
        profile = self.profile.currentText()
        for key, field in self.topic_fields.items():
            topic = field.text().strip()
            if any(c.isspace() for c in topic):
                self.setup_status.setText("Topic names cannot contain spaces.")
                field.setFocus()
                return
        self.settings.setValue("profile", profile)
        self.settings.setValue(f"{profile}/encoder_source", self.encoder_source.currentText())
        self.settings.setValue(f"{profile}/wheel_radius", self.wheel_radius.value())
        for key, field in self.topic_fields.items():
            self.settings.setValue(f"{profile}/{key}", field.text().strip())
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.setup_status.setText("Could not save settings locally.")
            return
        self.setup_status.setText(f"{profile} topic settings saved. Reconnect ROS 2 to apply.")

    # ---------------- Clean Lifecycle ----------------

    def showEvent(self, event):
        super().showEvent(event)
        if not self.freshness_timer.isActive():
            self.freshness_timer.start(250)
        if not self.preview_timer.isActive():
            self.preview_timer.start(20)

    def hideEvent(self, event):
        if self.safety.can_publish_ros() and self.ros_link is not None:
            self.ros_link.publish_stop_burst()
            self.safety.set_armed(False)
            self._sync_safety_ui()
        self.clear_inputs()
        self.freshness_timer.stop()
        self.preview_timer.stop()
        super().hideEvent(event)

    def final_shutdown(self):
        """Force full cleanup of timers, ROS, camera, recording, and VTK when exiting app."""
        self._closing_app = True
        self.freshness_timer.stop()
        self.preview_timer.stop()
        self.clear_inputs()
        self.stop_recording()
        if self.ros_link is not None and self.ros_link.isRunning():
            self.ready = False
            self.safety.on_ros_connected(False)
            self.ros_link.publish_stop_burst()
            self.ros_link.requestInterruption()
            self.ros_link.wait(1500)
        self.camera.shutdown(timeout_ms=1000)
        if self.motor_window is not None:
            self.motor_window.close()
        self.stl_viewer.shutdown()

    def closeEvent(self, event):
        if self.ros_link is not None and self.ros_link.isRunning():
            self.ready = False
            self.safety.on_ros_connected(False)
            self.ros_link.publish_stop_burst()
            self.ros_link.requestInterruption()
            if not self.ros_link.wait(1500):
                self.log_event("WARN", "ROS thread still stopping; try closing again shortly.")
                event.ignore()
                return

        if self.stl_viewer.loader and self.stl_viewer.loader.isRunning():
            self.stl_viewer.info.setText("Wait for STL model load to finish before closing.")
            event.ignore()
            return

        if not self.camera.shutdown(timeout_ms=800):
            self._cam_close_attempts += 1
            if self._cam_close_attempts < 6:
                self.log_event("INFO", "Waiting for camera capture to stop…")
                QTimer.singleShot(250, self.close)
                event.ignore()
                return
            self.log_event("WARN", "Camera thread slow to stop; detaching window.")
        self._cam_close_attempts = 0

        self.safety.set_armed(False)
        self.safety.set_preview_enabled(False)
        self.clear_inputs()
        self._sync_safety_ui()
        self.motor_window.hide()
        self.stop_recording()
        self.closed.emit()
        super().closeEvent(event)

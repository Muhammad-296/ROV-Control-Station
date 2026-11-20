"""Offline Rover dashboard and topic setup; no motion commands are transmitted."""

from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QFormLayout, QGroupBox, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QPushButton, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)


# Defaults follow the current Autonmous_Ws simulation and SLAM contracts.
TOPICS = (
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
        self.setWindowTitle("MESEKET · Mars Rover")
        self.resize(1050, 720)
        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 20)
        title = QLabel("Mars Rover Control Station")
        title.setObjectName("title")
        layout.addWidget(title)
        status = message("OFFLINE · No rover connected · Motion controls unavailable")
        status.setObjectName("status")
        layout.addWidget(status)

        tabs = QTabWidget()
        tabs.addTab(self.navigation_page(), "Drive and navigation")
        tabs.addTab(self.camera_page(), "Cameras and perception")
        tabs.addTab(self.health_page(), "Rover health")
        tabs.addTab(self.setup_page(), "ROS topic setup")
        layout.addWidget(tabs)
        back = QPushButton("Back to vehicle selection")
        back.clicked.connect(self.close)
        layout.addWidget(back)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Offline setup only. No ROS or serial commands are sent.")

    def navigation_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message(
            "Current rover: four-wheel skid-steer. Driving uses forward speed and "
            "turning rate; navigation uses the existing SLAM and Nav2 stack."
        ))
        map_box = QGroupBox("Map and route")
        map_layout = QVBoxLayout(map_box)
        map_layout.addWidget(message(
            "Waiting for a ROS connection. The live map, rover pose and planned "
            "route will appear here. No simulated position is displayed."
        ))
        layout.addWidget(map_box, 1)
        for text in ("Manual driving — backend required", "Navigate to goal — backend required",
                     "Stop / cancel navigation — backend required"):
            button = QPushButton(text)
            button.setEnabled(False)
            button.setToolTip("Requires rover communication and manual/autonomous command arbitration.")
            layout.addWidget(button)
        layout.addWidget(message(
            "Manual/autonomous handover and controller stop acknowledgement must "
            "be connected before enabling motion."
        ))
        return page

    def camera_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        for title, text in (
            ("RGB and depth", "No camera frames received. Topic names are configurable in ROS topic setup."),
            ("Perception", "No marker, obstacle or terrain data received. These displays will use "
             "the existing perception interfaces once the backend is connected."),
        ):
            box = QGroupBox(title)
            QVBoxLayout(box).addWidget(message(text))
            layout.addWidget(box)
        layout.addStretch()
        return page

    def health_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        table = QTableWidget(6, 2)
        table.setHorizontalHeaderLabels(["Signal", "Status"])
        for row, name in enumerate(("SLAM / localization", "Nav2 navigation", "IMU",
                                    "Wheel odometry", "Wheel slip", "Camera / perception")):
            table.setItem(row, 0, QTableWidgetItem(name))
            table.setItem(row, 1, QTableWidgetItem("Unknown — disconnected"))
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(table)
        layout.addWidget(message(
            "Battery, arm and science telemetry have no connection in this version. "
            "Health is unknown until real messages and freshness checks are available."
        ))
        return page

    def setup_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(message(
            "Save topic names for the future ROS connection. Saving does not connect "
            "to a rover. This screen runs on Windows and Ubuntu without ROS installed."
        ))
        form = QFormLayout()
        self.profile = QComboBox()
        self.profile.addItems(["Simulation", "Hardware"])
        form.addRow("Environment", self.profile)
        layout.addLayout(form)
        self.topic_table = QTableWidget(len(TOPICS), 3)
        self.topic_table.setHorizontalHeaderLabels(["Signal", "ROS topic", "Message type"])
        self.topic_fields = {}
        for row, (key, title, default, kind) in enumerate(TOPICS):
            for column, value in ((0, title), (2, kind)):
                item = QTableWidgetItem(value)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.topic_table.setItem(row, column, item)
            field = QLineEdit(default)
            self.topic_fields[key] = field
            self.topic_table.setCellWidget(row, 1, field)
        self.topic_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.topic_table)
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
            self.topic_fields[key].setText(self.settings.value(f"{profile}/{key}", default))
        self.setup_status.setText("Offline · Unsaved edits are discarded when changing environment.")

    def save_topics(self):
        profile = self.profile.currentText()
        for key, field in self.topic_fields.items():
            topic = field.text().strip()
            if not topic or any(character.isspace() for character in topic):
                self.setup_status.setText("Enter non-empty topic names without whitespace before saving.")
                field.setFocus()
                return
        self.settings.setValue("profile", profile)
        for key, field in self.topic_fields.items():
            self.settings.setValue(f"{profile}/{key}", field.text().strip())
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.setup_status.setText("Could not save settings. Check local settings storage permissions.")
            return
        self.setup_status.setText(f"{profile} topic settings saved locally. Still offline.")

    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)

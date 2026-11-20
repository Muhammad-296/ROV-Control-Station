"""Unit and offscreen Qt integration tests for the Mars Rover Control Station."""

from types import SimpleNamespace
import csv
import math
import os
from pathlib import Path
import sys
import tempfile

from rover_kinematics import (
    DEFAULT_WHEEL_RADIUS_M,
    MODE_COORDINATED,
    MODE_CRAB,
    MODE_TURN_IN_PLACE,
    OUTPUT_PREVIEW,
    OUTPUT_ROS_TELEOP,
    SAFETY_ARMED,
    SAFETY_DISARMED,
    SAFETY_ESTOP,
    SAFETY_PREVIEW,
    SafetyController,
    WHEELS,
    compute_cmd_vel,
    compute_wheel_kinematics,
    shape_axis,
)


def run_unit_tests():
    # 1. Input shaping
    assert shape_axis(0.05, 8.0, 30.0) == 0.0
    assert shape_axis(1.0, 8.0, 30.0) == 100.0
    assert shape_axis(-1.0, 8.0, 30.0) == -100.0
    assert shape_axis(1.0, 8.0, 30.0, invert=True) == -100.0
    assert shape_axis(0.5, 100.0, 0.0) == 0.0

    # 2. 4WS kinematics
    drv, st, rad = compute_wheel_kinematics(50.0, 100.0, 0.0, MODE_COORDINATED, 30.0, 100.0)
    assert st["FL"] == 30.0 and st["FR"] == 30.0
    assert st["RL"] == -30.0 and st["RR"] == -30.0
    assert rad is not None and rad > 0.0

    # Turning right (turn > 0) in coordinated mode drives left faster than right.
    drv, _, _ = compute_wheel_kinematics(50.0, 0.0, 40.0, MODE_COORDINATED, 30.0, 100.0)
    assert drv["FL"] > drv["FR"] and drv["RL"] > drv["RR"]

    # Crab mode
    drv, st, rad = compute_wheel_kinematics(40.0, -50.0, 100.0, MODE_CRAB, 30.0, 100.0)
    assert all(st[w] == -15.0 for w in WHEELS)
    assert all(drv[w] == 40.0 for w in WHEELS)
    assert rad is None

    # Turn in place: tangent angles (45, -45, -45, 45); right turn drives L+, R-.
    drv, st, rad = compute_wheel_kinematics(0.0, 0.0, 60.0, MODE_TURN_IN_PLACE, 30.0, 50.0)
    assert st == {"FL": 45.0, "FR": -45.0, "RL": -45.0, "RR": 45.0}
    assert drv["FL"] > 0 and drv["RL"] > 0
    assert drv["FR"] < 0 and drv["RR"] < 0
    assert rad == 0.0

    # 3. compute_cmd_vel REP-103 sign & clamping
    vx, vy, wz = compute_cmd_vel(100.0, 0.0, 100.0, MODE_COORDINATED, 0.5, 1.0, 100.0)
    assert math.isclose(vx, 0.5) and vy == 0.0 and wz < 0.0
    vx, vy, wz = compute_cmd_vel(-100.0, 100.0, 0.0, MODE_COORDINATED, 0.5, 1.0, 100.0)
    assert vx < 0.0 and wz > 0.0  # Reversing with front wheels right turns CCW (+wz)
    vx, vy, wz = compute_cmd_vel(100.0, 0.0, 100.0, MODE_TURN_IN_PLACE, 0.5, 1.0, 50.0)
    assert vx == 0.0 and math.isclose(wz, -0.5)

    # 4. SafetyController invariants
    sc = SafetyController()
    assert sc.state() == SAFETY_PREVIEW and not sc.can_publish_ros()
    assert not sc.set_armed(True)
    sc.set_output_mode(OUTPUT_ROS_TELEOP)
    assert sc.state() == SAFETY_DISARMED and not sc.set_armed(True)  # Not connected
    sc.on_ros_connected(True)
    assert sc.set_armed(True) and sc.can_publish_ros() and sc.state() == SAFETY_ARMED
    sc.trigger_estop()
    assert sc.state() == SAFETY_ESTOP and not sc.can_publish_ros()
    assert not sc.set_armed(True)  # Blocked while latched
    sc.reset_latch()
    assert sc.state() == SAFETY_DISARMED and not sc.can_publish_ros()
    assert sc.set_armed(True)
    sc.on_ros_connected(False)
    assert not sc.can_publish_ros()
    sc.on_ros_connected(True)
    assert sc.set_armed(True)
    sc.set_output_mode(OUTPUT_PREVIEW)
    assert not sc.can_publish_ros() and sc.state() == SAFETY_PREVIEW
    print("PASS: Rover kinematics and SafetyController unit tests")


def _stamp(sec: int, nsec: int = 0):
    return SimpleNamespace(sec=sec, nanosec=nsec)


def _header(sec: int = 1, nsec: int = 0, frame_id: str = "map"):
    return SimpleNamespace(stamp=_stamp(sec, nsec), frame_id=frame_id)


def run_ros_parser_tests():
    import numpy as np
    from rover_ros import (
        parse_detections_3d,
        parse_diagnostics,
        parse_imu,
        parse_joint_states,
        parse_occupancy_grid,
        parse_odometry,
        parse_path,
        parse_pose_stamped,
        parse_single_tick,
        parse_wheel_speeds,
        parse_wheel_ticks,
        ros_image_to_qimage,
    )

    prev = {}
    msg1 = SimpleNamespace(
        header=_header(10, 0),
        name=["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"],
        position=[0.0, 0.0, 0.0, 0.0],
        velocity=[2.0 * math.pi, -2.0 * math.pi, 0.0, 0.0],
    )
    out1 = parse_joint_states(msg1, prev)
    assert len(out1) == 4 and math.isclose(out1[0][2], 60.0, rel_tol=1e-4)
    # Repeat identical timestamp with valid velocity MUST still emit RPM.
    out_rep = parse_joint_states(msg1, prev)
    assert len(out_rep) == 4 and math.isclose(out_rep[1][2], -60.0, rel_tol=1e-4)

    # Position differentiation fallback when velocity is empty
    msg2 = SimpleNamespace(
        header=_header(11, 0),
        name=["left_front_wheel_joint"],
        position=[2.0 * math.pi],
        velocity=[],
    )
    out2 = parse_joint_states(msg2, prev)
    assert len(out2) == 1 and math.isclose(out2[0][2], 60.0, rel_tol=1e-4)

    # Wheel ticks (4 and 6 element) & individual tick
    t4 = parse_wheel_ticks(SimpleNamespace(data=[10, 20, 30, 40]))
    assert t4 == [("D_FL", 10, None), ("D_FR", 20, None), ("D_RL", 30, None), ("D_RR", 40, None)]
    t6 = parse_wheel_ticks(SimpleNamespace(data=[1, 2, 3, 4, 5, 6]))
    assert t6 == [("D_FL", 1, None), ("D_FR", 4, None), ("D_RL", 3, None), ("D_RR", 6, None)]
    assert parse_single_tick("RL", SimpleNamespace(data=77)) == ("D_RL", 77, None)

    # Wheel speeds
    sp = parse_wheel_speeds(SimpleNamespace(data=[2 * math.pi * DEFAULT_WHEEL_RADIUS_M, 0.0, 0.0, 0.0]), DEFAULT_WHEEL_RADIUS_M)
    assert len(sp) == 4 and math.isclose(sp[0][2], 60.0, rel_tol=1e-4)

    # IMU with orientation missing (-1) must still preserve gyro & accel
    imu_msg = SimpleNamespace(
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[-1.0] + [0.0] * 8,
        angular_velocity=SimpleNamespace(x=math.pi, y=0.0, z=0.0),
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.80665),
        linear_acceleration_covariance=[0.0] * 9,
    )
    imu_parsed = parse_imu(imu_msg)
    assert imu_parsed is not None and "roll" not in imu_parsed
    assert math.isclose(imu_parsed["gx"], 180.0, rel_tol=1e-4)
    assert math.isclose(imu_parsed["az"], 1.0, rel_tol=1e-4)

    # Odometry
    odom_msg = SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=-2.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)),
        )),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.3, y=0.4, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.2),
        )),
    )
    odom = parse_odometry(odom_msg)
    assert odom and math.isclose(odom["yaw_deg"], 90.0, abs_tol=1e-3) and math.isclose(odom["speed_mps"], 0.5, rel_tol=1e-4)

    # OccupancyGrid
    og_msg = SimpleNamespace(
        info=SimpleNamespace(
            width=4,
            height=4,
            resolution=0.1,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-1.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[-1, 0, 50, 100] * 4,
    )
    grid = parse_occupancy_grid(og_msg)
    assert grid and grid["image"].width() == 4 and grid["image"].height() == 4

    # Path downsampling + PoseStamped + Detection3DArray + Diagnostics
    big_path = SimpleNamespace(poses=[
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=float(i), y=0.0, z=0.0)))
        for i in range(2500)
    ])
    pts = parse_path(big_path)
    assert len(pts) == 2000 and pts[0] == (0.0, 0.0) and pts[-1] == (2499.0, 0.0)

    ps = parse_pose_stamped(SimpleNamespace(
        header=_header(frame_id="map"),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=3.0, y=4.0, z=0.5),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    ))
    assert ps and ps["x"] == 3.0 and ps["frame_id"] == "map"

    dets = parse_detections_3d(SimpleNamespace(
        header=_header(frame_id="base_link"),
        detections=[SimpleNamespace(bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=2.0, y=0.5, z=0.0)),
            size=SimpleNamespace(x=0.4, y=0.4, z=0.5),
        ))],
    ))
    assert len(dets) == 1 and dets[0]["frame_id"] == "base_link"

    diags = parse_diagnostics(SimpleNamespace(status=[
        SimpleNamespace(name="encoder", level=b"\x01", message="Slow ticks", values=[SimpleNamespace(key="hz", value="10")]),
        SimpleNamespace(name="slip", level=2, message="Slip high", values=[]),
    ]))
    assert diags[0]["level"] == 1 and diags[1]["level"] == 2 and diags[0]["values"]["hz"] == "10"

    # Images
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    q_rgb = ros_image_to_qimage(SimpleNamespace(height=12, width=16, step=48, encoding="rgb8", is_bigendian=0, data=rgb.tobytes()))
    assert q_rgb and q_rgb.width() == 16 and q_rgb.height() == 12
    q_bgr = ros_image_to_qimage(SimpleNamespace(height=12, width=16, step=48, encoding="bgr8", is_bigendian=0, data=rgb.tobytes()))
    assert q_bgr and q_bgr.pixelColor(0, 0).blue() == 255

    d16 = np.full((8, 8), 1500, dtype="<u2")
    q_d16 = ros_image_to_qimage(SimpleNamespace(height=8, width=8, step=16, encoding="16UC1", is_bigendian=0, data=d16.tobytes()), is_depth=True)
    assert q_d16 and q_d16.width() == 8

    d32 = np.full((8, 8), 2.5, dtype="<f4")
    d32[0, 0] = float("nan")
    q_d32 = ros_image_to_qimage(SimpleNamespace(height=8, width=8, step=32, encoding="32FC1", is_bigendian=0, data=d32.tobytes()), is_depth=True)
    assert q_d32 and q_d32.width() == 8
    assert ros_image_to_qimage(SimpleNamespace(height=8, width=8, step=8, encoding="rgb8", is_bigendian=0, data=b"short")) is None
    print("PASS: All ROS 2 telemetry, OccupancyGrid, diagnostics, and NumPy Image parser tests")


def run_qt_integration_tests():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QEvent, Qt
    from PySide6.QtGui import QImage, QKeyEvent
    from PySide6.QtWidgets import QApplication
    import numpy as np

    from control_station import VehicleSelector
    from rover_camera import rtsp_credentials_error

    app = QApplication.instance() or QApplication(sys.argv)
    selector = VehicleSelector()
    selector.show()
    app.processEvents()

    selector.open_rover()
    dash = selector.rover
    assert dash is not None
    app.processEvents()

    # Cycle all center tabs and confirm NO QGraphicsOpacityEffect remains on STLViewer
    for idx in range(dash.center_tabs.count()):
        dash.center_tabs.setCurrentIndex(idx)
        app.processEvents()
        assert dash.mode.currentIndex() == idx
    assert dash.stl_viewer.graphicsEffect() is None

    # Cycle all settings tabs
    for idx in range(dash.settings_tabs.count()):
        dash.settings_tabs.setCurrentIndex(idx)
        app.processEvents()

    dash.center_tabs.setCurrentIndex(0)
    app.processEvents()

    # Input preview + keyboard + Space stop
    dash.preview_button.setChecked(True)
    app.processEvents()
    app.sendEvent(dash, QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_W, Qt.KeyboardModifier.NoModifier, "w"))
    app.sendEvent(dash, QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_D, Qt.KeyboardModifier.NoModifier, "d"))
    dash.input_tick()
    app.processEvents()
    assert dash.rover_view.command_active
    assert dash.rover_view.steer_commands["FL"] > 0
    app.sendEvent(dash, QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Space, Qt.KeyboardModifier.NoModifier, " "))
    app.processEvents()
    assert not dash.safety.preview_enabled

    # Map panel + synthetic SLAM/Nav2/Perception + JSON save/load
    dash.ready = True
    dash.receive_map({
        "width": 20,
        "height": 20,
        "resolution": 0.2,
        "origin_x": -2.0,
        "origin_y": -2.0,
        "origin_yaw": 0.0,
        "image": QImage(20, 20, QImage.Format.Format_RGB888),
    })
    dash.receive_odometry({"x": 0.4, "y": 0.2, "yaw_deg": 15.0, "vx": 0.3, "vy": 0.0, "wz": 0.0, "speed_mps": 0.3})
    dash.map_panel.set_path([(0.4, 0.2), (1.0, 0.8), (1.8, 1.2)])
    dash.receive_aruco({"x": 1.2, "y": 0.5, "z": 0.2, "yaw_deg": 0.0, "frame_id": "map"})
    dash.receive_obstacles([{"x": 1.0, "y": -0.3, "z": 0.1, "sx": 0.4, "sy": 0.4, "sz": 0.4, "frame_id": "base_link"}])
    dash.map_panel.add_waypoint(1.5, 1.0, 30.0)
    assert len(dash.map_panel.waypoints) == 1

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        wp_file = tmp / "mission.json"
        dash.map_panel.save_waypoints_file(str(wp_file))
        dash.map_panel.clear_waypoints()
        assert dash.map_panel.load_waypoints_file(str(wp_file)) == 1

        # Motor test station + guided inspection CSV
        dash.open_motor_window()
        app.processEvents()
        dash.preview_button.setChecked(True)
        dash.hold_motor(0, 1)
        dash.input_tick()
        dash.release_motor(0)
        dash.mark_guided_step("PASS")
        dash.mark_guided_step("FAIL")
        insp_csv = tmp / "inspection.csv"
        dash.export_inspection_to(str(insp_csv))
        assert "PASS" in insp_csv.read_text(encoding="utf-8")
        dash.motor_window.hide()

        # Camera validation + ROS frame + snapshot
        assert rtsp_credentials_error("", "") == ""
        assert rtsp_credentials_error("admin", "") != ""
        dash.camera.set_ros_connected(True)
        dash.camera.source_combo.setCurrentIndex(0)  # ROS RGB
        dash.camera.start()
        rgb = np.full((16, 16, 3), 120, dtype=np.uint8)
        qimg = QImage(rgb.data, 16, 16, 48, QImage.Format.Format_RGB888).copy()
        dash.camera.receive_ros_frame("rgb", qimg)
        snap = tmp / "snap.png"
        assert dash.camera.save_snapshot_to(str(snap)) and snap.exists()
        dash.camera.open_window()
        dash.camera.toggle_fullscreen()
        dash.camera.toggle_fullscreen()
        dash.camera.window.close()
        dash.camera.stop()

        # Sensors, IMU, Zero Yaw, Encoders, Unified CSV recording
        dash.receive_imu({"roll": 2.5, "pitch": -1.5, "yaw": 45.0, "gx": 0.5, "gy": -0.2, "gz": 1.0, "ax": 0.0, "ay": 0.0, "az": 1.0})
        dash.zero_yaw()
        dash.receive_imu({"roll": 2.5, "pitch": -1.5, "yaw": 45.0})
        assert abs(dash.imu["yaw"]) < 1e-6

        dash.update_encoder("D_FL", 120, 42.0)
        dash.update_encoder("D_FR", 125, 43.0)
        dash.receive_slip(True, False)
        dash.receive_diagnostics("Encoder", [{"name": "ticks", "level": 0, "message": "OK", "values": {}}])
        dash.receive_topic_rates({"imu": {"hz": 50.0, "last_seen": 1e12}})
        dash.refresh_readings()

        tel_csv = tmp / "telemetry.csv"
        dash.start_recording_to(str(tel_csv))
        dash._last_csv_write = 0.0
        dash.write_telemetry_row()
        dash.stop_recording()
        rows = list(csv.DictReader(tel_csv.open(encoding="utf-8")))
        assert len(rows) == 1 and rows[0]["fl_ticks"] == "120" and rows[0]["slip_detected"] == "1"

        enc_csv = tmp / "encoders.csv"
        dash.export_encoders_to(str(enc_csv))
        assert "E5,S1,FL" in enc_csv.read_text(encoding="utf-8")

    # Test E-STOP latch + reset
    dash.output_mode_combo.setCurrentIndex(1)
    dash.safety.on_ros_connected(True)
    dash._sync_safety_ui()
    dash.arm_button.setChecked(True)
    assert dash.safety.can_publish_ros()
    dash.emergency_stop()
    assert not dash.safety.can_publish_ros() and dash.safety.estop_latched
    dash.reset_estop_latch()
    assert not dash.safety.estop_latched and not dash.safety.can_publish_ros()

    # Lifecycle close -> reopen -> final_shutdown
    dash.ready = False
    dash.close()
    app.processEvents()
    selector.open_rover()
    app.processEvents()
    selector.close()
    app.processEvents()
    print("PASS: Offscreen Qt integration, Map, Camera, Motor Station, CSV, and lifecycle tests")


if __name__ == "__main__":
    run_unit_tests()
    if "--unit-only" not in sys.argv:
        run_ros_parser_tests()
        run_qt_integration_tests()

"""Optional ROS 2 telemetry, on-demand cameras, diagnostics, and gated teleop/nav output.

ROS 2 (`rclpy` and message packages) are imported lazily inside `RoverRosLink.run()`
so the GUI operates cleanly offline without ROS installed.
"""

import math
import threading
import time
from collections import deque

import numpy as np
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

from rover_kinematics import WHEELS


_WHEEL_ALIASES = (
    ("FL", ("left_front", "front_left")),
    ("FR", ("right_front", "front_right")),
    ("RL", ("left_rear", "rear_left")),
    ("RR", ("right_rear", "rear_right")),
)

_HARDWARE_WHEEL_TOPICS = (
    ("FL", "left_front"),
    ("FR", "right_front"),
    ("RL", "left_rear"),
    ("RR", "right_rear"),
)


def _build_depth_lut() -> np.ndarray:
    """Build a 256x3 uint8 Inferno/Mars-like colormap LUT for depth visualization."""
    stops = np.array([
        [0, 255, 224, 130],   # near (0.2m): bright warm amber
        [64, 243, 118, 105],  # Mars red-coral
        [128, 178, 68, 128],  # plum
        [192, 82, 54, 125],   # deep violet
        [255, 20, 26, 46],    # far (6.0m): dark slate
    ], dtype=np.float32)
    idx = np.arange(256, dtype=np.float32)
    r = np.interp(idx, stops[:, 0], stops[:, 1])
    g = np.interp(idx, stops[:, 0], stops[:, 2])
    b = np.interp(idx, stops[:, 0], stops[:, 3])
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


_DEPTH_LUT = _build_depth_lut()


def is_finite_number(value) -> bool:
    """Return True for finite numeric values (excluding booleans)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def quat_to_euler_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float] | None:
    """Convert quaternion (x, y, z, w) to (roll, pitch, yaw) in degrees."""
    values = (x, y, z, w)
    if not all(is_finite_number(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        return None
    x, y, z, w = (v / norm for v in values)
    roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return roll, pitch, yaw


def parse_joint_states(message, previous_state: dict) -> list[tuple[str, None, float]]:
    """Extract drive wheel RPM readings from a sensor_msgs/JointState-like message.

    Fixes repeated-timestamp drops by always honoring valid velocity measurements
    even when header stamps do not advance.
    """
    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
    if stamp <= 0:
        stamp = time.monotonic()

    results = []
    names = list(message.name)
    positions = list(getattr(message, "position", ()) or ())
    velocities = list(getattr(message, "velocity", ()) or ())

    for wheel, aliases in _WHEEL_ALIASES:
        accepted = tuple(f"{alias}_wheel_joint" for alias in aliases) + aliases
        for index, name in enumerate(names):
            joint = name.rsplit("/", 1)[-1]
            if joint not in accepted:
                continue
            position = positions[index] if index < len(positions) else None
            velocity = velocities[index] if index < len(velocities) else None
            if not is_finite_number(position):
                position = None
            if not is_finite_number(velocity):
                velocity = None

            previous = previous_state.get(wheel)
            if position is not None:
                if previous is None or stamp > previous[0]:
                    if velocity is None and previous is not None:
                        elapsed = stamp - previous[0]
                        if 0.0 < elapsed <= 2.0:
                            velocity = (position - previous[1]) / elapsed
                    previous_state[wheel] = (stamp, position)
                elif stamp < previous[0]:
                    # Clock reset: re-anchor baseline without position differentiation.
                    previous_state[wheel] = (stamp, position)

            if velocity is not None:
                rpm = float(velocity) * 60.0 / (2.0 * math.pi)
                results.append((f"D_{wheel}", None, rpm))
            break
    return results


def parse_wheel_ticks(message) -> list[tuple[str, int, None]]:
    """Extract raw drive tick counts from a 4- or 6-element Int64MultiArray."""
    counts = list(message.data)
    if len(counts) == 6:
        counts = [counts[0], counts[3], counts[2], counts[5]]
    if len(counts) != 4:
        return []
    results = []
    for wheel, ticks in zip(WHEELS, counts):
        if isinstance(ticks, bool):
            continue
        try:
            tick_int = int(ticks)
        except (TypeError, ValueError):
            continue
        results.append((f"D_{wheel}", tick_int, None))
    return results


def parse_single_tick(wheel: str, message) -> tuple[str, int, None] | None:
    """Extract a single wheel tick integer from std_msgs/Int64."""
    if wheel not in WHEELS:
        return None
    ticks = getattr(message, "data", None)
    if isinstance(ticks, bool):
        return None
    try:
        return (f"D_{wheel}", int(ticks), None)
    except (TypeError, ValueError):
        return None


def parse_wheel_speeds(message, wheel_radius: float) -> list[tuple[str, None, float, float]]:
    """Convert 4-wheel linear speeds (m/s) into (motor, None, rpm, speed_mps)."""
    if not is_finite_number(wheel_radius) or wheel_radius <= 0:
        return []
    values = list(message.data)
    if len(values) != 4:
        return []
    results = []
    for wheel, speed in zip(WHEELS, values):
        if not is_finite_number(speed):
            continue
        speed_f = float(speed)
        rpm = speed_f * 60.0 / (2.0 * math.pi * wheel_radius)
        results.append((f"D_{wheel}", None, rpm, speed_f))
    return results


def parse_imu(message) -> dict | None:
    """Extract attitude, angular velocity (deg/s), and linear acceleration (g).

    Orientation, gyro, and accelerometer channels are evaluated independently so a
    missing orientation covariance never discards gyro or acceleration plots.
    """
    sample = {}
    orient_cov = getattr(message, "orientation_covariance", (0,))
    if orient_cov is None or len(orient_cov) == 0 or orient_cov[0] != -1:
        q = message.orientation
        euler = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        if euler is not None:
            sample["roll"], sample["pitch"], sample["yaw"] = euler

    for vector, covariance, names, scale in (
        (
            message.angular_velocity,
            getattr(message, "angular_velocity_covariance", (0,)),
            ("gx", "gy", "gz"),
            180.0 / math.pi,
        ),
        (
            message.linear_acceleration,
            getattr(message, "linear_acceleration_covariance", (0,)),
            ("ax", "ay", "az"),
            1.0 / 9.80665,
        ),
    ):
        if covariance is None or len(covariance) == 0 or covariance[0] != -1:
            components = (vector.x, vector.y, vector.z)
            if all(is_finite_number(v) for v in components):
                for name, value in zip(names, components):
                    sample[name] = float(value) * scale

    return sample or None


def parse_odometry(message) -> dict | None:
    """Extract 2D pose and twist speed from nav_msgs/Odometry."""
    pos = message.pose.pose.position
    ori = message.pose.pose.orientation
    lin = message.twist.twist.linear
    ang = message.twist.twist.angular
    if not all(is_finite_number(v) for v in (pos.x, pos.y, lin.x, lin.y, ang.z)):
        return None
    euler = quat_to_euler_deg(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = euler[2] if euler is not None else 0.0
    vx, vy = float(lin.x), float(lin.y)
    return {
        "x": float(pos.x),
        "y": float(pos.y),
        "yaw_deg": yaw_deg,
        "vx": vx,
        "vy": vy,
        "wz": float(ang.z),
        "speed_mps": math.hypot(vx, vy),
    }


def parse_occupancy_grid(message) -> dict | None:
    """Convert nav_msgs/OccupancyGrid to metadata + vertically flipped Mars-themed QImage."""
    info = message.info
    w = int(info.width)
    h = int(info.height)
    res = float(info.resolution)
    if w <= 0 or h <= 0 or not is_finite_number(res) or res <= 0:
        return None
    data = np.asarray(message.data, dtype=np.int16)
    if data.size != w * h:
        return None
    grid = data.reshape((h, w))

    # Build RGB888 canvas: unknown=-1 (#12161f), free=0 (#222938), occupied=100 (#f37669).
    rgb = np.empty((h, w, 3), dtype=np.uint8)
    unknown = grid < 0
    free = grid == 0
    occ = grid >= 100
    mid = (~unknown) & (~free) & (~occ)

    rgb[unknown] = (18, 22, 31)
    rgb[free] = (34, 41, 56)
    rgb[occ] = (243, 118, 105)
    if np.any(mid):
        t = np.clip(grid[mid].astype(np.float32) / 99.0, 0.0, 1.0)
        rgb[mid, 0] = (46 + t * (215 - 46)).astype(np.uint8)
        rgb[mid, 1] = (38 + t * (94 - 38)).astype(np.uint8)
        rgb[mid, 2] = (54 + t * (90 - 54)).astype(np.uint8)

    # Flip vertically so world +Y is upward in Qt screen coordinates.
    rgb = np.ascontiguousarray(np.flipud(rgb))
    image = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()

    ori = info.origin.orientation
    euler = quat_to_euler_deg(ori.x, ori.y, ori.z, ori.w)
    origin_yaw = math.radians(euler[2]) if euler is not None else 0.0

    return {
        "width": w,
        "height": h,
        "resolution": res,
        "origin_x": float(info.origin.position.x),
        "origin_y": float(info.origin.position.y),
        "origin_yaw": origin_yaw,
        "image": image,
    }


def parse_path(message, max_points: int = 2000) -> list[tuple[float, float]]:
    """Extract (x, y) waypoints from nav_msgs/Path, preserving endpoints."""
    poses = getattr(message, "poses", None) or ()
    pts = []
    for pose_stamped in poses:
        p = pose_stamped.pose.position
        if is_finite_number(p.x) and is_finite_number(p.y):
            pts.append((float(p.x), float(p.y)))
    if len(pts) > max_points > 2:
        step = (len(pts) - 1) / (max_points - 1)
        pts = [pts[int(round(i * step))] for i in range(max_points - 1)] + [pts[-1]]
    return pts


def parse_pose_stamped(message) -> dict | None:
    """Extract pose summary from geometry_msgs/PoseStamped."""
    pos = message.pose.position
    ori = message.pose.orientation
    if not all(is_finite_number(v) for v in (pos.x, pos.y, pos.z)):
        return None
    euler = quat_to_euler_deg(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = euler[2] if euler is not None else 0.0
    frame_id = getattr(getattr(message, "header", None), "frame_id", "") or ""
    return {
        "x": float(pos.x),
        "y": float(pos.y),
        "z": float(pos.z),
        "yaw_deg": yaw_deg,
        "frame_id": str(frame_id),
    }


def parse_detections_3d(message) -> list[dict]:
    """Extract bounding boxes from vision_msgs/Detection3DArray."""
    frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or "")
    detections = []
    for det in getattr(message, "detections", None) or ():
        bbox = getattr(det, "bbox", None)
        if bbox is None:
            continue
        c = bbox.center.position
        s = bbox.size
        if not all(is_finite_number(v) for v in (c.x, c.y, c.z, s.x, s.y, s.z)):
            continue
        detections.append({
            "x": float(c.x),
            "y": float(c.y),
            "z": float(c.z),
            "sx": max(0.05, float(s.x)),
            "sy": max(0.05, float(s.y)),
            "sz": max(0.05, float(s.z)),
            "frame_id": frame_id,
        })
    return detections


def parse_diagnostics(message) -> list[dict]:
    """Extract entries from diagnostic_msgs/DiagnosticArray."""
    entries = []
    for st in getattr(message, "status", None) or ():
        raw_level = getattr(st, "level", 0)
        if isinstance(raw_level, (bytes, bytearray)):
            level = raw_level[0] if raw_level else 0
        else:
            try:
                level = int(raw_level)
            except (TypeError, ValueError):
                level = 3
        values = {}
        for kv in getattr(st, "values", None) or ():
            values[str(kv.key)] = str(kv.value)
        entries.append({
            "name": str(getattr(st, "name", "")),
            "level": level,
            "message": str(getattr(st, "message", "")),
            "values": values,
        })
    return entries


def ros_image_to_qimage(message, is_depth: bool = False) -> QImage | None:
    """Convert sensor_msgs/Image directly with NumPy (no cv_bridge required).

    Always returns an owned QImage via `.copy()`.
    """
    h = int(getattr(message, "height", 0))
    w = int(getattr(message, "width", 0))
    step = int(getattr(message, "step", 0))
    enc = str(getattr(message, "encoding", "")).strip().lower()
    raw = bytes(getattr(message, "data", b""))
    if h <= 0 or w <= 0 or step <= 0 or len(raw) < h * step:
        return None

    big_endian = bool(getattr(message, "is_bigendian", 0))
    buf = np.frombuffer(raw, dtype=np.uint8, count=h * step).reshape((h, step))

    if enc in ("rgb8", "bgr8") and not is_depth:
        if step < w * 3:
            return None
        arr = buf[:, :w * 3].reshape((h, w, 3))
        if enc == "bgr8":
            arr = np.ascontiguousarray(arr[:, :, ::-1])
        else:
            arr = np.ascontiguousarray(arr)
        return QImage(arr.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()

    if enc in ("rgba8", "bgra8") and not is_depth:
        if step < w * 4:
            return None
        arr = buf[:, :w * 4].reshape((h, w, 4))
        if enc == "bgra8":
            arr = np.ascontiguousarray(arr[:, :, [2, 1, 0, 3]])
        else:
            arr = np.ascontiguousarray(arr)
        return QImage(arr.data, w, h, w * 4, QImage.Format.Format_RGBA8888).copy()

    if enc in ("mono8", "8uc1"):
        if step < w:
            return None
        arr = np.ascontiguousarray(buf[:, :w])
        if is_depth:
            rgb = _DEPTH_LUT[arr]
            rgb = np.ascontiguousarray(rgb)
            return QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        return QImage(arr.data, w, h, w, QImage.Format.Format_Grayscale8).copy()

    if enc in ("16uc1", "mono16"):
        if step < w * 2:
            return None
        dtype = ">u2" if big_endian else "<u2"
        depth_mm = buf[:, :w * 2].view(dtype=dtype).reshape((h, w))
        depth_m = depth_mm.astype(np.float32) * 0.001
        valid = depth_m > 0.0
    elif enc == "32fc1":
        if step < w * 4:
            return None
        dtype = ">f4" if big_endian else "<f4"
        depth_m = buf[:, :w * 4].view(dtype=dtype).reshape((h, w))
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
    else:
        return None

    # Colorize metric depth [0.2 m, 6.0 m] -> LUT index, invalid pixels dark slate.
    rgb = np.full((h, w, 3), (13, 17, 24), dtype=np.uint8)
    if np.any(valid):
        norm = np.clip((depth_m[valid] - 0.2) / 5.8, 0.0, 1.0)
        indices = np.clip((norm * 255.0).astype(np.int32), 0, 255)
        rgb[valid] = _DEPTH_LUT[indices]
    rgb = np.ascontiguousarray(rgb)
    return QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()


class RoverRosLink(QThread):
    """Background ROS 2 executor thread with telemetry, diagnostics, and gated commands."""

    status = Signal(bool, str)
    encoders = Signal(str, object, object, object)  # motor, ticks, rpm, speed_mps
    imu = Signal(dict)
    odometry = Signal(dict)
    wheel_odom = Signal(dict)
    slip = Signal(bool, bool)                       # slip_detected, single_wheel_slip
    map_data = Signal(dict)
    path_data = Signal(list)
    aruco_pose = Signal(dict)
    obstacles = Signal(list)
    diagnostics = Signal(str, list)                 # source, entries
    camera_frame = Signal(str, object)              # feed_key, QImage
    topic_rates = Signal(dict)                      # key -> {"hz": float, "last_seen": float}
    discovered_topics = Signal(list)                # [(name, [types...])]

    def __init__(self, topics: dict, source: str, wheel_radius: float, parent=None,
                 use_sim_time: bool = False):
        super().__init__(parent)
        self.topics = {k: str(v).strip() for k, v in topics.items()}
        self.source = source
        self.radius = float(wheel_radius)
        self.use_sim_time = bool(use_sim_time)
        self.previous = {}

        self._lock = threading.Lock()
        self._desired_camera_feed: str | None = None
        self._cmd_active = False
        self._cmd_updated_at = 0.0
        self._cmd_twist = (0.0, 0.0, 0.0)
        self._stop_bursts = 0
        self._pending_goals: deque[tuple[float, float, float, str]] = deque()
        self._scan_requested = False

        self._slip_detected = False
        self._single_slip = False
        self._last_camera_emit = 0.0
        self._rate_buckets: dict[str, deque[float]] = {}

    def set_active_camera_feed(self, feed_key: str | None) -> None:
        """Select which ROS camera stream to subscribe to ('rgb', 'depth', 'aruco_debug', 'pose_debug', or None)."""
        with self._lock:
            self._desired_camera_feed = feed_key

    def set_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float, active: bool) -> None:
        """Update gated /cmd_vel setpoint streamed at ~20 Hz while active."""
        with self._lock:
            self._cmd_twist = (float(linear_x), float(linear_y), float(angular_z))
            self._cmd_active = bool(active)
            self._cmd_updated_at = time.monotonic()

    def publish_stop_burst(self) -> None:
        """Immediately halt active teleop and send three consecutive zero Twist commands."""
        with self._lock:
            self._cmd_active = False
            self._cmd_twist = (0.0, 0.0, 0.0)
            self._stop_bursts = 3

    def publish_goal_pose(self, x: float, y: float, yaw_rad: float, frame_id: str = "map") -> None:
        """Queue a single 2D Nav2 goal PoseStamped."""
        with self._lock:
            self._pending_goals.append((float(x), float(y), float(yaw_rad), str(frame_id or "map")))

    def request_topic_scan(self) -> None:
        """Queue a ROS graph topic/type discovery scan."""
        with self._lock:
            self._scan_requested = True

    def _record_rate(self, key: str) -> None:
        now = time.monotonic()
        bucket = self._rate_buckets.setdefault(key, deque())
        bucket.append(now)
        cutoff = now - 2.0
        while bucket and bucket[0] < cutoff:
            bucket.popleft()

    def _snapshot_rates(self) -> dict:
        now = time.monotonic()
        cutoff = now - 2.0
        out = {}
        for key, bucket in self._rate_buckets.items():
            while bucket and bucket[0] < cutoff:
                bucket.popleft()
            if not bucket:
                continue
            hz = len(bucket) / 2.0
            out[key] = {"hz": hz, "last_seen": bucket[-1]}
        return out

    def run(self):
        context = node = executor = None
        cmd_vel_pub = None
        Twist = None
        ever_published_cmd = False

        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import PoseStamped, Twist as TwistMsg
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
            from sensor_msgs.msg import Image, Imu, JointState
            from std_msgs.msg import Bool, Float64MultiArray, Int64, Int64MultiArray

            Twist = TwistMsg
            try:
                from vision_msgs.msg import Detection3DArray
            except ImportError:
                Detection3DArray = None

            context = Context()
            rclpy.init(args=[], context=context)
            node = rclpy.create_node("meseket_gui_telemetry", context=context)
            node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=self.use_sim_time)])
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            qos_sensor = qos_profile_sensor_data
            qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            qos_map = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            def sub(msg_type, key, callback, qos_profile):
                topic = self.topics.get(key, "")
                if not topic:
                    return None
                return node.create_subscription(msg_type, topic, callback, qos_profile)

            if self.source == "Joint states":
                sub(JointState, "joints", self._on_joints, qos_sensor)
            elif self.source == "Individual tick topics":
                base = (self.topics.get("ticks") or "/wheel/ticks").rstrip("/")
                for wheel, suffix in _HARDWARE_WHEEL_TOPICS:
                    node.create_subscription(
                        Int64,
                        f"{base}/{suffix}",
                        lambda msg, w=wheel: self._on_single_tick(w, msg),
                        qos_reliable,
                    )
                sub(Float64MultiArray, "wheel_speeds", self._on_speeds, qos_reliable)
            else:
                sub(Int64MultiArray, "ticks", self._on_ticks, qos_reliable)
                sub(Float64MultiArray, "wheel_speeds", self._on_speeds, qos_reliable)

            sub(Imu, "imu", self._on_imu, qos_sensor)
            sub(Odometry, "pose", self._on_odom, qos_reliable)
            sub(Odometry, "wheel_odom", self._on_wheel_odom, qos_reliable)
            sub(Bool, "slip", lambda msg: self._on_slip(msg, False), qos_reliable)
            sub(Bool, "single_slip", lambda msg: self._on_slip(msg, True), qos_reliable)
            sub(OccupancyGrid, "map", self._on_map, qos_map)
            sub(Path, "path", self._on_path, qos_reliable)
            sub(PoseStamped, "aruco_pose", self._on_aruco_pose, qos_reliable)
            if Detection3DArray is not None:
                sub(Detection3DArray, "obstacles", self._on_obstacles, qos_reliable)
            sub(DiagnosticArray, "enc_diag", lambda msg: self._on_diagnostics("Encoder", msg), qos_reliable)
            sub(DiagnosticArray, "slip_diag", lambda msg: self._on_diagnostics("Slip", msg), qos_reliable)

            cmd_topic = self.topics.get("cmd_vel", "")
            if cmd_topic:
                cmd_vel_pub = node.create_publisher(Twist, cmd_topic, 10)
            goal_topic = self.topics.get("goal_pose", "")
            goal_pub = node.create_publisher(PoseStamped, goal_topic, 10) if goal_topic else None

            active_cam_feed = None
            cam_sub = None
            last_cmd_pub = 0.0
            last_rate_emit = time.monotonic()

            self.status.emit(True, "ROS 2 listening · waiting for telemetry")

            while context.ok() and not self.isInterruptionRequested():
                executor.spin_once(timeout_sec=0.04)
                now = time.monotonic()

                with self._lock:
                    desired_cam = self._desired_camera_feed
                    cmd_active = self._cmd_active
                    cmd_twist = self._cmd_twist
                    if now - self._cmd_updated_at > 0.3:
                        cmd_twist = (0.0, 0.0, 0.0)
                    stop_bursts = self._stop_bursts
                    if stop_bursts > 0:
                        self._stop_bursts = 0
                    pending_goals = list(self._pending_goals)
                    self._pending_goals.clear()
                    scan_req = self._scan_requested
                    self._scan_requested = False

                # Manage on-demand ROS camera subscription.
                if desired_cam != active_cam_feed:
                    if cam_sub is not None:
                        node.destroy_subscription(cam_sub)
                        cam_sub = None
                    active_cam_feed = desired_cam
                    if active_cam_feed:
                        cam_topic = self.topics.get(active_cam_feed, "")
                        if cam_topic:
                            feed_name = active_cam_feed
                            cam_sub = node.create_subscription(
                                Image,
                                cam_topic,
                                lambda msg, k=feed_name: self._on_image(k, msg),
                                qos_sensor,
                            )

                # Handle stop bursts & 20 Hz /cmd_vel stream.
                if cmd_vel_pub is not None:
                    if stop_bursts > 0:
                        zero = Twist()
                        for _ in range(stop_bursts):
                            cmd_vel_pub.publish(zero)
                        ever_published_cmd = True
                    elif cmd_active and now - last_cmd_pub >= 0.05:
                        msg = Twist()
                        msg.linear.x = cmd_twist[0]
                        msg.linear.y = cmd_twist[1]
                        msg.angular.z = cmd_twist[2]
                        cmd_vel_pub.publish(msg)
                        last_cmd_pub = now
                        ever_published_cmd = True

                # Publish queued 2D Nav2 goal poses.
                if goal_pub is not None and pending_goals:
                    for gx, gy, gyaw, frame_id in pending_goals:
                        goal = PoseStamped()
                        goal.header.stamp = node.get_clock().now().to_msg()
                        goal.header.frame_id = frame_id
                        goal.pose.position.x = gx
                        goal.pose.position.y = gy
                        goal.pose.orientation.z = math.sin(gyaw * 0.5)
                        goal.pose.orientation.w = math.cos(gyaw * 0.5)
                        goal_pub.publish(goal)

                if scan_req:
                    self.discovered_topics.emit(sorted(node.get_topic_names_and_types()))

                if now - last_rate_emit >= 1.0:
                    last_rate_emit = now
                    self.topic_rates.emit(self._snapshot_rates())

            if cam_sub is not None:
                node.destroy_subscription(cam_sub)
                cam_sub = None

        except Exception as error:
            self.status.emit(False, f"ROS 2 unavailable: {error}")
        finally:
            if ever_published_cmd and cmd_vel_pub is not None and Twist is not None:
                try:
                    cmd_vel_pub.publish(Twist())
                except Exception:
                    pass
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    def _on_joints(self, message):
        self._record_rate("joints")
        for motor, ticks, rpm in parse_joint_states(message, self.previous):
            self.encoders.emit(motor, ticks, rpm, None)

    def _on_ticks(self, message):
        self._record_rate("ticks")
        for motor, ticks, rpm in parse_wheel_ticks(message):
            self.encoders.emit(motor, ticks, rpm, None)

    def _on_single_tick(self, wheel: str, message):
        self._record_rate("ticks")
        parsed = parse_single_tick(wheel, message)
        if parsed is not None:
            self.encoders.emit(parsed[0], parsed[1], None, None)

    def _on_speeds(self, message):
        self._record_rate("wheel_speeds")
        for motor, ticks, rpm, speed_mps in parse_wheel_speeds(message, self.radius):
            self.encoders.emit(motor, ticks, rpm, speed_mps)

    def _on_imu(self, message):
        self._record_rate("imu")
        sample = parse_imu(message)
        if sample is not None:
            self.imu.emit(sample)

    def _on_odom(self, message):
        self._record_rate("pose")
        odom = parse_odometry(message)
        if odom is not None:
            self.odometry.emit(odom)

    def _on_wheel_odom(self, message):
        self._record_rate("wheel_odom")
        odom = parse_odometry(message)
        if odom is not None:
            self.wheel_odom.emit(odom)

    def _on_slip(self, message, is_single: bool):
        self._record_rate("single_slip" if is_single else "slip")
        flag = bool(getattr(message, "data", False))
        if is_single:
            self._single_slip = flag
        else:
            self._slip_detected = flag
        self.slip.emit(self._slip_detected, self._single_slip)

    def _on_map(self, message):
        self._record_rate("map")
        parsed = parse_occupancy_grid(message)
        if parsed is not None:
            self.map_data.emit(parsed)

    def _on_path(self, message):
        self._record_rate("path")
        self.path_data.emit(parse_path(message))

    def _on_aruco_pose(self, message):
        self._record_rate("aruco_pose")
        parsed = parse_pose_stamped(message)
        if parsed is not None:
            self.aruco_pose.emit(parsed)

    def _on_obstacles(self, message):
        self._record_rate("obstacles")
        self.obstacles.emit(parse_detections_3d(message))

    def _on_diagnostics(self, source: str, message):
        self._record_rate("enc_diag" if source == "Encoder" else "slip_diag")
        entries = parse_diagnostics(message)
        if entries:
            self.diagnostics.emit(source, entries)

    def _on_image(self, feed_key: str, message):
        self._record_rate(feed_key)
        now = time.monotonic()
        if now - self._last_camera_emit < 0.066:
            return
        qimg = ros_image_to_qimage(message, is_depth=(feed_key == "depth"))
        if qimg is not None:
            self._last_camera_emit = now
            self.camera_frame.emit(feed_key, qimg)

"""Optional, read-only ROS 2 telemetry. ROS is imported only on connection."""

import math
import time

from PySide6.QtCore import QThread, Signal


class RoverRosLink(QThread):
    status = Signal(bool, str)
    encoders = Signal(str, object, object)
    imu = Signal(dict)

    def __init__(self, topics, source, wheel_radius, parent=None):
        super().__init__(parent)
        self.topics = dict(topics)
        self.source = source
        self.radius = wheel_radius
        self.previous = {}

    def run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Imu, JointState
            from std_msgs.msg import Float64MultiArray, Int64MultiArray

            context = Context()
            rclpy.init(args=[], context=context)
            node = rclpy.create_node("meseket_gui_telemetry", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = qos_profile_sensor_data
            if self.source == "Joint states":
                node.create_subscription(JointState, self.topics["joints"], self.on_joints, qos)
            else:
                node.create_subscription(Int64MultiArray, self.topics["ticks"], self.on_ticks, qos)
                node.create_subscription(Float64MultiArray, self.topics["wheel_speeds"], self.on_speeds, qos)
            node.create_subscription(Imu, self.topics["imu"], self.on_imu, qos)
            self.status.emit(True, "ROS 2 listening · waiting for telemetry")
            while context.ok() and not self.isInterruptionRequested():
                executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self.status.emit(False, f"ROS 2 unavailable: {error}")
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    @staticmethod
    def number(value):
        return isinstance(value, (int, float)) and math.isfinite(value)

    def on_joints(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp <= 0:
            stamp = time.monotonic()
        for wheel, aliases in (
            ("FL", ("left_front", "front_left")), ("FR", ("right_front", "front_right")),
            ("RL", ("left_rear", "rear_left")), ("RR", ("right_rear", "rear_right")),
        ):
            for index, name in enumerate(message.name):
                joint = name.rsplit("/", 1)[-1]
                if joint not in tuple(f"{alias}_wheel_joint" for alias in aliases) + aliases:
                    continue
                position = message.position[index] if index < len(message.position) else None
                velocity = message.velocity[index] if index < len(message.velocity) else None
                if not self.number(position):
                    position = None
                if not self.number(velocity):
                    velocity = None
                previous = self.previous.get(wheel)
                if position is not None:
                    if previous and stamp <= previous[0]:
                        # Clock reset or repeated message: establish a new baseline.
                        self.previous[wheel] = (stamp, position)
                        continue
                    if velocity is None and previous:
                        elapsed = stamp - previous[0]
                        if 0 < elapsed <= 2:
                            velocity = (position - previous[1]) / elapsed
                    self.previous[wheel] = (stamp, position)
                if velocity is not None:
                    # Joint positions are radians, not measured raw encoder ticks.
                    self.encoders.emit(f"D_{wheel}", None, velocity * 60 / (2 * math.pi))
                break

    def on_ticks(self, message):
        counts = list(message.data)
        if len(counts) == 6:
            counts = [counts[0], counts[3], counts[2], counts[5]]
        if len(counts) != 4:
            return
        for wheel, ticks in zip(("FL", "FR", "RL", "RR"), counts):
            # Raw tick updates do not make an old speed reading fresh.
            self.encoders.emit(f"D_{wheel}", ticks, None)

    def on_speeds(self, message):
        values = list(message.data)
        if len(values) != 4:
            return
        for wheel, speed in zip(("FL", "FR", "RL", "RR"), values):
            if not self.number(speed):
                continue
            self.encoders.emit(f"D_{wheel}", None, speed * 60 / (2 * math.pi * self.radius))

    def on_imu(self, message):
        if message.orientation_covariance[0] == -1:
            return
        q = message.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(self.number(value) for value in values):
            return
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            return
        x, y, z, w = (value / norm for value in values)
        sample = {
            "roll": math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))),
            "pitch": math.degrees(math.asin(max(-1, min(1, 2 * (w*y - z*x))))),
            "yaw": math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))),
        }
        for vector, covariance, names, scale in (
            (message.angular_velocity, message.angular_velocity_covariance, ("gx", "gy", "gz"), 180 / math.pi),
            (message.linear_acceleration, message.linear_acceleration_covariance, ("ax", "ay", "az"), 1 / 9.80665),
        ):
            if covariance[0] != -1:
                components = (vector.x, vector.y, vector.z)
                if all(self.number(value) for value in components):
                    sample.update({name: value * scale for name, value in zip(names, components)})
        self.imu.emit(sample)

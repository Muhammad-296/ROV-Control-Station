"""Pure-Python Mars Rover 4WD/4WS kinematics, input shaping, and safety interlocks."""

import math


WHEELS = ("FL", "FR", "RL", "RR")
DRIVE_MOTORS = ("D_FL", "D_FR", "D_RL", "D_RR")
STEER_MOTORS = ("S_FL", "S_FR", "S_RL", "S_RR")

DEFAULT_WHEEL_RADIUS_M = 0.06
DEFAULT_TRACK_WIDTH_M = 0.49
DEFAULT_WHEELBASE_M = 0.56

MODE_COORDINATED = 0
MODE_CRAB = 1
MODE_TURN_IN_PLACE = 2

OUTPUT_PREVIEW = "preview"
OUTPUT_ROS_TELEOP = "ros_teleop"

SAFETY_PREVIEW = "preview"
SAFETY_DISARMED = "disarmed"
SAFETY_ARMED = "armed"
SAFETY_ESTOP = "estop"


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp a numeric value to [lo, hi]."""
    return max(lo, min(hi, float(value)))


def shape_axis(
    raw: float,
    deadzone_pct: float,
    expo_pct: float,
    invert: bool = False,
) -> float:
    """Apply inversion, deadzone, and cubic expo to a normalized axis in [-1, 1].

    Returns a shaped percentage in [-100.0, 100.0].
    """
    value = clamp(raw, -1.0, 1.0)
    if invert:
        value = -value
    dz = clamp(deadzone_pct, 0.0, 100.0) / 100.0
    if dz >= 1.0 or abs(value) <= dz:
        return 0.0
    normalized = math.copysign((abs(value) - dz) / (1.0 - dz), value)
    expo = clamp(expo_pct, 0.0, 100.0) / 100.0
    shaped = (1.0 - expo) * normalized + expo * (normalized ** 3)
    return clamp(shaped * 100.0, -100.0, 100.0)


def compute_wheel_kinematics(
    drive: float,
    steer: float,
    turn: float,
    steering_mode: int,
    steer_limit_deg: float,
    power_pct: float,
) -> tuple[dict[str, float], dict[str, float], float | None]:
    """Compute per-wheel drive percentages, steering angles (deg), and turning radius (m).

    Conventions:
    - drive: [-100, +100], +forward
    - steer: [-100, +100], +right (clockwise in top-down Qt view)
    - turn: [-100, +100], +right (clockwise yaw)
    - steering angles: degrees in Qt top-down view (0 = straight ahead, + = clockwise)
    """
    steer_lim = max(0.0, float(steer_limit_deg))
    angle = clamp(steer, -100.0, 100.0) * steer_lim / 100.0
    power_scale = clamp(power_pct, 0.0, 100.0) / 100.0

    if steering_mode == MODE_CRAB:
        steering = dict.fromkeys(WHEELS, angle)
        wheel_drive = dict.fromkeys(WHEELS, clamp(drive, -100.0, 100.0))
        turning_radius = None
        return wheel_drive, steering, turning_radius

    if steering_mode == MODE_TURN_IN_PLACE:
        if abs(turn) > 1.0:
            # Tangent wheel orientations around chassis center for clockwise/CCW spin.
            steering = {"FL": 45.0, "FR": -45.0, "RL": -45.0, "RR": 45.0}
        else:
            steering = dict.fromkeys(WHEELS, 0.0)
        # Positive turn (clockwise / right) drives left wheels forward (+), right backward (-).
        wheel_drive = {
            wheel: clamp(
                turn * (1.0 if wheel.endswith("L") else -1.0) * power_scale,
                -100.0,
                100.0,
            )
            for wheel in WHEELS
        }
        return wheel_drive, steering, 0.0

    # MODE_COORDINATED (4WS counter-phase Ackermann + differential turn assist).
    steering = {
        "FL": angle,
        "FR": angle,
        "RL": -angle,
        "RR": -angle,
    }
    wheel_drive = {
        wheel: clamp(
            drive + turn * (0.5 if wheel.endswith("L") else -0.5),
            -100.0,
            100.0,
        )
        for wheel in WHEELS
    }
    if abs(angle) > 0.5:
        turning_radius = (DEFAULT_WHEELBASE_M / 2.0) / math.tan(math.radians(angle))
    else:
        turning_radius = None
    return wheel_drive, steering, turning_radius


def compute_cmd_vel(
    drive: float,
    steer: float,
    turn: float,
    steering_mode: int,
    speed_limit_mps: float,
    yaw_limit_radps: float,
    power_pct: float,
) -> tuple[float, float, float]:
    """Convert shaped operator inputs into ROS REP-103 Twist components.

    Returns (linear_x, linear_y, angular_z):
    - +linear_x: forward (m/s)
    - +linear_y: left (m/s)
    - +angular_z: counter-clockwise / turn left (rad/s); so right turn gives -angular_z.
    """
    max_v = max(0.0, float(speed_limit_mps))
    max_w = max(0.0, float(yaw_limit_radps))
    power_scale = clamp(power_pct, 0.0, 100.0) / 100.0

    if steering_mode == MODE_TURN_IN_PLACE:
        linear_x = 0.0
        linear_y = 0.0
        angular_z = clamp(-(turn / 100.0) * max_w * power_scale, -max_w, max_w)
        return linear_x, linear_y, angular_z

    if steering_mode == MODE_CRAB:
        linear_x = clamp((drive / 100.0) * max_v * power_scale, -max_v, max_v)
        linear_y = 0.0
        angular_z = clamp(-(turn / 100.0) * max_w * power_scale, -max_w, max_w)
        return linear_x, linear_y, angular_z

    # MODE_COORDINATED
    linear_x = clamp((drive / 100.0) * max_v * power_scale, -max_v, max_v)
    linear_y = 0.0
    turn_term = clamp(turn, -100.0, 100.0) / 100.0
    if abs(drive) > 1.0:
        steer_term = (clamp(steer, -100.0, 100.0) / 100.0) * (1.0 if drive >= 0.0 else -1.0)
    else:
        steer_term = 0.0
    combined_yaw = clamp(turn_term + 0.75 * steer_term, -1.0, 1.0)
    angular_z = clamp(-combined_yaw * max_w * power_scale, -max_w, max_w)
    return linear_x, linear_y, angular_z


class SafetyController:
    """State machine enforcing arming, preview isolation, and E-STOP latching."""

    def __init__(self):
        self.output_mode = OUTPUT_PREVIEW
        self.ros_connected = False
        self.preview_enabled = False
        self.armed = False
        self.estop_latched = False

    def can_publish_ros(self) -> bool:
        """Return True only when live ROS teleop is selected, connected, armed, and not latched."""
        return (
            self.output_mode == OUTPUT_ROS_TELEOP
            and self.ros_connected
            and self.armed
            and not self.estop_latched
        )

    def set_output_mode(self, mode: str) -> None:
        """Switch between OUTPUT_PREVIEW and OUTPUT_ROS_TELEOP. Always disarms and stops preview."""
        if mode not in (OUTPUT_PREVIEW, OUTPUT_ROS_TELEOP):
            raise ValueError(f"Unsupported output mode: {mode}")
        self.output_mode = mode
        self.armed = False
        self.preview_enabled = False

    def set_preview_enabled(self, enabled: bool) -> bool:
        """Enable/disable local preview. Always forces disarmed state."""
        if enabled and self.output_mode != OUTPUT_PREVIEW:
            self.set_output_mode(OUTPUT_PREVIEW)
        self.armed = False
        self.preview_enabled = bool(enabled) and self.output_mode == OUTPUT_PREVIEW
        return self.preview_enabled

    def on_ros_connected(self, connected: bool) -> None:
        """Update ROS link availability. Disconnection always disarms."""
        self.ros_connected = bool(connected)
        if not self.ros_connected:
            self.armed = False

    def set_armed(self, desired: bool) -> bool:
        """Arm or disarm ROS teleop output. Returns resulting armed state."""
        if not desired:
            self.armed = False
            return False
        if self.output_mode != OUTPUT_ROS_TELEOP or not self.ros_connected or self.estop_latched:
            self.armed = False
            return False
        self.preview_enabled = False
        self.armed = True
        return True

    def trigger_estop(self) -> bool:
        """Latch E-STOP, disarm immediately, and stop preview."""
        self.estop_latched = True
        self.armed = False
        self.preview_enabled = False
        return True

    def reset_latch(self) -> bool:
        """Clear E-STOP latch while remaining safely disarmed."""
        self.estop_latched = False
        self.armed = False
        return True

    def state(self) -> str:
        """Return high-level safety state identifier."""
        if self.estop_latched:
            return SAFETY_ESTOP
        if self.output_mode == OUTPUT_PREVIEW:
            return SAFETY_PREVIEW
        if self.can_publish_ros():
            return SAFETY_ARMED
        return SAFETY_DISARMED

"""Animated Mars Rover top-down schematic with 4WD/4WS trajectory preview, slip, and tilt HUD."""

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ROV_Control_Station_Integrated import animate, approach, mono_font
from rover_kinematics import (
    DRIVE_MOTORS,
    MODE_COORDINATED,
    MODE_CRAB,
    MODE_TURN_IN_PLACE,
    SAFETY_ARMED,
    SAFETY_DISARMED,
    SAFETY_ESTOP,
    SAFETY_PREVIEW,
    WHEELS,
)
from rover_theme import ACCENT, DANGER, WARN


class RoverView(QWidget):
    """Four drive wheels and four steering pivots; live telemetry and preview stay distinct."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(380, 290)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.readings = {}
        self.fixed_steering = False
        self.command_active = False
        self.safety_state = SAFETY_PREVIEW
        self.steering_mode = MODE_COORDINATED
        self.drive_commands = dict.fromkeys(WHEELS, 0.0)
        self.steer_commands = dict.fromkeys(WHEELS, 0.0)
        self.drive_axis = 0.0
        self.steer_axis = 0.0
        self.turn_axis = 0.0
        self.turning_radius_m: float | None = None

        self.slip_detected = False
        self.high_tilt = False
        self.tilt_value_deg = 0.0

        self.speeds = dict.fromkeys(WHEELS, 0.0)
        self.angles = dict.fromkeys(WHEELS, 0.0)
        self.treads = dict.fromkeys(WHEELS, 0.0)
        self.phase = 0.0
        animate(self)

    def set_readings(self, readings: dict):
        self.readings = dict(readings)
        self.update()

    def set_alerts(self, slip_detected: bool, roll_deg: float = 0.0, pitch_deg: float = 0.0):
        self.slip_detected = bool(slip_detected)
        max_tilt = max(abs(roll_deg), abs(pitch_deg))
        self.high_tilt = max_tilt > 20.0
        self.tilt_value_deg = max_tilt
        self.update()

    def set_commands(
        self,
        active: bool,
        drive: dict,
        steering: dict,
        steering_mode: int = MODE_COORDINATED,
        safety_state: str = SAFETY_PREVIEW,
        drive_axis: float = 0.0,
        steer_axis: float = 0.0,
        turn_axis: float = 0.0,
        turning_radius_m: float | None = None,
    ):
        self.command_active = bool(active)
        self.safety_state = safety_state
        self.steering_mode = steering_mode
        self.drive_commands = dict(drive)
        self.steer_commands = dict(steering)
        self.drive_axis = float(drive_axis)
        self.steer_axis = float(steer_axis)
        self.turn_axis = float(turn_axis)
        self.turning_radius_m = turning_radius_m

    def anim_step(self, dt, phase):
        self.phase = phase
        for wheel in WHEELS:
            reading = self.readings.get(f"D_{wheel}")
            live_val = reading["value"] if reading else 0.0
            target = self.drive_commands.get(wheel, 0.0) if self.command_active and not reading else live_val if reading else self.drive_commands.get(wheel, 0.0) if self.command_active else 0.0
            angle = self.steer_commands.get(wheel, 0.0) if self.command_active else 0.0
            self.speeds[wheel] = approach(self.speeds[wheel], target, dt, 10)
            self.angles[wheel] = approach(self.angles[wheel], angle, dt, 12)
            self.treads[wheel] = (self.treads[wheel] + self.speeds[wheel] * dt * 0.65) % 10.0
        self.update()

    def _draw_predicted_path(self, p: QPainter):
        if not self.command_active:
            return
        path_col = QColor("#fbbf24" if self.safety_state == SAFETY_ARMED else ACCENT)
        path_col.setAlpha(185)
        p.setPen(QPen(path_col, 2.2, Qt.PenStyle.DashLine, Qt.PenCapStyle.RoundCap))
        p.setBrush(Qt.BrushStyle.NoBrush)

        if self.steering_mode == MODE_TURN_IN_PLACE and abs(self.turn_axis) > 2.0:
            clockwise = self.turn_axis > 0.0
            ring_r = 165.0
            rect = QRectF(-ring_r, -ring_r, 2 * ring_r, 2 * ring_r)
            for base_deg in (45, 225):
                span = -55 if clockwise else 55
                p.drawArc(rect, base_deg * 16, span * 16)
            return

        if self.steering_mode == MODE_CRAB and (abs(self.drive_axis) > 2.0 or abs(self.steer_axis) > 2.0):
            angle_rad = math.radians(self.angles["FL"])
            sign = 1.0 if self.drive_axis >= 0.0 else -1.0
            dx = math.sin(angle_rad) * 75.0 * sign
            dy = -math.cos(angle_rad) * 75.0 * sign
            for ox in (-40, 40):
                start = QPointF(ox, -135 if sign > 0 else 135)
                p.drawLine(start, QPointF(start.x() + dx, start.y() + dy))
            return

        # Coordinated mode trajectory curve projecting clearly ahead/behind chassis
        if abs(self.drive_axis) > 2.0 or abs(self.steer_axis) > 2.0:
            if self.turning_radius_m is not None and abs(self.steer_axis) > 2.0:
                r_px = max(120.0, min(420.0, abs(self.turning_radius_m) * 260.0))
                turn_right = self.steer_axis > 0.0
                cx = r_px if turn_right else -r_px
                cy = -55.0 if self.drive_axis >= -1.0 else 55.0
                rect = QRectF(cx - r_px, cy - r_px, 2 * r_px, 2 * r_px)
                bumper_deg = math.degrees(math.atan2(85.0, r_px))
                forward = self.drive_axis >= -1.0
                if turn_right:
                    start_deg = (180.0 - bumper_deg) if forward else (180.0 + bumper_deg)
                    span_deg = -38.0 if forward else 38.0
                else:
                    start_deg = bumper_deg if forward else -bumper_deg
                    span_deg = 38.0 if forward else -38.0
                p.drawArc(rect, int(start_deg * 16), int(span_deg * 16))
            elif abs(self.drive_axis) > 2.0:
                dir_y = -1.0 if self.drive_axis >= 0 else 1.0
                p.drawLine(QPointF(0, dir_y * 135), QPointF(0, dir_y * 200))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        scale = min(self.width() / 620.0, self.height() / 440.0)
        p.translate(self.width() * 0.5, self.height() * 0.5)
        p.scale(scale, scale)

        # Background engineering grid
        p.setPen(QPen(QColor("#232533"), 1))
        for x in range(-300, 301, 30):
            p.drawLine(QPointF(x, -205), QPointF(x, 205))
        for y in range(-180, 181, 30):
            p.drawLine(QPointF(-300, y), QPointF(300, y))

        # Static rings + 3 pulsing Mars-red radar waves
        p.setBrush(Qt.BrushStyle.NoBrush)
        for radius in (130, 195, 265):
            p.setPen(QPen(QColor("#302730"), 1))
            p.drawEllipse(QPointF(0, 0), radius, radius)
        for j in range(3):
            t = (self.phase * 0.22 + j / 3.0) % 1.0
            pulse_col = QColor(ACCENT)
            pulse_col.setAlpha(int(55 * (1.0 - t)))
            p.setPen(QPen(pulse_col, 1.5))
            r = 55.0 + 185.0 * t
            p.drawEllipse(QPointF(0, 0), r, r)

        # Predicted trajectory overlay
        self._draw_predicted_path(p)

        # Rover chassis
        body = QRectF(-80, -125, 160, 250)
        gradient = QLinearGradient(body.topLeft(), body.bottomRight())
        gradient.setColorAt(0, QColor("#6c4547"))
        gradient.setColorAt(1, QColor("#27232e"))
        p.setBrush(gradient)
        p.setPen(QPen(QColor("#af6c65"), 2))
        p.drawRoundedRect(body, 22, 22)

        # Electronics deck
        p.setBrush(QColor("#17212c"))
        p.drawRoundedRect(QRectF(-56, -65, 112, 135), 10, 10)
        p.setPen(QPen(QColor("#3a5270"), 2))
        for x in range(-40, 41, 20):
            p.drawLine(QPointF(x, -56), QPointF(x, 60))
        for y in range(-48, 64, 18):
            p.drawLine(QPointF(-46, y), QPointF(46, y))

        # Stereo camera mast & forward direction indicator
        p.setPen(QPen(QColor(ACCENT), 4))
        p.drawLine(QPointF(0, -80), QPointF(0, -155))
        p.setBrush(QColor("#563238"))
        p.drawRoundedRect(QRectF(-30, -168, 60, 24), 6, 6)
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QPointF(-15, -156), 5, 5)
        p.drawEllipse(QPointF(15, -156), 5, 5)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -115), QPointF(-10, -95), QPointF(10, -95)]))

        p.setFont(mono_font(12, True))
        p.setPen(QColor("#cbd5e1"))
        p.drawText(QRectF(-70, 80, 140, 24), Qt.AlignmentFlag.AlignCenter, "MARS ROVER")

        # Four drive + steering wheels
        spots = ((-126, -95), (126, -95), (-126, 95), (126, 95))
        slip_pulse = 0.5 + 0.5 * math.sin(self.phase * 8.0)

        for idx, (wheel, (x, y)) in enumerate(zip(WHEELS, spots)):
            drive = self.readings.get(f"D_{wheel}")
            active = drive is not None or self.command_active

            # Suspension arm
            p.setPen(QPen(QColor("#4b6580"), 10))
            p.drawLine(QPointF(-80 if x < 0 else 80, y), QPointF(x, y))

            # Slip warning ring
            if self.slip_detected:
                sc = QColor(DANGER)
                sc.setAlpha(int(120 + 130 * slip_pulse))
                p.setPen(QPen(sc, 3))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(QPointF(x, y), 30, 30)

            # Steering pivot collar
            p.setPen(QPen(QColor(ACCENT if self.command_active else "#66788f"), 2))
            p.setBrush(QColor("#101822"))
            p.drawEllipse(QPointF(x, y), 23, 23)

            # Rotated wheel + clipped seamless treads
            p.save()
            p.translate(x, y)
            p.rotate(self.angles[wheel])

            wheel_rect = QRectF(-20, -37, 40, 74)
            glow = abs(self.speeds[wheel]) > 1.0
            border_col = QColor(DANGER if self.slip_detected else (ACCENT if glow or active else "#66788f"))
            p.setPen(QPen(border_col, 2.2))
            p.setBrush(QColor("#1e2d3f" if glow else "#182735"))
            p.drawRoundedRect(wheel_rect, 7, 7)

            clip = QPainterPath()
            clip.addRoundedRect(wheel_rect.adjusted(2, 2, -2, -2), 5, 5)
            p.setClipPath(clip)
            p.setPen(QPen(QColor("#456080" if glow else "#3a5270"), 3))
            offset = self.treads[wheel] % 10.0
            for tread in range(-45, 46, 10):
                ty = tread + offset
                p.drawLine(QPointF(-16, ty), QPointF(16, ty))
            p.restore()

            # Label box (155px wide, fixed logical font sizes -> zero overflow)
            label_x = -305 if x < 0 else 150
            p.setFont(mono_font(12, True))
            p.setPen(QColor(ACCENT))
            p.drawText(
                QRectF(label_x, y - 34, 155, 22),
                Qt.AlignmentFlag.AlignCenter,
                f"D{idx + 1}+S{idx + 1} · {wheel}",
            )
            p.setFont(mono_font(10))
            p.setPen(QColor("#cbd5e1") if active else QColor("#9fb0c3"))
            if drive is not None and self.command_active:
                rpm_txt = f"{drive['value']:+.1f} rpm · {self.drive_commands[wheel]:+.0f}%"
            elif drive is not None:
                rpm_txt = f"Drive: {drive['value']:+.1f} rpm"
            elif self.command_active:
                rpm_txt = f"Cmd: {self.drive_commands[wheel]:+.0f}%"
            else:
                rpm_txt = "Drive: -- rpm"

            angle_txt = (
                "Fixed wheel · skid steer" if self.fixed_steering else
                f"Steer: {self.steer_commands[wheel]:+.0f}° sim"
                if self.command_active
                else "Steer: deferred"
            )
            p.drawText(QRectF(label_x, y - 11, 155, 20), Qt.AlignmentFlag.AlignCenter, rpm_txt)
            p.drawText(QRectF(label_x, y + 10, 155, 20), Qt.AlignmentFlag.AlignCenter, angle_txt)

        # Top HUD alert badges (slip / chassis tilt)
        if self.slip_detected or self.high_tilt:
            badge_txt = "WHEEL SLIP DETECTED" if self.slip_detected else f"HIGH TILT ({self.tilt_value_deg:.0f}°)"
            if self.slip_detected and self.high_tilt:
                badge_txt = f"WHEEL SLIP + HIGH TILT ({self.tilt_value_deg:.0f}°)"
            badge_col = QColor(DANGER if self.slip_detected else WARN)
            badge_rect = QRectF(-150, -210, 300, 24)
            p.setPen(QPen(badge_col, 1.5))
            p.setBrush(QColor(48, 20, 22, 220))
            p.drawRoundedRect(badge_rect, 10, 10)
            p.setFont(mono_font(10, True))
            p.setPen(badge_col.lighter(130))
            p.drawText(badge_rect, Qt.AlignmentFlag.AlignCenter, badge_txt)

        # Footer status banner
        if self.safety_state == SAFETY_ESTOP:
            footer = "E-STOP LATCHED · outputs forced to zero"
            fcol = QColor(DANGER)
        elif self.safety_state == SAFETY_ARMED:
            footer = "ROS TELEOP ARMED · live /cmd_vel · steering feedback deferred"
            fcol = QColor("#fbbf24")
        elif self.command_active:
            footer = "PREVIEW ONLY · no commands sent to ROS or hardware"
            fcol = QColor(ACCENT)
        elif self.safety_state == SAFETY_DISARMED:
            footer = "ROS TELEOP DISARMED · click ARM to drive · steering deferred"
            fcol = QColor("#8aa0b6")
        else:
            footer = "LIVE TELEMETRY · read-only · steering feedback deferred"
            fcol = QColor("#7c8ea6")

        p.setPen(fcol)
        p.setFont(mono_font(11))
        p.drawText(QRectF(-300, 188, 600, 24), Qt.AlignmentFlag.AlignCenter, footer)

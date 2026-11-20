"""Rover schematic in the control station's existing Qt drawing style."""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ROV_Control_Station_Integrated import animate, approach, mono_font
from rover_theme import ACCENT


WHEELS = ("FL", "FR", "RL", "RR")
DRIVE_MOTORS = tuple(f"D_{wheel}" for wheel in WHEELS)


class RoverView(QWidget):
    """Four drive wheels and four steering pivots; unknown data stays unknown."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(380, 320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.readings = {}
        self.preview = False
        self.drive_commands = dict.fromkeys(WHEELS, 0.0)
        self.steer_commands = dict.fromkeys(WHEELS, 0.0)
        self.speeds = dict.fromkeys(WHEELS, 0.0)
        self.angles = dict.fromkeys(WHEELS, 0.0)
        self.treads = dict.fromkeys(WHEELS, 0.0)
        self.phase = 0.0
        animate(self)

    def set_readings(self, readings):
        self.readings = readings
        self.update()

    def set_commands(self, enabled, drive, steering):
        self.preview = enabled
        self.drive_commands = dict(drive)
        self.steer_commands = dict(steering)

    def anim_step(self, dt, phase):
        self.phase = phase
        for wheel in WHEELS:
            reading = self.readings.get(f"D_{wheel}")
            target = self.drive_commands.get(wheel, 0.0) if self.preview else reading["value"] if reading else 0.0
            angle = self.steer_commands.get(wheel, 0.0) if self.preview else 0.0
            self.speeds[wheel] = approach(self.speeds[wheel], target, dt, 10)
            self.angles[wheel] = approach(self.angles[wheel], angle, dt, 12)
            self.treads[wheel] = (self.treads[wheel] + self.speeds[wheel] * dt * 0.65) % 10
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        # Fixed design coordinates keep labels legible as the centre panel grows.
        scale = min(self.width() / 620, self.height() / 440)
        px = max(12, round(11 / scale))
        p.translate(self.width() / 2, self.height() / 2)
        p.scale(scale, scale)
        p.setPen(QPen(QColor("#252532"), 1))
        for x in range(-300, 301, 30):
            p.drawLine(QPointF(x, -205), QPointF(x, 205))
        for y in range(-180, 181, 30):
            p.drawLine(QPointF(-300, y), QPointF(300, y))

        p.setBrush(Qt.BrushStyle.NoBrush)
        for radius in (130, 195, 265):
            p.setPen(QPen(QColor("#302730"), 1))
            p.drawEllipse(QPointF(0, 0), radius, radius)
        scan = QColor(ACCENT)
        scan.setAlpha(35)
        p.setPen(QPen(scan, 2))
        p.drawArc(QRectF(-195, -195, 390, 390), int(self.phase * 24 * 16) % 5760, 42 * 16)
        body = QRectF(-80, -125, 160, 250)
        gradient = QLinearGradient(body.topLeft(), body.bottomRight())
        gradient.setColorAt(0, QColor("#6c4547"))
        gradient.setColorAt(1, QColor("#27232e"))
        p.setBrush(gradient)
        p.setPen(QPen(QColor("#af6c65"), 2))
        p.drawRoundedRect(body, 22, 22)
        p.setBrush(QColor("#17212c"))
        p.drawRoundedRect(QRectF(-56, -65, 112, 135), 10, 10)
        p.setPen(QPen(QColor("#3a5270"), 2))
        for x in range(-40, 41, 20):
            p.drawLine(QPointF(x, -56), QPointF(x, 60))
        for y in range(-48, 64, 18):
            p.drawLine(QPointF(-46, y), QPointF(46, y))
        p.setPen(QPen(QColor(ACCENT), 4))
        p.drawLine(QPointF(0, -80), QPointF(0, -155))
        p.setBrush(QColor("#563238"))
        p.drawRoundedRect(QRectF(-30, -168, 60, 24), 6, 6)
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QPointF(-15, -156), 5, 5)
        p.drawEllipse(QPointF(15, -156), 5, 5)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -115), QPointF(-10, -95), QPointF(10, -95)]))
        p.setFont(mono_font(px + 1, True))
        p.setPen(QColor("#9fb0c3"))
        p.drawText(QRectF(-70, 80, 140, 25), Qt.AlignmentFlag.AlignCenter, "MARS ROVER")

        spots = ((-126, -95), (126, -95), (-126, 95), (126, 95))
        for wheel, (x, y) in zip(WHEELS, spots):
            drive = self.readings.get(f"D_{wheel}")
            steer = self.preview
            p.setPen(QPen(QColor("#4b6580"), 10))
            p.drawLine(QPointF(-80 if x < 0 else 80, y), QPointF(x, y))
            p.setPen(QPen(QColor(ACCENT if steer else "#66788f"), 2))
            p.setBrush(QColor("#101822"))
            p.drawEllipse(QPointF(x, y), 23, 23)
            p.save()
            p.translate(x, y)
            p.rotate(self.angles[wheel])
            p.setPen(QPen(QColor(ACCENT if drive or self.preview else "#66788f"), 2))
            p.setBrush(QColor("#182735"))
            p.drawRoundedRect(QRectF(-20, -37, 40, 74), 7, 7)
            p.setPen(QPen(QColor("#3a5270"), 3))
            for tread in range(-25, 26, 10):
                p.drawLine(QPointF(-15, tread + self.treads[wheel]), QPointF(15, tread + self.treads[wheel]))
            p.restore()

            label_x = -300 if x < 0 else 170
            p.setFont(mono_font(px + 1, True))
            p.setPen(QColor(ACCENT))
            p.drawText(QRectF(label_x, y - 34, 130, 22), Qt.AlignmentFlag.AlignCenter,
                       f"{wheel}  D + S")
            p.setFont(mono_font(px))
            p.setPen(QColor("#9fb0c3"))
            rpm = f'{self.drive_commands[wheel]:+.0f}% preview' if self.preview else f'{drive["value"]:+.1f} rpm' if drive else "Drive: -- rpm"
            angle = f'{self.steer_commands[wheel]:+.0f}° preview' if self.preview else "Steer: pending"
            p.drawText(QRectF(label_x, y - 10, 130, 22), Qt.AlignmentFlag.AlignCenter, rpm)
            p.drawText(QRectF(label_x, y + 12, 130, 22), Qt.AlignmentFlag.AlignCenter, angle)
        p.setPen(QColor("#66788f"))
        p.setFont(mono_font(px))
        p.drawText(QRectF(-300, 183, 600, 24), Qt.AlignmentFlag.AlignCenter,
                   "PREVIEW ONLY · no commands sent" if self.preview else "LIVE · drive RPM · steering feedback pending")

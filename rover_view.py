"""Rover schematic in the control station's existing Qt drawing style."""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ROV_Control_Station_Integrated import mono_font


WHEELS = ("FL", "FR", "RL", "RR")
DRIVE_MOTORS = tuple(f"D_{wheel}" for wheel in WHEELS)


class RoverView(QWidget):
    """Four drive wheels and four steering pivots; unknown data stays unknown."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(380, 320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.readings = {}

    def set_readings(self, readings):
        self.readings = readings
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        # Fixed design coordinates keep labels legible as the centre panel grows.
        scale = min(self.width() / 620, self.height() / 440)
        p.translate(self.width() / 2, self.height() / 2)
        p.scale(scale, scale)
        p.setPen(QPen(QColor("#16212d"), 1))
        for x in range(-300, 301, 30):
            p.drawLine(QPointF(x, -205), QPointF(x, 205))
        for y in range(-180, 181, 30):
            p.drawLine(QPointF(-300, y), QPointF(300, y))

        body = QRectF(-80, -125, 160, 250)
        gradient = QLinearGradient(body.topLeft(), body.bottomRight())
        gradient.setColorAt(0, QColor("#33475f"))
        gradient.setColorAt(1, QColor("#1a2533"))
        p.setBrush(gradient)
        p.setPen(QPen(QColor("#5a7699"), 2))
        p.drawRoundedRect(body, 22, 22)
        p.setBrush(QColor("#101e2d"))
        p.drawRoundedRect(QRectF(-56, -65, 112, 135), 10, 10)
        p.setPen(QPen(QColor("#3a5270"), 2))
        for y in range(-48, 64, 18):
            p.drawLine(QPointF(-46, y), QPointF(46, y))
        p.setPen(QPen(QColor("#5fd0ff"), 4))
        p.drawLine(QPointF(0, -80), QPointF(0, -155))
        p.setBrush(QColor("#203c50"))
        p.drawRoundedRect(QRectF(-30, -168, 60, 24), 6, 6)
        p.setBrush(QColor("#5fd0ff"))
        p.drawEllipse(QPointF(-15, -156), 5, 5)
        p.drawEllipse(QPointF(15, -156), 5, 5)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -115), QPointF(-10, -95), QPointF(10, -95)]))
        p.setFont(mono_font(13, True))
        p.setPen(QColor("#9fb0c3"))
        p.drawText(QRectF(-70, 80, 140, 25), Qt.AlignmentFlag.AlignCenter, "MARS ROVER")

        spots = ((-126, -95), (126, -95), (-126, 95), (126, 95))
        for wheel, (x, y) in zip(WHEELS, spots):
            drive = self.readings.get(f"D_{wheel}")
            steer = self.readings.get(f"S_{wheel}")
            p.setPen(QPen(QColor("#4b6580"), 10))
            p.drawLine(QPointF(-80 if x < 0 else 80, y), QPointF(x, y))
            p.setPen(QPen(QColor("#2dd4bf" if steer else "#66788f"), 2))
            p.setBrush(QColor("#101822"))
            p.drawEllipse(QPointF(x, y), 23, 23)
            p.save()
            p.translate(x, y)
            if steer is not None:
                p.rotate(steer["value"])
            p.setPen(QPen(QColor("#5fd0ff" if drive else "#66788f"), 2))
            p.setBrush(QColor("#182735"))
            p.drawRoundedRect(QRectF(-20, -37, 40, 74), 7, 7)
            p.setPen(QPen(QColor("#3a5270"), 3))
            for tread in range(-25, 26, 10):
                p.drawLine(QPointF(-15, tread), QPointF(15, tread))
            p.restore()

            label_x = -300 if x < 0 else 170
            p.setFont(mono_font(13, True))
            p.setPen(QColor("#5fd0ff"))
            p.drawText(QRectF(label_x, y - 34, 130, 22), Qt.AlignmentFlag.AlignCenter,
                       f"{wheel}  D + S")
            p.setFont(mono_font(12))
            p.setPen(QColor("#9fb0c3"))
            rpm = f'{drive["value"]:+.1f} rpm' if drive else "Drive: -- rpm"
            angle = f'{steer["value"]:+.1f}°' if steer else "Steer: deferred"
            p.drawText(QRectF(label_x, y - 10, 130, 22), Qt.AlignmentFlag.AlignCenter, rpm)
            p.drawText(QRectF(label_x, y + 12, 130, 22), Qt.AlignmentFlag.AlignCenter, angle)
        p.setPen(QColor("#66788f"))
        p.setFont(mono_font(12))
        p.drawText(QRectF(-300, 183, 600, 24), Qt.AlignmentFlag.AlignCenter,
                   "TOP VIEW · D = drive · S = steering · -- = no fresh telemetry")

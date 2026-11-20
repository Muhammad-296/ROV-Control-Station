"""Interactive 2D Mars Yard SLAM map, Nav2 path, ArUco/obstacle overlay, and waypoint runner."""

from collections import deque
import json
import math
from pathlib import Path
import time

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QAbstractItemView, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ROV_Control_Station_Integrated import animate, mono_font
from rover_kinematics import DEFAULT_TRACK_WIDTH_M, DEFAULT_WHEELBASE_M, clamp
from rover_theme import ACCENT


TOOL_PAN = "pan"
TOOL_WAYPOINT = "waypoint"
TOOL_GOAL = "goal"


class MapCanvas(QWidget):
    """Zoomable/pannable 2D ROS map canvas with live rover, SLAM/Nav2, and perception overlays."""

    goal_requested = Signal(float, float, float)      # x, y, yaw_rad
    waypoint_added = Signal(float, float, float)      # x, y, yaw_deg

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)

        self.cam_x = 0.0
        self.cam_y = 0.0
        self.px_per_m = 48.0
        self.follow_rover = True
        self.tool = TOOL_PAN
        self.goal_allowed = False

        self.map_info: dict | None = None
        self.pose: dict | None = None
        self.trail: deque[tuple[float, float]] = deque(maxlen=800)
        self.plan: list[tuple[float, float]] = []
        self.aruco_markers: list[dict] = []
        self.obstacles: list[dict] = []
        self.waypoints: list[dict] = []
        self.active_wp_index: int | None = None
        self.active_goal: tuple[float, float, float] | None = None

        self._phase = 0.0
        self._drag_mode: str | None = None
        self._pan_start = QPointF()
        self._cam_start = (0.0, 0.0)
        self._goal_start_world: tuple[float, float] | None = None
        self._goal_cur_world: tuple[float, float] | None = None
        self._cursor_world: tuple[float, float] | None = None
        animate(self)

    def anim_step(self, dt, phase):
        self._phase = phase
        if self.pose is not None or self.active_goal is not None:
            self.update()

    def world_to_screen(self, wx: float, wy: float) -> QPointF:
        sx = self.width() * 0.5 + (wx - self.cam_x) * self.px_per_m
        sy = self.height() * 0.5 - (wy - self.cam_y) * self.px_per_m
        return QPointF(sx, sy)

    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        wx = self.cam_x + (sx - self.width() * 0.5) / self.px_per_m
        wy = self.cam_y - (sy - self.height() * 0.5) / self.px_per_m
        return wx, wy

    def set_tool(self, tool: str):
        self.tool = tool
        self._goal_start_world = None
        self._goal_cur_world = None
        self.setCursor(Qt.CursorShape.CrossCursor if tool in (TOOL_WAYPOINT, TOOL_GOAL) else Qt.CursorShape.OpenHandCursor)
        self.update()

    def set_follow(self, enabled: bool):
        self.follow_rover = bool(enabled)
        if self.follow_rover and self.pose is not None:
            self.cam_x = self.pose["x"]
            self.cam_y = self.pose["y"]
            self.update()

    def set_map(self, info: dict | None):
        first = self.map_info is None and info is not None
        self.map_info = info
        if first and not self.follow_rover:
            self.fit_view()
        self.update()

    def set_odometry(self, odom: dict):
        self.pose = dict(odom)
        pt = (odom["x"], odom["y"])
        if not self.trail or math.hypot(pt[0] - self.trail[-1][0], pt[1] - self.trail[-1][1]) >= 0.05:
            self.trail.append(pt)
        if self.follow_rover:
            self.cam_x = pt[0]
            self.cam_y = pt[1]
        self.update()

    def set_path(self, pts: list[tuple[float, float]]):
        self.plan = list(pts)
        self.update()

    def set_aruco(self, pose: dict):
        entry = dict(pose)
        entry["seen"] = time.monotonic()
        wx, wy = self._to_world_xy(entry["x"], entry["y"], entry.get("frame_id", ""))
        entry["wx"], entry["wy"] = wx, wy
        # Replace nearby marker or append up to 12 markers.
        for i, existing in enumerate(self.aruco_markers):
            if math.hypot(existing["wx"] - wx, existing["wy"] - wy) < 0.45:
                self.aruco_markers[i] = entry
                self.update()
                return
        self.aruco_markers.append(entry)
        if len(self.aruco_markers) > 12:
            self.aruco_markers.pop(0)
        self.update()

    def set_obstacles(self, dets: list[dict]):
        self.obstacles = list(dets)
        self.update()

    def set_waypoints(self, waypoints: list[dict], active_index: int | None):
        self.waypoints = list(waypoints)
        self.active_wp_index = active_index
        self.update()

    def clear_trail(self):
        self.trail.clear()
        self.update()

    def fit_view(self):
        if self.map_info is not None:
            mi = self.map_info
            mw = mi["width"] * mi["resolution"]
            mh = mi["height"] * mi["resolution"]
            self.cam_x = mi["origin_x"] + mw * 0.5
            self.cam_y = mi["origin_y"] + mh * 0.5
            if mw > 0 and mh > 0:
                zx = max(10.0, (self.width() * 0.85) / mw)
                zy = max(10.0, (self.height() * 0.85) / mh)
                self.px_per_m = clamp(min(zx, zy), 10.0, 250.0)
        elif self.pose is not None:
            self.cam_x = self.pose["x"]
            self.cam_y = self.pose["y"]
            self.px_per_m = 48.0
        else:
            self.cam_x = 0.0
            self.cam_y = 0.0
            self.px_per_m = 48.0
        self.update()

    def _to_world_xy(self, x: float, y: float, frame_id: str) -> tuple[float, float]:
        fid = frame_id.lower()
        if ("base_link" in fid or "camera" in fid) and self.pose is not None:
            yaw = math.radians(self.pose["yaw_deg"])
            c, s = math.cos(yaw), math.sin(yaw)
            return (
                self.pose["x"] + x * c - y * s,
                self.pose["y"] + x * s + y * c,
            )
        return x, y

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.15 if delta > 0 else (1.0 / 1.15)
        pos = event.position()
        wx, wy = self.screen_to_world(pos.x(), pos.y())
        self.px_per_m = clamp(self.px_per_m * factor, 10.0, 250.0)
        if not self.follow_rover:
            self.cam_x = wx - (pos.x() - self.width() * 0.5) / self.px_per_m
            self.cam_y = wy + (pos.y() - self.height() * 0.5) / self.px_per_m
        self.update()
        event.accept()

    def mousePressEvent(self, event):
        pos = event.position()
        if event.button() == Qt.MouseButton.MiddleButton or (
            event.button() == Qt.MouseButton.LeftButton and self.tool == TOOL_PAN
        ):
            self._drag_mode = "pan"
            self._pan_start = pos
            self._cam_start = (self.cam_x, self.cam_y)
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        if event.button() == Qt.MouseButton.LeftButton and self.tool in (TOOL_WAYPOINT, TOOL_GOAL):
            self._drag_mode = self.tool
            wx, wy = self.screen_to_world(pos.x(), pos.y())
            self._goal_start_world = (wx, wy)
            self._goal_cur_world = (wx, wy)
            self.update()
            event.accept()

    def mouseMoveEvent(self, event):
        pos = event.position()
        self._cursor_world = self.screen_to_world(pos.x(), pos.y())
        if self._drag_mode == "pan":
            dx = (pos.x() - self._pan_start.x()) / self.px_per_m
            dy = (pos.y() - self._pan_start.y()) / self.px_per_m
            self.cam_x = self._cam_start[0] - dx
            self.cam_y = self._cam_start[1] + dy
            if self.follow_rover and (abs(dx) > 0.05 or abs(dy) > 0.05):
                # Keep follow toggle authoritative from UI; panning shifts center until next odom tick.
                pass
            self.update()
        elif self._drag_mode in (TOOL_WAYPOINT, TOOL_GOAL):
            self._goal_cur_world = self._cursor_world
            self.update()
        else:
            self.update()

    def mouseReleaseEvent(self, event):
        if self._drag_mode == "pan":
            self._drag_mode = None
            self.set_tool(self.tool)
            return
        if self._drag_mode in (TOOL_WAYPOINT, TOOL_GOAL) and self._goal_start_world is not None:
            sx, sy = self._goal_start_world
            ex, ey = self._goal_cur_world or self._goal_start_world
            dist_px = math.hypot(ex - sx, ey - sy) * self.px_per_m
            if dist_px >= 8.0:
                yaw_rad = math.atan2(ey - sy, ex - sx)
            elif self.pose is not None:
                yaw_rad = math.atan2(sy - self.pose["y"], sx - self.pose["x"])
            else:
                yaw_rad = 0.0

            mode = self._drag_mode
            self._drag_mode = None
            self._goal_start_world = None
            self._goal_cur_world = None
            if mode == TOOL_WAYPOINT:
                self.waypoint_added.emit(sx, sy, math.degrees(yaw_rad))
            elif mode == TOOL_GOAL:
                self.active_goal = (sx, sy, yaw_rad)
                self.goal_requested.emit(sx, sy, yaw_rad)
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#0b0f17"))

        # 1. Metric Grid
        step_m = 1.0
        for candidate in (0.5, 1.0, 2.0, 5.0, 10.0):
            if candidate * self.px_per_m >= 48.0:
                step_m = candidate
                break
        x0_w, y1_w = self.screen_to_world(0, 0)
        x1_w, y0_w = self.screen_to_world(w, h)
        gx_start = math.floor(x0_w / step_m) * step_m
        gy_start = math.floor(y0_w / step_m) * step_m

        p.setPen(QPen(QColor("#171d29"), 1))
        gx = gx_start
        while gx <= x1_w + step_m:
            sx = self.world_to_screen(gx, 0).x()
            p.drawLine(QPointF(sx, 0), QPointF(sx, h))
            gx += step_m
        gy = gy_start
        while gy <= y1_w + step_m:
            sy = self.world_to_screen(0, gy).y()
            p.drawLine(QPointF(0, sy), QPointF(w, sy))
            gy += step_m

        # World origin crosshairs
        orig = self.world_to_screen(0.0, 0.0)
        p.setPen(QPen(QColor("#2c2a3a"), 1.5))
        p.drawLine(QPointF(orig.x(), 0), QPointF(orig.x(), h))
        p.drawLine(QPointF(0, orig.y()), QPointF(w, orig.y()))

        # 2. OccupancyGrid /map
        if self.map_info is not None:
            mi = self.map_info
            opt = self.world_to_screen(mi["origin_x"], mi["origin_y"])
            p.save()
            p.translate(opt)
            p.rotate(-math.degrees(mi["origin_yaw"]))
            mw_px = mi["width"] * mi["resolution"] * self.px_per_m
            mh_px = mi["height"] * mi["resolution"] * self.px_per_m
            p.drawImage(QRectF(0.0, -mh_px, mw_px, mh_px), mi["image"])
            p.restore()
        else:
            # Offline / pre-map metric range rings
            center_w = (self.pose["x"], self.pose["y"]) if self.pose else (0.0, 0.0)
            c_pt = self.world_to_screen(*center_w)
            p.setBrush(Qt.BrushStyle.NoBrush)
            for radius_m in (2.0, 5.0, 10.0):
                r_px = radius_m * self.px_per_m
                p.setPen(QPen(QColor("#26222c"), 1, Qt.PenStyle.DashLine))
                p.drawEllipse(c_pt, r_px, r_px)
                p.setFont(mono_font(9))
                p.setPen(QColor("#5b697d"))
                p.drawText(QPointF(c_pt.x() + 4, c_pt.y() - r_px - 3), f"{radius_m:.0f}m")

        # 3. Nav2 Global Plan (/plan)
        if len(self.plan) >= 2:
            p.setPen(QPen(QColor("#efb16a"), 2.5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            pts = [self.world_to_screen(x, y) for x, y in self.plan]
            for a, b in zip(pts[:-1], pts[1:]):
                p.drawLine(a, b)

        # 4. Odometry breadcrumb trail
        if len(self.trail) >= 2:
            p.setPen(QPen(QColor("#a64c4d"), 1.6))
            pts = [self.world_to_screen(x, y) for x, y in self.trail]
            for a, b in zip(pts[:-1], pts[1:]):
                p.drawLine(a, b)

        # 5. 3D Obstacles (/perception/obstacles_only)
        for det in self.obstacles:
            wx, wy = self._to_world_xy(det["x"], det["y"], det.get("frame_id", ""))
            pt = self.world_to_screen(wx, wy)
            bw = max(6.0, det["sx"] * self.px_per_m)
            bh = max(6.0, det["sy"] * self.px_per_m)
            p.setPen(QPen(QColor("#ef4444"), 1.5))
            p.setBrush(QColor(239, 68, 68, 70))
            p.drawRect(QRectF(pt.x() - bw * 0.5, pt.y() - bh * 0.5, bw, bh))

        # 6. ArUco Landmark poses (/perception/aruco_pose)
        for i, mk in enumerate(self.aruco_markers):
            pt = self.world_to_screen(mk["wx"], mk["wy"])
            s = 8.0
            poly = QPolygonF([
                QPointF(pt.x(), pt.y() - s),
                QPointF(pt.x() + s, pt.y()),
                QPointF(pt.x(), pt.y() + s),
                QPointF(pt.x() - s, pt.y()),
            ])
            p.setPen(QPen(QColor("#2dd4bf"), 2))
            p.setBrush(QColor(45, 212, 191, 95))
            p.drawPolygon(poly)
            p.setFont(mono_font(10, True))
            p.setPen(QColor("#99f6e4"))
            p.drawText(QPointF(pt.x() + 10, pt.y() + 4), f"ArUco #{i + 1}")

        # 7. Mission Waypoints queue
        if len(self.waypoints) >= 2:
            p.setPen(QPen(QColor("#60a5fa"), 1.4, Qt.PenStyle.DashLine))
            wpts = [self.world_to_screen(wp["x"], wp["y"]) for wp in self.waypoints]
            for a, b in zip(wpts[:-1], wpts[1:]):
                p.drawLine(a, b)
        for idx, wp in enumerate(self.waypoints):
            pt = self.world_to_screen(wp["x"], wp["y"])
            active = idx == self.active_wp_index
            col = QColor("#fbbf24" if active else "#60a5fa")
            yaw = math.radians(wp["yaw_deg"])
            tip = QPointF(pt.x() + 18 * math.cos(yaw), pt.y() - 18 * math.sin(yaw))
            p.setPen(QPen(col, 2))
            p.drawLine(pt, tip)
            p.setBrush(QColor("#1a2232"))
            p.drawEllipse(pt, 9, 9)
            p.setFont(mono_font(9, True))
            p.setPen(col)
            p.drawText(QRectF(pt.x() - 9, pt.y() - 9, 18, 18), Qt.AlignmentFlag.AlignCenter, str(idx + 1))

        # Active single Nav2 goal marker
        if self.active_goal is not None:
            gx, gy, gyaw = self.active_goal
            gpt = self.world_to_screen(gx, gy)
            tip = QPointF(gpt.x() + 26 * math.cos(gyaw), gpt.y() - 26 * math.sin(gyaw))
            p.setPen(QPen(QColor("#4ade80"), 2.5))
            p.drawLine(gpt, tip)
            p.setBrush(QColor(74, 222, 128, 70))
            p.drawEllipse(gpt, 8, 8)

        # 8. Live Rover footprint
        if self.pose is not None:
            rx, ry = self.pose["x"], self.pose["y"]
            rpt = self.world_to_screen(rx, ry)
            pulse = 0.5 + 0.5 * math.sin(self._phase * 4.0)
            p.setPen(QPen(QColor(243, 118, 105, int(60 + 90 * pulse)), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(rpt, 18 + 6 * pulse, 18 + 6 * pulse)

            p.save()
            p.translate(rpt)
            # World +Y is up, screen +Y is down -> negate yaw for Qt rotation.
            p.rotate(-self.pose["yaw_deg"])
            L = max(18.0, DEFAULT_WHEELBASE_M * self.px_per_m)
            W = max(14.0, DEFAULT_TRACK_WIDTH_M * self.px_per_m)
            p.setPen(QPen(QColor(ACCENT), 2))
            p.setBrush(QColor(243, 118, 105, 85))
            p.drawRoundedRect(QRectF(-L * 0.5, -W * 0.5, L, W), 4, 4)
            # Forward nose triangle (+X in body frame)
            p.setBrush(QColor("#fde68a"))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawPolygon(QPolygonF([
                QPointF(L * 0.5 + 7, 0),
                QPointF(L * 0.15, -W * 0.28),
                QPointF(L * 0.15, W * 0.28),
            ]))
            p.restore()

        # 9. Interactive drag arrow
        if self._goal_start_world is not None and self._goal_cur_world is not None:
            sp = self.world_to_screen(*self._goal_start_world)
            ep = self.world_to_screen(*self._goal_cur_world)
            col = QColor("#fbbf24" if self._drag_mode == TOOL_WAYPOINT else "#4ade80")
            p.setPen(QPen(col, 2.5, Qt.PenStyle.DashLine))
            p.drawLine(sp, ep)
            p.setBrush(col)
            p.drawEllipse(sp, 6, 6)

        # 10. Crisp HUD Overlay
        hud_rect = QRectF(10, 10, 320, 74)
        p.setPen(QPen(QColor("#3c2a34"), 1))
        p.setBrush(QColor(11, 15, 23, 210))
        p.drawRoundedRect(hud_rect, 8, 8)
        p.setFont(mono_font(10, True))
        p.setPen(QColor(ACCENT))
        map_badge = "LIVE /map" if self.map_info else "OFFLINE / GRID"
        p.drawText(QPointF(20, 28), f"{map_badge} · {self.px_per_m:.0f} px/m")
        p.setFont(mono_font(10))
        p.setPen(QColor("#cbd5e1"))
        if self.pose is not None:
            p.drawText(
                QPointF(20, 47),
                f"Pose X:{self.pose['x']:+.2f}m  Y:{self.pose['y']:+.2f}m  Yaw:{self.pose['yaw_deg']:+.1f}°",
            )
            goal_txt = f"Speed:{self.pose['speed_mps']:.2f} m/s"
            if self.active_goal is not None:
                dist = math.hypot(self.active_goal[0] - self.pose["x"], self.active_goal[1] - self.pose["y"])
                goal_txt += f"  ·  Goal:{dist:.2f}m"
            p.drawText(QPointF(20, 66), goal_txt)
        else:
            p.drawText(QPointF(20, 47), "Pose: awaiting /odometry/filtered")
            cursor_txt = (
                f"Cursor X:{self._cursor_world[0]:+.2f}m  Y:{self._cursor_world[1]:+.2f}m"
                if self._cursor_world
                else "Wheel: zoom · Drag: pan"
            )
            p.drawText(QPointF(20, 66), cursor_txt)


class RoverMapPanel(QWidget):
    """Combined 2D SLAM/Nav2 Map Canvas, Waypoint Mission Runner, and Perception Panel."""

    goal_requested = Signal(float, float, float)      # x, y, yaw_rad
    mission_cancelled = Signal()
    log_message = Signal(str, str)                    # category, message

    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints: list[dict] = []
        self.active_wp_index: int | None = None
        self.mission_running = False
        self.goal_allowed = False
        self.last_aruco: dict | None = None
        self.last_pose: dict | None = None
        self.last_obstacles: list[dict] = []

        root = QHBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(10)

        # Left/main canvas + toolbar
        left = QVBoxLayout()
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        self.tool_buttons = {}
        for key, label in ((TOOL_PAN, "Pan"), (TOOL_WAYPOINT, "+ Waypoint"), (TOOL_GOAL, "2D Goal")):
            btn = QPushButton(label)
            btn.setObjectName("mapBtn")
            btn.setCheckable(True)
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            btn.clicked.connect(lambda _=False, k=key: self.select_tool(k))
            self.tool_buttons[key] = btn
            toolbar.addWidget(btn)
        self.tool_buttons[TOOL_PAN].setChecked(True)

        self.follow_btn = QPushButton("Follow")
        self.follow_btn.setObjectName("mapBtn")
        self.follow_btn.setCheckable(True)
        self.follow_btn.setChecked(True)
        self.follow_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.follow_btn.toggled.connect(self._on_follow_toggled)
        toolbar.addWidget(self.follow_btn)

        fit_btn = QPushButton("Fit")
        fit_btn.setObjectName("mapBtn")
        fit_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        fit_btn.clicked.connect(lambda: self.canvas.fit_view())
        toolbar.addWidget(fit_btn)

        clear_trail_btn = QPushButton("Clear Trail")
        clear_trail_btn.setObjectName("mapBtn")
        clear_trail_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        clear_trail_btn.clicked.connect(lambda: self.canvas.clear_trail())
        toolbar.addWidget(clear_trail_btn)
        toolbar.addStretch()
        left.addLayout(toolbar)

        self.canvas = MapCanvas(self)
        self.canvas.waypoint_added.connect(self.add_waypoint)
        self.canvas.goal_requested.connect(self._on_canvas_goal)
        left.addWidget(self.canvas, 1)
        root.addLayout(left, 1)

        # Right sidebar: Waypoints + Perception
        side = QWidget()
        side.setFixedWidth(295)
        side_lay = QVBoxLayout(side)
        side_lay.setContentsMargins(0, 0, 0, 0)
        side_lay.setSpacing(8)

        self.gate_label = QLabel("ARM + ROS Teleop required to send Nav2 goals.")
        self.gate_label.setWordWrap(True)
        self.gate_label.setStyleSheet("color:#fbbf24; font-weight:600;")
        side_lay.addWidget(self.gate_label)

        wp_group = QGroupBox("Mission waypoints")
        wp_lay = QVBoxLayout(wp_group)
        self.wp_table = QTableWidget(0, 5)
        self.wp_table.setHorizontalHeaderLabels(["#", "X", "Y", "Yaw", "Dist"])
        self.wp_table.verticalHeader().setVisible(False)
        self.wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.wp_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.wp_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        wp_lay.addWidget(self.wp_table, 1)

        row1 = QHBoxLayout()
        row1.setSpacing(6)
        self.send_next_btn = QPushButton("Send Next")
        self.send_next_btn.setObjectName("mapBtn")
        self.send_next_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.send_next_btn.clicked.connect(self.send_next_waypoint)
        self.run_queue_btn = QPushButton("Run Queue")
        self.run_queue_btn.setObjectName("mapBtn")
        self.run_queue_btn.setCheckable(True)
        self.run_queue_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.run_queue_btn.toggled.connect(self.toggle_mission_queue)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setObjectName("mapBtn")
        self.cancel_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.cancel_btn.clicked.connect(self.cancel_mission)
        for b in (self.send_next_btn, self.run_queue_btn, self.cancel_btn):
            row1.addWidget(b)
        wp_lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(6)
        for text, cb in (
            ("Del", self.remove_selected_waypoint),
            ("Clear", self.clear_waypoints),
            ("Save", self.save_waypoints_dialog),
            ("Load", self.load_waypoints_dialog),
        ):
            btn = QPushButton(text)
            btn.setObjectName("mapBtn")
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            btn.clicked.connect(cb)
            row2.addWidget(btn)
        wp_lay.addLayout(row2)
        side_lay.addWidget(wp_group, 1)

        perc_group = QGroupBox("Perception detections")
        perc_lay = QVBoxLayout(perc_group)
        self.aruco_label = QLabel("ArUco landmark: none")
        self.aruco_label.setWordWrap(True)
        self.obstacle_label = QLabel("3D obstacles: 0 active")
        self.obstacle_label.setWordWrap(True)
        perc_lay.addWidget(self.aruco_label)
        perc_lay.addWidget(self.obstacle_label)
        side_lay.addWidget(perc_group)

        root.addWidget(side)
        self.set_goal_allowed(False)

    def select_tool(self, tool: str):
        for key, btn in self.tool_buttons.items():
            btn.setChecked(key == tool)
        self.canvas.set_tool(tool)

    def _on_follow_toggled(self, checked: bool):
        self.canvas.set_follow(checked)

    def set_goal_allowed(self, allowed: bool):
        self.goal_allowed = bool(allowed)
        self.canvas.goal_allowed = self.goal_allowed
        self.send_next_btn.setEnabled(self.goal_allowed)
        self.run_queue_btn.setEnabled(self.goal_allowed)
        if self.goal_allowed:
            self.gate_label.setText("ARMED · Ready to send /goal_pose to Nav2.")
            self.gate_label.setStyleSheet("color:#4ade80; font-weight:600;")
        else:
            if self.mission_running:
                self.cancel_mission()
            self.gate_label.setText("ARM + ROS Teleop required to send Nav2 goals.")
            self.gate_label.setStyleSheet("color:#fbbf24; font-weight:600;")

    def _on_canvas_goal(self, x: float, y: float, yaw_rad: float):
        if not self.goal_allowed:
            self.canvas.active_goal = None
            self.log_message.emit("WARN", "Blocked 2D Nav2 goal: switch to ROS 2 Teleop, connect ROS, and ARM first.")
            return
        self.goal_requested.emit(x, y, yaw_rad)
        self.log_message.emit("ROS", f"Sent Nav2 goal X:{x:+.2f}m Y:{y:+.2f}m Yaw:{math.degrees(yaw_rad):+.1f}°")

    def add_waypoint(self, x: float, y: float, yaw_deg: float):
        self.waypoints.append({"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw_deg, 1)})
        self._refresh_waypoints()
        self.log_message.emit("INFO", f"Added waypoint #{len(self.waypoints)} ({x:+.2f}, {y:+.2f}, {yaw_deg:+.0f}°)")

    def remove_selected_waypoint(self):
        row = self.wp_table.currentRow()
        if 0 <= row < len(self.waypoints):
            self.waypoints.pop(row)
            if self.active_wp_index is not None:
                if row == self.active_wp_index:
                    self.active_wp_index = None
                elif row < self.active_wp_index:
                    self.active_wp_index -= 1
            self._refresh_waypoints()

    def clear_waypoints(self):
        self.waypoints.clear()
        self.active_wp_index = None
        self.mission_running = False
        self.run_queue_btn.blockSignals(True)
        self.run_queue_btn.setChecked(False)
        self.run_queue_btn.blockSignals(False)
        self.canvas.active_goal = None
        self._refresh_waypoints()

    def _dispatch_waypoint(self, idx: int):
        wp = self.waypoints[idx]
        self.active_wp_index = idx
        yaw_rad = math.radians(wp["yaw_deg"])
        self.canvas.active_goal = (wp["x"], wp["y"], yaw_rad)
        self.goal_requested.emit(wp["x"], wp["y"], yaw_rad)
        self._refresh_waypoints()
        self.log_message.emit("ROS", f"Dispatched waypoint #{idx + 1}: X:{wp['x']:+.2f}m Y:{wp['y']:+.2f}m")

    def send_next_waypoint(self):
        if not self.goal_allowed or not self.waypoints:
            return
        nxt = 0 if self.active_wp_index is None else (self.active_wp_index + 1)
        if nxt >= len(self.waypoints):
            nxt = 0
        self._dispatch_waypoint(nxt)

    def toggle_mission_queue(self, running: bool):
        if running and (not self.goal_allowed or not self.waypoints):
            self.run_queue_btn.blockSignals(True)
            self.run_queue_btn.setChecked(False)
            self.run_queue_btn.blockSignals(False)
            self.mission_running = False
            return
        self.mission_running = running
        if running:
            idx = 0 if self.active_wp_index is None else self.active_wp_index
            self._dispatch_waypoint(idx)
            self.log_message.emit("INFO", f"Mission queue started ({len(self.waypoints)} waypoints).")
        else:
            self.log_message.emit("INFO", "Mission queue paused.")

    def cancel_mission(self):
        self.mission_running = False
        self.active_wp_index = None
        self.canvas.active_goal = None
        self.run_queue_btn.blockSignals(True)
        self.run_queue_btn.setChecked(False)
        self.run_queue_btn.blockSignals(False)
        self._refresh_waypoints()
        self.mission_cancelled.emit()

    def _refresh_waypoints(self):
        self.wp_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            dist_txt = "--"
            if self.last_pose is not None:
                d = math.hypot(wp["x"] - self.last_pose["x"], wp["y"] - self.last_pose["y"])
                dist_txt = f"{d:.2f}m"
            marker = f"▶{i + 1}" if i == self.active_wp_index else str(i + 1)
            for col, text in enumerate((marker, f"{wp['x']:.2f}", f"{wp['y']:.2f}", f"{wp['yaw_deg']:.0f}°", dist_txt)):
                self.wp_table.setItem(i, col, QTableWidgetItem(text))
        self.canvas.set_waypoints(self.waypoints, self.active_wp_index)

    def save_waypoints_file(self, path: str) -> None:
        Path(path).write_text(json.dumps({"waypoints": self.waypoints}, indent=2), encoding="utf-8")

    def load_waypoints_file(self, path: str) -> int:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        loaded = []
        for item in data.get("waypoints", ()):
            loaded.append({
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw_deg": float(item.get("yaw_deg", 0.0)),
            })
        self.waypoints = loaded
        self.active_wp_index = None
        self._refresh_waypoints()
        return len(loaded)

    def save_waypoints_dialog(self):
        if not self.waypoints:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save waypoint mission", str(Path.home() / "mars_mission.json"), "JSON (*.json)")
        if path:
            try:
                self.save_waypoints_file(path)
                self.log_message.emit("INFO", f"Saved {len(self.waypoints)} waypoints to {Path(path).name}.")
            except OSError as error:
                self.log_message.emit("WARN", f"Cannot save mission: {error}")

    def load_waypoints_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load waypoint mission", str(Path.home()), "JSON (*.json)")
        if path:
            try:
                count = self.load_waypoints_file(path)
                self.log_message.emit("INFO", f"Loaded {count} waypoints from {Path(path).name}.")
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
                self.log_message.emit("WARN", f"Invalid mission file: {error}")

    def set_map(self, info: dict):
        self.canvas.set_map(info)

    def set_odometry(self, odom: dict):
        self.last_pose = dict(odom)
        self.canvas.set_odometry(odom)
        self._refresh_waypoints()

        # Check auto-advance for active waypoint mission within 0.45m tolerance.
        if self.mission_running and self.goal_allowed and self.active_wp_index is not None:
            if 0 <= self.active_wp_index < len(self.waypoints):
                wp = self.waypoints[self.active_wp_index]
                dist = math.hypot(wp["x"] - odom["x"], wp["y"] - odom["y"])
                if dist <= 0.45:
                    nxt = self.active_wp_index + 1
                    if nxt < len(self.waypoints):
                        self.log_message.emit("ROS", f"Waypoint #{self.active_wp_index + 1} reached; advancing.")
                        self._dispatch_waypoint(nxt)
                    else:
                        self.log_message.emit("ROS", "Mission queue complete — final waypoint reached.")
                        self.cancel_mission()

    def set_path(self, pts: list[tuple[float, float]]):
        self.canvas.set_path(pts)

    def set_aruco(self, pose: dict):
        self.last_aruco = dict(pose)
        self.canvas.set_aruco(pose)
        dist = math.hypot(pose["x"], pose["y"])
        if self.last_pose is not None and not ("base_link" in pose.get("frame_id", "").lower() or "camera" in pose.get("frame_id", "").lower()):
            dist = math.hypot(pose["x"] - self.last_pose["x"], pose["y"] - self.last_pose["y"])
        self.aruco_label.setText(
            f"ArUco landmark: X:{pose['x']:+.2f}m Y:{pose['y']:+.2f}m ({dist:.2f}m) · {len(self.canvas.aruco_markers)} logged"
        )

    def set_obstacles(self, dets: list[dict]):
        self.last_obstacles = list(dets)
        self.canvas.set_obstacles(dets)
        if not dets:
            self.obstacle_label.setText("3D obstacles: 0 active")
            return
        nearest = min(math.hypot(d["x"], d["y"]) for d in dets)
        self.obstacle_label.setText(f"3D obstacles: {len(dets)} active (nearest {nearest:.2f}m)")

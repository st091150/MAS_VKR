#!/usr/bin/env python3
"""Навигация по локальным XY-точкам с маркерами в RViz."""

from __future__ import annotations

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from navigation_module.config.sim_settings import SimWaypointSettings
from navigation_module.core.navigation import distance_xy
from navigation_module.core.sim_waypoint import (
    drive_twist,
    needs_course_realign,
    reached_waypoint,
    segment_metrics,
    turn_twist,
)
from navigation_module.mission.mission_frame import load_mission_frame
from navigation_module.mission.waypoint_loader import LoadedWaypoint, load_waypoints
from navigation_ros.sensor_pose import GpsImuPoseEstimator
from navigation_ros.waypoint_palette import rgb_for_index


def _pt(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _as_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _rgb_mulclamp(r: float, g: float, b: float, mul: float) -> tuple[float, float, float]:
    """Линейное усиление RGB с насыщением в 1.0 (для «ярче в N раз» на экране)."""
    return min(1.0, r * mul), min(1.0, g * mul), min(1.0, b * mul)


def _quat_wz_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Поворот вокруг Z: ось +X маркера направлена по yaw (плоскость XY)."""
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


class WaypointNavigator(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_nav")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # Контракт с реальным роботом: поза из GPS + IMU (в симе — мост из Gazebo).
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("gps_filter_alpha", 0.15)
        self.declare_parameter("yaw_noise_stddev_rad", 0.02)
        self.declare_parameter("min_gps_fix_status", 0)
        self.declare_parameter("gps_timeout_sec", 2.0)
        self.declare_parameter("imu_timeout_sec", 1.0)
        self.declare_parameter("marker_topic", "/waypoint_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("turn_kp", 2.2)
        self.declare_parameter("max_turn_cmd", 0.8)
        self.declare_parameter("turn_min_cmd", 0.12)
        self.declare_parameter("max_forward_cmd", 0.35)
        self.declare_parameter("forward_gain", 0.45)
        self.declare_parameter("forward_min_cmd", 0.05)
        self.declare_parameter("yaw_tolerance_rad", 0.05)
        self.declare_parameter("settle_time_sec", 0.25)
        self.declare_parameter("settle_timeout_sec", 1.5)
        self.declare_parameter("settle_use_gps_velocity", False)
        self.declare_parameter("settle_w_epsilon_rps", 0.08)
        self.declare_parameter("settle_v_epsilon_mps", 0.05)
        self.declare_parameter("drive_heading_hold", True)
        self.declare_parameter("drive_heading_kp", 2.0)
        self.declare_parameter("drive_heading_max_rps", 0.18)
        self.declare_parameter("slowdown_distance_m", 1.2)
        self.declare_parameter("slowdown_forward_cmd", 0.25)
        self.declare_parameter("turn_direction_sign", 1.0)
        self.declare_parameter("forward_direction_sign", 1.0)
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("default_reach_radius_m", 0.9)
        self.declare_parameter("overshoot_retry_margin_m", 1.0)
        self.declare_parameter("overshoot_min_travel_m", 1.5)
        self.declare_parameter("overshoot_best_improve_m", 0.12)
        self.declare_parameter("show_trajectory", True)
        self.declare_parameter("trajectory_min_step_m", 0.35)
        self.declare_parameter("heading_realign_rad", 0.35)
        self.declare_parameter("debug_log_hz", 1.0)
        self.declare_parameter("course_check_period_sec", 5.0)

        self._settings = SimWaypointSettings(
            yaw_tolerance_rad=max(0.001, float(self.get_parameter("yaw_tolerance_rad").value)),
            heading_realign_rad=max(
                0.001, float(self.get_parameter("heading_realign_rad").value)
            ),
            turn_kp=max(0.0, float(self.get_parameter("turn_kp").value)),
            turn_min_cmd=max(0.0, float(self.get_parameter("turn_min_cmd").value)),
            max_turn_cmd=max(0.0, float(self.get_parameter("max_turn_cmd").value)),
            max_forward_cmd=max(0.0, float(self.get_parameter("max_forward_cmd").value)),
            forward_gain=max(0.0, float(self.get_parameter("forward_gain").value)),
            forward_min_cmd=max(0.0, float(self.get_parameter("forward_min_cmd").value)),
            drive_heading_hold=bool(self.get_parameter("drive_heading_hold").value),
            drive_heading_kp=max(0.0, float(self.get_parameter("drive_heading_kp").value)),
            drive_heading_max_rps=max(
                0.0, float(self.get_parameter("drive_heading_max_rps").value)
            ),
            slowdown_distance_m=max(0.0, float(self.get_parameter("slowdown_distance_m").value)),
            slowdown_forward_cmd=max(0.0, float(self.get_parameter("slowdown_forward_cmd").value)),
            overshoot_retry_margin_m=max(
                0.02, float(self.get_parameter("overshoot_retry_margin_m").value)
            ),
            turn_direction_sign=(
                1.0 if float(self.get_parameter("turn_direction_sign").value) >= 0.0 else -1.0
            ),
            forward_direction_sign=(
                1.0 if float(self.get_parameter("forward_direction_sign").value) >= 0.0 else -1.0
            ),
            course_check_period_sec=max(
                0.1, float(self.get_parameter("course_check_period_sec").value)
            ),
        )
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value).strip() or "/cmd_vel"
        self._gps_topic = str(self.get_parameter("gps_topic").value).strip() or "/gps/fix"
        self._imu_topic = str(self.get_parameter("imu_topic").value).strip() or "/imu/data"
        self._marker_frame_id = str(self.get_parameter("frame_id").value).strip() or "map"
        gps_filter_alpha = max(0.01, min(1.0, float(self.get_parameter("gps_filter_alpha").value)))
        yaw_noise_stddev_rad = max(0.0, float(self.get_parameter("yaw_noise_stddev_rad").value))
        min_gps_fix_status = int(self.get_parameter("min_gps_fix_status").value)
        self._gps_timeout_sec = max(0.0, float(self.get_parameter("gps_timeout_sec").value))
        self._imu_timeout_sec = max(0.0, float(self.get_parameter("imu_timeout_sec").value))
        self._control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self._yaw_tol = self._settings.yaw_tolerance_rad
        self._settle_sec = max(0.0, float(self.get_parameter("settle_time_sec").value))
        self._settle_timeout = max(
            self._settle_sec, float(self.get_parameter("settle_timeout_sec").value)
        )
        self._settle_use_gps_velocity = bool(
            self.get_parameter("settle_use_gps_velocity").value
        )
        self._settle_w_eps = max(0.0, float(self.get_parameter("settle_w_epsilon_rps").value))
        self._settle_v_eps = max(0.0, float(self.get_parameter("settle_v_epsilon_mps").value))
        self._overshoot_retry_margin = self._settings.overshoot_retry_margin_m
        self._overshoot_min_travel = max(
            0.0, float(self.get_parameter("overshoot_min_travel_m").value)
        )
        self._overshoot_best_improve = max(
            0.0, float(self.get_parameter("overshoot_best_improve_m").value)
        )
        self._trajectory_min_step = max(
            0.01, float(self.get_parameter("trajectory_min_step_m").value)
        )
        self._show_trajectory = bool(self.get_parameter("show_trajectory").value)
        self._default_reach = max(0.05, float(self.get_parameter("default_reach_radius_m").value))
        debug_hz = max(0.0, float(self.get_parameter("debug_log_hz").value))
        self._debug_period = 1.0 / debug_hz if debug_hz > 1e-9 else 0.0
        self._course_check_period = self._settings.course_check_period_sec
        waypoint_file = str(self.get_parameter("waypoint_file").value).strip()
        self._waypoint_file = Path(waypoint_file) if waypoint_file else None

        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub_cmd = self.create_publisher(Twist, self._cmd_topic, cmd_qos)
        self._pub_mark = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), marker_qos
        )
        self.create_subscription(NavSatFix, self._gps_topic, self._gps_cb, sensor_qos)
        self.create_subscription(Imu, self._imu_topic, self._imu_cb, sensor_qos)

        self._pose_est: GpsImuPoseEstimator | None = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v_meas = 0.0
        self._w_meas = 0.0
        self._have_pose = False
        self._robot_marker_yaw = 0.0
        self._waypoints: list[LoadedWaypoint] = []
        self._target_idx = 0
        self._completed = False
        self._warned_no_pose = False
        self._logged_pose_ready = False
        self._gps_filter_alpha = gps_filter_alpha
        self._yaw_noise_stddev_rad = yaw_noise_stddev_rad
        self._min_gps_fix_status = min_gps_fix_status

        self._state = "TURN_INIT"
        self._yaw_goal = 0.0
        self._yaw_hold = 0.0
        self._drive_goal_dist = 0.0
        self._drive_start_x = 0.0
        self._drive_start_y = 0.0
        self._drive_best_dist = float("inf")
        self._drive_best_x = 0.0
        self._drive_best_y = 0.0
        self._settle_deadline = 0.0
        self._settle_timeout_deadline = 0.0
        self._next_debug_log = 0.0
        self._next_cmd_log = 0.0
        self._trajectory: list[tuple[float, float]] = []
        self._last_traj_x: float | None = None
        self._last_traj_y: float | None = None
        self._max_route_arrow_id = 2999

        self._load_waypoints()
        self.get_logger().info(
            f"waypoint_nav: поза из {self._gps_topic} + {self._imu_topic}; "
            f"FSM через navigation_module.sim_waypoint; "
            f"drive_heading_hold={self._settings.drive_heading_hold}; "
            f"проверка курса таймером каждые {self._course_check_period:.1f} с "
            f"(порог {math.degrees(self._settings.heading_realign_rad):.1f}°)"
        )
        if self._marker_frame_id:
            self._publish_markers()
        self.create_timer(1.0 / self._control_hz, self._tick)
        self.create_timer(self._course_check_period, self._periodic_course_check_timer)
        self.create_timer(0.2, self._publish_markers)

    def _resolve_waypoint_file(self) -> Path:
        if self._waypoint_file and self._waypoint_file.is_file():
            return self._waypoint_file
        pkg_share = Path(get_package_share_directory("navigation_ros"))
        p = pkg_share / "config" / "waypoints.json"
        if p.is_file():
            return p
        raise FileNotFoundError(
            "Не найден config/waypoints.json; задайте параметр waypoint_file с полным путём."
        )

    def _load_waypoints(self) -> None:
        path = self._resolve_waypoint_file()
        if path.suffix.lower() != ".json":
            raise RuntimeError(
                f"Ожидается JSON с маршрутом: {path.name}. Конвертируйте маршрут в waypoints.json."
            )
        self._waypoints = load_waypoints(path, default_reach_radius_m=self._default_reach)
        frame = load_mission_frame(path)
        self._pose_est = GpsImuPoseEstimator(
            frame,
            gps_filter_alpha=self._gps_filter_alpha,
            min_gps_fix_status=self._min_gps_fix_status,
            yaw_noise_stddev_rad=self._yaw_noise_stddev_rad,
        )
        self.get_logger().info(f"Загружено точек: {len(self._waypoints)}, файл: {path}")
        for i, wp in enumerate(self._waypoints):
            geo = ""
            if wp.latitude is not None and wp.longitude is not None:
                geo = f" geo=({wp.latitude:.8f},{wp.longitude:.8f})"
            self.get_logger().info(
                f"Точка {i + 1}: {wp.wid} xy=({wp.x:.3f},{wp.y:.3f}) "
                f"радиус={wp.reach_radius_m:.2f}{geo}"
            )
        first = self._waypoints[0]
        self.get_logger().info(
            f"Первая цель: {first.wid} ({first.x:.2f}, {first.y:.2f}), радиус достижения {first.reach_radius_m:.2f} м"
        )

    def _sync_pose_from_estimator(self) -> None:
        if self._pose_est is None:
            return
        self._x = self._pose_est.x
        self._y = self._pose_est.y
        self._yaw = self._pose_est.yaw
        self._robot_marker_yaw = self._yaw
        self._v_meas = self._pose_est.v_meas
        self._w_meas = self._pose_est.w_meas
        self._have_pose = self._pose_est.have_pose

    def _refresh_pose_ready(self, now: float) -> None:
        if self._pose_est is None:
            self._have_pose = False
            return
        self._have_pose = self._pose_est.has_fresh_pose(
            now,
            gps_timeout_sec=self._gps_timeout_sec,
            imu_timeout_sec=self._imu_timeout_sec,
        )

    def _gps_cb(self, msg: NavSatFix) -> None:
        if self._pose_est is None:
            return
        self._pose_est.update_gps(msg)
        self._sync_pose_from_estimator()

    def _imu_cb(self, msg: Imu) -> None:
        if self._pose_est is None:
            return
        self._pose_est.update_imu(msg)
        self._sync_pose_from_estimator()

    def _append_trajectory_point(self) -> None:
        if not self._have_pose:
            return
        if self._last_traj_x is not None and self._last_traj_y is not None:
            if (
                math.hypot(self._x - self._last_traj_x, self._y - self._last_traj_y)
                < self._trajectory_min_step
            ):
                return
        self._trajectory.append((self._x, self._y))
        self._last_traj_x = self._x
        self._last_traj_y = self._y

    def _periodic_course_check_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._periodic_course_check(now)

    def _periodic_course_check(self, sim_t: float) -> None:
        if not self._have_pose or self._completed or self._target_idx >= len(self._waypoints):
            return

        wp = self._waypoints[self._target_idx]
        dist, target_bearing, heading_err = segment_metrics(
            self._x, self._y, self._yaw, wp.x, wp.y
        )

        self.get_logger().info(
            f"КУРС_ПРОВЕРКА t={sim_t:.1f}с цель={wp.wid} ({self._target_idx + 1}/{len(self._waypoints)}) "
            f"поза=({self._x:.2f},{self._y:.2f}) дист={dist:.2f}м "
            f"курс={math.degrees(self._yaw):.1f}° азимут={math.degrees(target_bearing):.1f}° "
            f"ошибка_курса={math.degrees(heading_err):.1f}° состояние={self._state}"
        )

        if self._state != "DRIVING":
            return
        if not needs_course_realign(heading_err, self._settings):
            return

        self._trigger_course_realign(
            sim_t,
            wp,
            heading_err,
            reason="периодическая_проверка",
            threshold_deg=math.degrees(self._settings.heading_realign_rad),
        )

    def _trigger_course_realign(
        self,
        now: float,
        wp: LoadedWaypoint,
        heading_err: float,
        *,
        reason: str,
        threshold_deg: float,
    ) -> None:
        self._publish_stop()
        self._start_settle(now)
        self._state = "SETTLE_AFTER_DRIVE"
        self.get_logger().info(
            f"КУРС_КОРРЕКЦИЯ {wp.wid}: {reason} ошибка_курса={math.degrees(heading_err):.1f}° "
            f"порог={threshold_deg:.1f}° дист={math.hypot(wp.x - self._x, wp.y - self._y):.2f} м; "
            "остановка и повтор той же точки."
        )

    def _publish_stop(self) -> None:
        m = Twist()
        self._pub_cmd.publish(m)

    def _start_settle(self, now: float) -> None:
        self._settle_deadline = now + self._settle_sec
        self._settle_timeout_deadline = now + self._settle_timeout

    def _settled_or_timed_out(self, now: float) -> tuple[bool, str]:
        linear_ok = (not self._settle_use_gps_velocity) or abs(self._v_meas) <= self._settle_v_eps
        settled = (
            now >= self._settle_deadline
            and linear_ok
            and abs(self._w_meas) <= self._settle_w_eps
        )
        if settled:
            return True, "стабилизация"
        if now >= self._settle_timeout_deadline:
            return True, "таймаут_стабилизации"
        return False, "ожидание"

    def _debug_status(
        self,
        now: float,
        wp: LoadedWaypoint,
        dist: float,
        target_bearing: float,
        angle_err: float,
    ) -> None:
        if self._debug_period <= 0.0 or now < self._next_debug_log:
            return
        self._next_debug_log = now + self._debug_period
        traveled = math.hypot(self._x - self._drive_start_x, self._y - self._drive_start_y)
        self.get_logger().info(
            "НАВ "
            f"состояние={self._state} цель={wp.wid} шаг={self._target_idx + 1}/{len(self._waypoints)} "
            f"поза=({self._x:.2f},{self._y:.2f}) курс={math.degrees(self._yaw):.1f}° "
            f"точка=({wp.x:.2f},{wp.y:.2f}) дист={dist:.2f} радиус={wp.reach_radius_m:.2f} "
            f"азимут={math.degrees(target_bearing):.1f}° ошибка_курса={math.degrees(angle_err):.1f}° "
            f"цель_курса={math.degrees(self._yaw_goal):.1f}° удержание={math.degrees(self._yaw_hold):.1f}° "
            f"проезд={traveled:.2f}/{self._drive_goal_dist:.2f}м "
            f"sensor_v={self._v_meas:.3f} sensor_w={self._w_meas:.3f}"
        )

    def _publish_markers(self) -> None:
        if not self._marker_frame_id:
            return
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self._waypoints):
            br, bg, bb = rgb_for_index(i)
            if self._completed or i < self._target_idx:
                # Пройдено: заметнее, чем раньше, но всё ещё слабее «живых» точек.
                color_point = _as_color(
                    min(1.0, br * 0.78 + 0.08),
                    min(1.0, bg * 0.78 + 0.08),
                    min(1.0, bb * 0.78 + 0.08),
                    0.88,
                )
                color_radius = _as_color(br * 0.72, bg * 0.72, bb * 0.72, 0.22)
            elif i == self._target_idx:
                # Текущая цель — примерно в 2 раза ярче базовой палитры (по линейному RGB).
                pr, pg, pb = _rgb_mulclamp(br, bg, bb, 2.0)
                color_point = _as_color(pr, pg, pb, 1.0)
                color_radius = _as_color(pr, pg, pb, 0.72)
            else:
                # Ещё не посещённые — яркие, хорошо видны на карте.
                color_point = _as_color(br, bg, bb, 1.0)
                color_radius = _as_color(
                    min(1.0, br * 0.95 + 0.05),
                    min(1.0, bg * 0.95 + 0.05),
                    min(1.0, bb * 0.95 + 0.05),
                    0.32,
                )

            p = Marker()
            p.header.frame_id = self._marker_frame_id
            p.header.stamp = stamp
            p.ns = "waypoints_point"
            p.id = i
            p.type = Marker.SPHERE
            p.action = Marker.ADD
            p.pose.position.x = wp.x
            p.pose.position.y = wp.y
            p.pose.position.z = 0.15
            p.pose.orientation.w = 1.0
            if i == self._target_idx:
                ps = 0.48
            else:
                ps = 0.35
            p.scale.x = ps
            p.scale.y = ps
            p.scale.z = ps
            p.color = color_point
            arr.markers.append(p)

            r = Marker()
            r.header.frame_id = self._marker_frame_id
            r.header.stamp = stamp
            r.ns = "waypoints_radius"
            r.id = 1000 + i
            r.type = Marker.CYLINDER
            r.action = Marker.ADD
            r.pose.position.x = wp.x
            r.pose.position.y = wp.y
            r.pose.position.z = 0.02
            r.pose.orientation.w = 1.0
            r.scale.x = 2.0 * wp.reach_radius_m
            r.scale.y = 2.0 * wp.reach_radius_m
            r.scale.z = 0.04
            r.color = color_radius
            arr.markers.append(r)

            label = Marker()
            label.header.frame_id = self._marker_frame_id
            label.header.stamp = stamp
            label.ns = "waypoints_label"
            label.id = 2000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = wp.x
            label.pose.position.y = wp.y
            label.pose.position.z = 0.65
            label.pose.orientation.w = 1.0
            label.scale.z = 0.24
            lr, lg, lb = rgb_for_index(i)
            if i == self._target_idx:
                tr, tg, tb = _rgb_mulclamp(lr, lg, lb, 2.0)
                label.color = _as_color(tr, tg, tb, 1.0)
            else:
                label.color = _as_color(
                    min(1.0, lr * 1.05 + 0.05),
                    min(1.0, lg * 1.05 + 0.05),
                    min(1.0, lb * 1.05 + 0.05),
                    1.0,
                )
            label.text = f"{i + 1}: {wp.wid}\nxy=({wp.x:.1f},{wp.y:.1f})"
            arr.markers.append(label)

        # Стрелки маршрута: мелкие красные ARROW вдоль каждого сегмента (как на эталонном RViz).
        route_z = 0.28
        wps = self._waypoints
        arrow_spacing_m = 0.45
        arrow_len = 0.32
        arrow_shaft = 0.07
        arrow_head = 0.14

        # Убрать старые линии, если они остались в RViz от прошлых версий.
        for ns, mid in (
            ("waypoints_route_line", 4000),
            ("robot_target_line", 4001),
        ):
            del_old = Marker()
            del_old.header.frame_id = self._marker_frame_id
            del_old.header.stamp = stamp
            del_old.ns = ns
            del_old.id = mid
            del_old.action = Marker.DELETE
            arr.markers.append(del_old)

        if self._show_trajectory and len(self._trajectory) >= 2:
            traj_line = Marker()
            traj_line.header.frame_id = self._marker_frame_id
            traj_line.header.stamp = stamp
            traj_line.ns = "robot_trajectory_line"
            traj_line.id = 4100
            traj_line.type = Marker.LINE_STRIP
            traj_line.action = Marker.ADD
            traj_line.pose.orientation.w = 1.0
            traj_line.scale.x = 0.04
            traj_line.color = _as_color(0.2, 0.75, 0.95, 0.55)
            traj_line.points = [_pt(x, y, 0.18) for x, y in self._trajectory]
            arr.markers.append(traj_line)
        else:
            hide_traj = Marker()
            hide_traj.header.frame_id = self._marker_frame_id
            hide_traj.header.stamp = stamp
            hide_traj.ns = "robot_trajectory_line"
            hide_traj.id = 4100
            hide_traj.action = Marker.DELETE
            arr.markers.append(hide_traj)

        arrow_id = 3000
        for seg_i in range(len(wps) - 1):
            a, b = wps[seg_i], wps[seg_i + 1]
            dx = float(b.x - a.x)
            dy = float(b.y - a.y)
            length = math.hypot(dx, dy)
            if length < 1e-4:
                continue
            ux, uy = dx / length, dy / length
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = _quat_wz_from_yaw(yaw)

            usable = max(0.05, length - 0.28)
            n_arrows = max(1, int(usable / arrow_spacing_m) + 1)
            for k in range(n_arrows):
                t = (k + 0.5) / n_arrows
                cx = float(a.x) + dx * t
                cy = float(a.y) + dy * t
                tail_x = cx - ux * (arrow_len * 0.5)
                tail_y = cy - uy * (arrow_len * 0.5)

                arrow = Marker()
                arrow.header.frame_id = self._marker_frame_id
                arrow.header.stamp = stamp
                arrow.ns = "waypoints_route_arrow"
                arrow.id = arrow_id
                arrow_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = tail_x
                arrow.pose.position.y = tail_y
                arrow.pose.position.z = route_z
                arrow.pose.orientation.x = qx
                arrow.pose.orientation.y = qy
                arrow.pose.orientation.z = qz
                arrow.pose.orientation.w = qw
                arrow.scale.x = arrow_len
                arrow.scale.y = arrow_shaft
                arrow.scale.z = arrow_head
                arrow.color = _as_color(0.92, 0.12, 0.10, 0.95)
                arr.markers.append(arrow)

        if arrow_id - 1 > self._max_route_arrow_id:
            self._max_route_arrow_id = arrow_id - 1
        elif arrow_id <= self._max_route_arrow_id:
            for stale_id in range(arrow_id, self._max_route_arrow_id + 1):
                hide = Marker()
                hide.header.frame_id = self._marker_frame_id
                hide.header.stamp = stamp
                hide.ns = "waypoints_route_arrow"
                hide.id = stale_id
                hide.action = Marker.DELETE
                arr.markers.append(hide)
            self._max_route_arrow_id = arrow_id - 1

        robot = Marker()
        robot.header.frame_id = self._marker_frame_id
        robot.header.stamp = stamp
        robot.ns = "robot_pose_world"
        robot.id = 5000
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose.position.x = self._x
        robot.pose.position.y = self._y
        robot.pose.position.z = 0.55
        robot.pose.orientation.z = math.sin(self._robot_marker_yaw * 0.5)
        robot.pose.orientation.w = math.cos(self._robot_marker_yaw * 0.5)
        robot.scale.x = 0.8
        robot.scale.y = 0.22
        robot.scale.z = 0.22
        robot.color = _as_color(1.0, 1.0, 1.0, 1.0)
        arr.markers.append(robot)

        robot_label = Marker()
        robot_label.header.frame_id = self._marker_frame_id
        robot_label.header.stamp = stamp
        robot_label.ns = "robot_pose_world"
        robot_label.id = 5001
        robot_label.type = Marker.TEXT_VIEW_FACING
        robot_label.action = Marker.ADD
        robot_label.pose.position.x = self._x
        robot_label.pose.position.y = self._y
        robot_label.pose.position.z = 1.05
        robot_label.pose.orientation.w = 1.0
        robot_label.scale.z = 0.22
        robot_label.color = _as_color(1.0, 1.0, 1.0, 1.0)
        robot_label.text = f"поза GPS+IMU\nxy=({self._x:.2f},{self._y:.2f})"
        arr.markers.append(robot_label)

        self._pub_mark.publish(arr)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._refresh_pose_ready(now)
        if not self._have_pose:
            self._publish_stop()
            if not self._warned_no_pose and self._pose_est is not None:
                self._warned_no_pose = True
                missing: list[str] = []
                if not self._pose_est.have_gps:
                    missing.append(self._gps_topic)
                if not self._pose_est.have_imu:
                    missing.append(self._imu_topic)
                detail = ", ".join(missing) if missing else f"{self._gps_topic}, {self._imu_topic}"
                self.get_logger().warning(
                    f"Нет полной позы GPS+IMU; не получены: {detail}. "
                    "Проверьте мост ros_gz и плагины gz-sim-imu-system / gz-sim-navsat-system в мире."
                )
            return

        if not self._logged_pose_ready:
            self._logged_pose_ready = True
            self.get_logger().info(
                f"Поза готова: xy=({self._x:.2f},{self._y:.2f}) курс={math.degrees(self._yaw):.1f}°"
            )
            self._append_trajectory_point()
        else:
            self._append_trajectory_point()
        if self._completed:
            self._publish_stop()
            return
        if self._target_idx >= len(self._waypoints):
            self._completed = True
            self._publish_stop()
            self._publish_markers()
            self.get_logger().info("Маршрут по точкам завершён.")
            return

        wp = self._waypoints[self._target_idx]
        dist, target_bearing, angle_err = segment_metrics(
            self._x, self._y, self._yaw, wp.x, wp.y
        )
        if self._state == "TURN_INIT" and reached_waypoint(
            self._x, self._y, wp.x, wp.y, wp.reach_radius_m
        ):
            self.get_logger().info(
                f"Достигнута {wp.wid}: дист={dist:.2f} м радиус={wp.reach_radius_m:.2f} м "
                f"({self._target_idx + 1}/{len(self._waypoints)})"
            )
            self._target_idx += 1
            self._publish_markers()
            self._state = "TURN_INIT"
            return

        self._debug_status(now, wp, dist, target_bearing, angle_err)

        if self._state == "TURN_INIT":
            self._yaw_goal = target_bearing
            self._state = "TURNING"
            turn_dir = "влево (против часовой)" if angle_err >= 0.0 else "вправо (по часовой)"
            self.get_logger().info(
                f"ПОВОРОТ_СТАРТ {wp.wid}: робот=({self._x:.2f},{self._y:.2f}) "
                f"цель=({wp.x:.2f},{wp.y:.2f}) дист={dist:.2f} м "
                f"курс={math.degrees(self._yaw):.1f}° цель_курса={math.degrees(self._yaw_goal):.1f}° "
                f"ошибка={math.degrees(angle_err):.1f}° направление={turn_dir}"
            )

        if self._state == "TURNING":
            _, ang, yaw_err = turn_twist(self._yaw_goal, self._yaw, self._settings)
            ay = abs(yaw_err)
            if ay <= self._yaw_tol:
                self._publish_stop()
                self._start_settle(now)
                self._state = "SETTLE_BEFORE_DRIVE"
                self.get_logger().info(
                    f"ПОВОРОТ_ГОТОВ {wp.wid}: курс={math.degrees(self._yaw):.1f}° "
                    f"цель={math.degrees(self._yaw_goal):.1f}° ошибка={math.degrees(yaw_err):.1f}°"
                )
            else:
                m = Twist()
                m.linear.x = 0.0
                m.angular.z = float(ang)
                self._pub_cmd.publish(m)
                if self._debug_period > 0.0 and now >= self._next_cmd_log:
                    self._next_cmd_log = now + self._debug_period
                    self.get_logger().info(
                        f"ПОВОРОТ_CMD {wp.wid}: ошибка_курса={math.degrees(yaw_err):.1f}° "
                        f"cmd_w={m.angular.z:.3f} sensor_w={self._w_meas:.3f}"
                    )
            return

        if self._state == "SETTLE_BEFORE_DRIVE":
            self._publish_stop()
            ready, reason = self._settled_or_timed_out(now)
            if ready:
                self._yaw_hold = self._yaw
                self._drive_goal_dist = dist
                self._drive_start_x = self._x
                self._drive_start_y = self._y
                self._drive_best_dist = dist
                self._drive_best_x = self._x
                self._drive_best_y = self._y
                self._state = "DRIVING"
                self.get_logger().info(
                    f"ПРЯМОЙ_СТАРТ {wp.wid}: причина={reason} старт=({self._drive_start_x:.2f},{self._drive_start_y:.2f}) "
                    f"план_дист={self._drive_goal_dist:.2f} м удержание_курса={math.degrees(self._yaw_hold):.1f}° "
                    f"остаток_до_точки={dist:.2f} м"
                )
            return

        if self._state == "DRIVING":
            traveled = distance_xy(
                self._drive_start_x, self._drive_start_y, self._x, self._y
            )
            if dist + self._overshoot_best_improve < self._drive_best_dist:
                self._drive_best_dist = dist
                self._drive_best_x = self._x
                self._drive_best_y = self._y

            _, _, live_heading_err = segment_metrics(
                self._x, self._y, self._yaw, wp.x, wp.y
            )
            if (
                traveled > 0.5
                and dist > wp.reach_radius_m
                and needs_course_realign(live_heading_err, self._settings)
            ):
                self._trigger_course_realign(
                    now,
                    wp,
                    live_heading_err,
                    reason="перекурс_на_прямой",
                    threshold_deg=math.degrees(self._settings.heading_realign_rad),
                )
                return

            if (
                traveled >= self._overshoot_min_travel
                and self._drive_best_dist < float("inf")
                and dist > self._drive_best_dist + self._overshoot_retry_margin
            ):
                self._publish_stop()
                self._start_settle(now)
                self._state = "SETTLE_AFTER_DRIVE"
                self.get_logger().info(
                    f"ПРЯМОЙ_ПРОЛЁТ {wp.wid}: пройдено={traveled:.2f} м план={self._drive_goal_dist:.2f} м "
                    f"робот=({self._x:.2f},{self._y:.2f}) дист_до_точки={dist:.2f} м "
                    f"лучшая_дист={self._drive_best_dist:.2f} м лучшая_поза=({self._drive_best_x:.2f},{self._drive_best_y:.2f})"
                )
                return

            if traveled >= self._drive_goal_dist:
                self._publish_stop()
                self._start_settle(now)
                self._state = "SETTLE_AFTER_DRIVE"
                self.get_logger().info(
                    f"ПРЯМОЙ_ДИСТ_ГОТОВ {wp.wid}: пройдено={traveled:.2f} м "
                    f"план={self._drive_goal_dist:.2f} м робот=({self._x:.2f},{self._y:.2f}) "
                    f"дист_до_точки={dist:.2f} м радиус={wp.reach_radius_m:.2f} м "
                    f"лучшая_дист={self._drive_best_dist:.2f} м лучшая_поза=({self._drive_best_x:.2f},{self._drive_best_y:.2f})"
                )
                return

            lin, ang = drive_twist(dist, self._yaw, self._yaw_hold, self._settings)
            m = Twist()
            m.linear.x = float(lin)
            m.angular.z = float(ang)
            self._pub_cmd.publish(m)
            if self._debug_period > 0.0 and now >= self._next_cmd_log:
                self._next_cmd_log = now + self._debug_period
                self.get_logger().info(
                    f"ПРЯМОЙ_CMD {wp.wid}: cmd_v={m.linear.x:.3f} cmd_w={m.angular.z:.3f} "
                    f"sensor_v={self._v_meas:.3f} sensor_w={self._w_meas:.3f}"
                )
            return

        if self._state == "SETTLE_AFTER_DRIVE":
            self._publish_stop()
            ready, reason = self._settled_or_timed_out(now)
            if ready:
                if reached_waypoint(self._x, self._y, wp.x, wp.y, wp.reach_radius_m):
                    self.get_logger().info(
                        f"ПРОВЕРКА_ДОСТИГНУТО {wp.wid}: причина={reason} дист={dist:.2f} м "
                        f"радиус={wp.reach_radius_m:.2f} м ({self._target_idx + 1}/{len(self._waypoints)})"
                    )
                    self._target_idx += 1
                    self._publish_markers()
                else:
                    self.get_logger().info(
                        f"ПРОВЕРКА_МИМО {wp.wid}: причина={reason} дист={dist:.2f} м "
                        f"радиус={wp.reach_radius_m:.2f} м "
                        f"лучшая_дист={self._drive_best_dist:.2f} м лучшая_поза=({self._drive_best_x:.2f},{self._drive_best_y:.2f}) "
                        f"робот=({self._x:.2f},{self._y:.2f}); повтор той же точки."
                    )
                self._state = "TURN_INIT"
            return


def main() -> None:
    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

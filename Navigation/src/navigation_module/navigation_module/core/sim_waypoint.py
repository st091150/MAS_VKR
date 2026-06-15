"""Дискретная навигация по XY-waypoints: FSM опирается на generate_command из core/navigation."""

from __future__ import annotations

import math
from typing import Any

from navigation_module.config.sim_settings import SimWaypointSettings
from navigation_module.core.geometry import clamp, normalize_angle
from navigation_module.core.navigation import (
    angle_error,
    bearing_xy,
    distance_xy,
    generate_command,
    reached_target_xy,
)


def segment_metrics(
    x: float, y: float, yaw: float, target_x: float, target_y: float
) -> tuple[float, float, float]:
    """Дистанция, пеленг и ошибка курса до целевой точки."""
    dist = distance_xy(x, y, target_x, target_y)
    bearing = bearing_xy(x, y, target_x, target_y)
    err = angle_error(yaw, bearing)
    return dist, bearing, err


def high_level_command(
    dist_m: float,
    angle_err_rad: float,
    settings: SimWaypointSettings,
    reach_radius_m: float,
) -> dict[str, Any]:
    """
    Верхнеуровневая команда через navigation_module.generate_command.

    Порог поворота — heading_realign_rad; порог «движения» — радиус достижения точки.
    """
    return generate_command(
        dist_m,
        angle_err_rad,
        settings.heading_realign_rad,
        reach_radius_m,
    )


def needs_course_realign(angle_err_rad: float, settings: SimWaypointSettings) -> bool:
    """Сильное отклонение от курса — нужна остановка и повторный разворот."""
    return high_level_command(1.0, angle_err_rad, settings, 0.0)["type"] == "turn"


def reached_waypoint(
    x: float, y: float, target_x: float, target_y: float, radius_m: float
) -> bool:
    return reached_target_xy(x, y, target_x, target_y, radius_m)


def turn_twist(yaw_goal: float, yaw: float, settings: SimWaypointSettings) -> tuple[float, float, float]:
    """linear.x, angular.z (м/с, рад/с), yaw_err."""
    yaw_err = normalize_angle(yaw_goal - yaw)
    ang = clamp(settings.turn_kp * yaw_err, -settings.max_turn_cmd, settings.max_turn_cmd)
    if 0.0 < abs(ang) < settings.turn_min_cmd:
        ang = math.copysign(settings.turn_min_cmd, ang)
    lin = 0.0
    return (
        lin * settings.forward_direction_sign,
        ang * settings.turn_direction_sign,
        yaw_err,
    )


def drive_twist(
    dist: float,
    yaw: float,
    yaw_hold: float,
    settings: SimWaypointSettings,
) -> tuple[float, float]:
    """linear.x, angular.z на прямом участке."""
    lin = settings.forward_gain * dist
    lin = max(settings.forward_min_cmd, min(settings.max_forward_cmd, lin))
    if dist < settings.slowdown_distance_m:
        lin = min(lin, settings.slowdown_forward_cmd)
    if dist < 0.45:
        lin = min(lin, max(settings.forward_min_cmd, 0.12))

    ang = 0.0
    if settings.drive_heading_hold and settings.drive_heading_kp > 1e-9:
        hold_err = normalize_angle(yaw_hold - yaw)
        ang = clamp(
            settings.drive_heading_kp * hold_err,
            -settings.drive_heading_max_rps,
            settings.drive_heading_max_rps,
        )
    return lin * settings.forward_direction_sign, ang * settings.turn_direction_sign

"""Параметры дискретной навигации по XY-waypoints в симуляции."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SimWaypointSettings:
    """Настройки FSM «поворот → прямой» для XY-позы робота."""

    yaw_tolerance_rad: float = 0.05
    heading_realign_rad: float = 0.35
    turn_kp: float = 2.2
    turn_min_cmd: float = 0.12
    max_turn_cmd: float = 0.8
    max_forward_cmd: float = 0.35
    forward_gain: float = 0.45
    forward_min_cmd: float = 0.05
    drive_heading_hold: bool = True
    drive_heading_kp: float = 2.0
    drive_heading_max_rps: float = 0.18
    slowdown_distance_m: float = 1.2
    slowdown_forward_cmd: float = 0.25
    overshoot_retry_margin_m: float = 0.18
    turn_direction_sign: float = 1.0
    forward_direction_sign: float = 1.0
    course_check_period_sec: float = 5.0

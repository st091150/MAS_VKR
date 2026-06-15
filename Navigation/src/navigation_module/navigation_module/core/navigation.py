"""Примитивы навигации: расстояние, пеленг, ошибки, управление, команды."""

from __future__ import annotations

import math
from typing import Any, Dict

from navigation_module.core.geometry import normalize_angle
from navigation_module.core.gps_utils import bearing_between_points, gps_distance_meters


def distance_to_target(
    current_gps: tuple[float, float], target_gps: tuple[float, float]
) -> float:
    """Расстояние по гаверсинусу от текущих (lat, lon) в градусах до цели."""
    lat_c, lon_c = current_gps
    lat_t, lon_t = target_gps
    return gps_distance_meters(lat_c, lon_c, lat_t, lon_t)


def bearing_to_target(
    current_gps: tuple[float, float], target_gps: tuple[float, float]
) -> float:
    """Пеленг от текущей точки к цели (радианы; тот же кадр, что и локальный atan2)."""
    lat_c, lon_c = current_gps
    lat_t, lon_t = target_gps
    return normalize_angle(bearing_between_points(lat_c, lon_c, lat_t, lon_t))


def angle_error(current_yaw_rad: float, target_bearing_rad: float) -> float:
    """Нормализованная ошибка руления через atan2(sin, cos)."""
    return normalize_angle(target_bearing_rad - current_yaw_rad)


def distance_xy(x0: float, y0: float, x1: float, y1: float) -> float:
    """Расстояние между двумя точками в локальной плоскости (м)."""
    return math.hypot(x1 - x0, y1 - y0)


def bearing_xy(x0: float, y0: float, x1: float, y1: float) -> float:
    """Пеленг от (x0,y0) к (x1,y1) в радианах (0 = +X, как atan2(dy, dx))."""
    return math.atan2(y1 - y0, x1 - x0)


def reached_target_xy(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    radius_m: float,
) -> bool:
    """True, если расстояние в плоскости XY не больше допуска."""
    return distance_xy(x0, y0, x1, y1) <= radius_m


def reached_target(
    current_gps: tuple[float, float],
    target_gps: tuple[float, float],
    radius: float = 1.0,
) -> bool:
    """True, если расстояние до цели не больше допуска (метры)."""
    return distance_to_target(current_gps, target_gps) <= radius


def generate_command(
    distance_m: float,
    angle_error_rad: float,
    angle_turn_threshold_rad: float,
    distance_move_threshold_m: float,
) -> Dict[str, Any]:
    """
    Дискретные примитивы движения верхнего уровня.

    Приоритет: большая ошибка курса → поворот; иначе нужно ехать → движение; иначе стоп.
    Углы в возвращаемом словаре — в градусах.
    """
    ae = abs(normalize_angle(angle_error_rad))
    if ae > angle_turn_threshold_rad:
        deg = math.degrees(normalize_angle(angle_error_rad))
        return {"type": "turn", "angle_deg": deg}
    if distance_m > distance_move_threshold_m:
        return {"type": "move", "distance_m": float(distance_m)}
    return {"type": "stop"}

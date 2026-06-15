"""Преобразования ориентации: кватернион ↔ рысканье (yaw)."""

from __future__ import annotations

import math
from typing import Tuple

from navigation_module.core.geometry import normalize_angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Извлечь рысканье (поворот вокруг Z) в радианах.

    Кватернион в порядке XYZW, как часто в ROS (geometry_msgs).
    Yaw согласован с кадром робота/плоскости и навигацией через atan2(dy, dx).
    """
    # yaw из кватерниона (ZYX / плоское рысканье)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


def yaw_to_quaternion_z(z_yaw_rad: float) -> Tuple[float, float, float, float]:
    """Минимальный кватернион чистого поворота вокруг Z: (x, y, z, w)."""
    half = z_yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)

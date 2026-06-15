"""Базовые геометрические функции: углы, векторы, ограничение диапазона."""

from __future__ import annotations

import math
from typing import Tuple


def normalize_angle(angle_rad: float) -> float:
    """Свернуть угол в интервал (-π, π]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def clamp(value: float, low: float, high: float) -> float:
    """Ограничить значение отрезком [low, high]."""
    return max(low, min(high, value))


def normalize_vector(vx: float, vy: float) -> Tuple[float, float]:
    """Единичный вектор или (0, 0), если длина почти нуль."""
    length = math.hypot(vx, vy)
    if length < 1e-12:
        return 0.0, 0.0
    return vx / length, vy / length

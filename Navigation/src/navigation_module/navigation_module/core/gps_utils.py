"""Утилиты GPS: метры в локальной касательной плоскости от первого фикса, расстояние, пеленг."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# Средний радиус Земли (метры)
_EARTH_RADIUS_M = 6_371_000.0


@dataclass
class GPSOrigin:
    """Начало локальных координат в стиле ENU: XY в метрах (x — восток, y — север)."""

    lat0_rad: float
    lon0_rad: float
    cos_lat0: float


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def make_origin(lat_deg: float, lon_deg: float) -> GPSOrigin:
    """Создать начало координат по первому GPS-измерению."""
    lat0 = _deg_to_rad(lat_deg)
    lon0 = _deg_to_rad(lon_deg)
    return GPSOrigin(lat0_rad=lat0, lon0_rad=lon0, cos_lat0=math.cos(lat0))


def latlon_to_local_meters(
    lat_deg: float, lon_deg: float, origin: GPSOrigin
) -> Tuple[float, float]:
    """
    Плоская модель Земли: x — восток (м), y — север (м) относительно начала.
    Достаточно точно для типичных масштабов полевого робота (порядка километров).
    """
    lat = _deg_to_rad(lat_deg)
    lon = _deg_to_rad(lon_deg)
    dlat = lat - origin.lat0_rad
    dlon = lon - origin.lon0_rad
    x = dlon * origin.cos_lat0 * _EARTH_RADIUS_M
    y = dlat * _EARTH_RADIUS_M
    return x, y


def gps_distance_meters(
    lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float
) -> float:
    """Расстояние по дуге большого круга по формуле гаверсинуса (метры)."""
    phi1 = _deg_to_rad(lat1_deg)
    phi2 = _deg_to_rad(lat2_deg)
    dphi = _deg_to_rad(lat2_deg - lat1_deg)
    dlamb = _deg_to_rad(lon2_deg - lon1_deg)
    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlamb / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    return _EARTH_RADIUS_M * c


def bearing_between_points(
    lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float
) -> float:
    """
    Начальный пеленг из точки 1 в точку 2 (радианы; матем. угол: 0 = ось +x на восток).
    Перевод из навигационного пеленга (по часовой от севера) для плоской карты.
    """
    phi1 = _deg_to_rad(lat1_deg)
    phi2 = _deg_to_rad(lat2_deg)
    dlamb = _deg_to_rad(lon2_deg - lon1_deg)
    y = math.sin(dlamb) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(
        dlamb
    )
    bearing_geo = math.atan2(y, x)
    # Матем. конвенция: x — восток, y — север → рысканье от +x: (π/2 − пеленг_от_севера)
    return math.pi / 2.0 - bearing_geo

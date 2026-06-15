#!/usr/bin/env python3
"""Кадр миссии: origin JSON → локальные XY в map (для GPS и waypoints)."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from navigation_module.core.gps_utils import GPSOrigin, latlon_to_local_meters, make_origin
from navigation_module.mission.waypoint_loader import lat_lon_from_mapping


@dataclass(frozen=True)
class MissionFrame:
    """Локальная система координат маршрута (map), согласованная с waypoints.json."""

    origin_x: float
    origin_y: float
    origin_yaw: float
    gps_origin: GPSOrigin


def mission_frame_from_dict(raw: dict) -> MissionFrame:
    """Построить кадр миссии из корня JSON (поле origin + fallback по первой точке)."""
    origin = raw.get("origin", {}) if isinstance(raw, dict) else {}
    if not isinstance(origin, dict):
        origin = {}

    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    origin_yaw = float(origin.get("yaw", 0.0))
    origin_geo = lat_lon_from_mapping(origin)

    if origin_geo is None:
        points = raw.get("waypoints", [])
        if isinstance(points, list):
            for point in points:
                if isinstance(point, dict):
                    origin_geo = lat_lon_from_mapping(point)
                elif isinstance(point, (list, tuple)) and len(point) >= 2:
                    origin_geo = (float(point[0]), float(point[1]))
                if origin_geo is not None:
                    break

    if origin_geo is None:
        raise ValueError("для навигации по GPS нужен origin или waypoints с lat/lon")

    gps_origin = make_origin(origin_geo[0], origin_geo[1])
    return MissionFrame(
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        gps_origin=gps_origin,
    )


def load_mission_frame(path: str | Path) -> MissionFrame:
    waypoint_path = Path(path)
    raw = json.loads(waypoint_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"ожидается JSON-объект в {waypoint_path}")
    return mission_frame_from_dict(raw)


def gps_to_map_xy(lat_deg: float, lon_deg: float, frame: MissionFrame) -> tuple[float, float]:
    """GPS (WGS84) → XY в кадре map (та же формула, что при загрузке waypoints)."""
    x_local, y_local = latlon_to_local_meters(lat_deg, lon_deg, frame.gps_origin)
    cy = math.cos(frame.origin_yaw)
    sy = math.sin(frame.origin_yaw)
    x = frame.origin_x + cy * x_local - sy * y_local
    y = frame.origin_y + sy * x_local + cy * y_local
    return x, y

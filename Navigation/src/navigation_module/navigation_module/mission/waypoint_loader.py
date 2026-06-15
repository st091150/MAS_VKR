#!/usr/bin/env python3
"""Загрузка waypoints из JSON и перевод геокоординат в локальные XY."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from navigation_module.core.gps_utils import latlon_to_local_meters, make_origin


@dataclass
class LoadedWaypoint:
    wid: str
    x: float
    y: float
    reach_radius_m: float
    latitude: float | None = None
    longitude: float | None = None


def lat_lon_from_mapping(data: dict) -> tuple[float, float] | None:
    lat = data.get("latitude", data.get("lat"))
    lon = data.get("longitude", data.get("lon", data.get("lng")))
    if lat is None or lon is None:
        return None
    return float(lat), float(lon)


def load_waypoints(path: str | Path, default_reach_radius_m: float = 0.9) -> list[LoadedWaypoint]:
    waypoint_path = Path(path)
    raw = json.loads(waypoint_path.read_text(encoding="utf-8"))
    origin = raw.get("origin", {}) if isinstance(raw, dict) else {}
    if not isinstance(origin, dict):
        origin = {}

    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    origin_yaw = float(origin.get("yaw", 0.0))
    origin_geo = lat_lon_from_mapping(origin)
    points = raw.get("waypoints", [])
    if not isinstance(points, list) or len(points) == 0:
        raise ValueError(f"в файле {waypoint_path} пустой список waypoints")

    if origin_geo is None:
        for point in points:
            if isinstance(point, dict):
                origin_geo = lat_lon_from_mapping(point)
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                origin_geo = (float(point[0]), float(point[1]))
            if origin_geo is not None:
                break

    gps_origin = make_origin(origin_geo[0], origin_geo[1]) if origin_geo else None
    cy = math.cos(origin_yaw)
    sy = math.sin(origin_yaw)
    default_radius = float(raw.get("default_reach_radius_m", default_reach_radius_m))
    loaded: list[LoadedWaypoint] = []

    for i, point in enumerate(points):
        lat: float | None
        lon: float | None
        if isinstance(point, dict):
            wid = str(point.get("id", f"wp_{i + 1}"))
            radius = float(point.get("reach_radius_m", default_radius))
            geo = lat_lon_from_mapping(point)
            if geo is not None:
                if gps_origin is None:
                    raise ValueError("для точек lat/lon нужен origin с координатами")
                lat, lon = geo
                x_local, y_local = latlon_to_local_meters(lat, lon, gps_origin)
            else:
                lat = None
                lon = None
                x_local = float(point["x"])
                y_local = float(point["y"])
        elif isinstance(point, (list, tuple)) and len(point) >= 2:
            if gps_origin is None:
                raise ValueError("для точек lat/lon нужен origin с координатами")
            wid = f"wp_{i + 1}"
            lat = float(point[0])
            lon = float(point[1])
            radius = default_radius
            x_local, y_local = latlon_to_local_meters(lat, lon, gps_origin)
        else:
            raise ValueError(f"точка #{i + 1}: ожидается объект или пара [lat, lon]")

        x = origin_x + cy * x_local - sy * y_local
        y = origin_y + sy * x_local + cy * y_local
        loaded.append(
            LoadedWaypoint(
                wid=wid,
                x=x,
                y=y,
                reach_radius_m=max(0.05, radius),
                latitude=lat,
                longitude=lon,
            )
        )

    return loaded

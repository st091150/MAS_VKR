#!/usr/bin/env python3
"""Генерация SDF мира Gazebo: визуал waypoints из того же JSON, что и waypoint_nav."""

from __future__ import annotations

import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from navigation_module.mission.mission_frame import load_mission_frame
from navigation_module.mission.waypoint_loader import load_waypoints
from navigation_ros.waypoint_palette import gazebo_point_radius_rgba, rgb_for_index

# Радиус сферы маркера точки (должен совпадать с удлинением сегмента маршрута).
_POINT_VISUAL_RADIUS_M = 0.18


@dataclass
class VisualWaypoint:
    name: str
    x: float
    y: float
    radius: float


def _load_visual_waypoints(path: Path) -> list[VisualWaypoint]:
    return [
        VisualWaypoint(name=wp.wid, x=wp.x, y=wp.y, radius=wp.reach_radius_m)
        for wp in load_waypoints(path)
    ]


def _add_text(parent: ET.Element, name: str, text: str) -> ET.Element:
    child = ET.SubElement(parent, name)
    child.text = text
    return child


def _set_text(parent: ET.Element, name: str, text: str) -> None:
    child = parent.find(name)
    if child is None:
        child = ET.SubElement(parent, name)
    child.text = text


def _sync_spherical_coordinates(world: ET.Element, waypoint_path: Path) -> None:
    frame = load_mission_frame(waypoint_path)
    spherical = world.find("spherical_coordinates")
    if spherical is None:
        spherical = ET.SubElement(world, "spherical_coordinates")

    lat_deg = math.degrees(frame.gps_origin.lat0_rad)
    lon_deg = math.degrees(frame.gps_origin.lon0_rad)
    heading_deg = math.degrees(frame.origin_yaw)
    _set_text(spherical, "surface_model", "EARTH_WGS84")
    _set_text(spherical, "world_frame_orientation", "ENU")
    _set_text(spherical, "latitude_deg", f"{lat_deg:.12f}")
    _set_text(spherical, "longitude_deg", f"{lon_deg:.12f}")
    _set_text(spherical, "elevation", "0")
    _set_text(spherical, "heading_deg", f"{heading_deg:.12f}")
    print(
        "[gazebo_waypoint_world] GPS origin мира: "
        f"lat={lat_deg:.8f}, lon={lon_deg:.8f}, heading={heading_deg:.2f}°"
    )


def _make_waypoint_model(wp: VisualWaypoint, index: int) -> ET.Element:
    point_color, radius_color = gazebo_point_radius_rgba(index)
    model = ET.Element("model", {"name": wp.name})
    _add_text(model, "static", "true")
    _add_text(model, "pose", f"{wp.x:.6f} {wp.y:.6f} 0 0 0 0")

    link = ET.SubElement(model, "link", {"name": "l"})

    point_visual = ET.SubElement(link, "visual", {"name": "point"})
    _add_text(point_visual, "pose", "0 0 0.15 0 0 0")
    point_geometry = ET.SubElement(point_visual, "geometry")
    sphere = ET.SubElement(point_geometry, "sphere")
    _add_text(sphere, "radius", f"{_POINT_VISUAL_RADIUS_M:.6f}")
    point_material = ET.SubElement(point_visual, "material")
    _add_text(point_material, "ambient", point_color)
    _add_text(point_material, "diffuse", point_color)

    radius_visual = ET.SubElement(link, "visual", {"name": "reach_radius"})
    _add_text(radius_visual, "pose", "0 0 0.055 0 0 0")
    radius_geometry = ET.SubElement(radius_visual, "geometry")
    cylinder = ET.SubElement(radius_geometry, "cylinder")
    _add_text(cylinder, "radius", f"{wp.radius:.6f}")
    _add_text(cylinder, "length", "0.08")
    radius_material = ET.SubElement(radius_visual, "material")
    _add_text(radius_material, "ambient", radius_color)
    _add_text(radius_material, "diffuse", radius_color)

    return model


def _make_route_segment(a: VisualWaypoint, b: VisualWaypoint, seg_index: int) -> ET.Element:
    dx = b.x - a.x
    dy = b.y - a.y
    chord = math.hypot(dx, dy)
    if chord < 1e-6:
        chord = 1e-6
    # Цилиндр чуть длиннее хорды, чтобы визуально стыковался со сферами маркеров.
    length = chord + 2.0 * _POINT_VISUAL_RADIUS_M
    yaw = math.atan2(dy, dx)
    model = ET.Element("model", {"name": f"wp_route_{seg_index + 1}"})
    _add_text(model, "static", "true")
    _add_text(model, "pose", f"{(a.x + b.x) * 0.5:.6f} {(a.y + b.y) * 0.5:.6f} 0.08 0 1.570796 {yaw:.6f}")

    link = ET.SubElement(model, "link", {"name": "l"})
    visual = ET.SubElement(link, "visual", {"name": "route"})
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    _add_text(cylinder, "radius", "0.035")
    _add_text(cylinder, "length", f"{length:.6f}")
    material = ET.SubElement(visual, "material")
    # Цвет сегмента — как у конечной точки сегмента (индекс в маршруте seg_index + 1).
    dr, dg, db = rgb_for_index(seg_index + 1)
    amb = f"{dr:.4f} {dg:.4f} {db:.4f} 1.0"
    _add_text(material, "ambient", amb)
    _add_text(material, "diffuse", amb)
    return model


def generate_gazebo_world(base_world: str, waypoint_file: str) -> str:
    """Пишет временный SDF мира с визуалом точек из waypoint_file и возвращает путь к файлу."""
    base_path = Path(base_world)
    waypoint_path = Path(waypoint_file)
    tree = ET.parse(base_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"в {base_path} нет элемента <world>")

    _sync_spherical_coordinates(world, waypoint_path)

    for model in list(world.findall("model")):
        name = model.get("name") or ""
        if name.startswith("wp_"):
            world.remove(model)

    waypoints = _load_visual_waypoints(waypoint_path)
    print("[gazebo_waypoint_world] Координаты waypoints для Gazebo:")
    for index, waypoint in enumerate(waypoints):
        print(
            f"[gazebo_waypoint_world] {waypoint.name}: "
            f"x={waypoint.x:.3f}, y={waypoint.y:.3f}, r={waypoint.radius:.2f}"
        )
        world.append(_make_waypoint_model(waypoint, index))
    print("[gazebo_waypoint_world] Длины сегментов (хорда); диски r сильнее перекрываются, если хорда < r_a+r_b:")
    for index, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
        chord = math.hypot(b.x - a.x, b.y - a.y)
        overlap = a.radius + b.radius
        print(
            f"[gazebo_waypoint_world]   {a.name}→{b.name}: хорда={chord:.2f} м "
            f"(r {a.radius:.2f}+{b.radius:.2f}={overlap:.2f} м)"
        )
        world.append(_make_route_segment(a, b, index))

    out = Path(tempfile.gettempdir()) / "thrust_nav_sim_generated_world.sdf"
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return str(out)

#!/usr/bin/env python3
"""Tests for generated Gazebo world metadata."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from thrust_nav_sim.gazebo_waypoint_world import generate_gazebo_world


def test_generated_world_uses_route_origin(tmp_path: Path) -> None:
    base_world = Path(__file__).resolve().parents[1] / "worlds" / "thrust_world.sdf"
    route = tmp_path / "route.json"
    route.write_text(
        json.dumps(
            {
                "origin": {
                    "latitude": 59.88177647365234,
                    "longitude": 29.829061153390533,
                    "yaw": 0.25,
                },
                "waypoints": [
                    {
                        "id": "wp_1",
                        "latitude": 59.88177647365234,
                        "longitude": 29.829061153390533,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    generated = Path(generate_gazebo_world(str(base_world), str(route)))
    root = ET.parse(generated).getroot()
    spherical = root.find("world/spherical_coordinates")
    assert spherical is not None
    assert float(spherical.findtext("latitude_deg", "nan")) == pytest.approx(
        59.88177647365234
    )
    assert float(spherical.findtext("longitude_deg", "nan")) == pytest.approx(
        29.829061153390533
    )
    assert float(spherical.findtext("heading_deg", "nan")) == pytest.approx(
        14.323944878, abs=1e-6
    )

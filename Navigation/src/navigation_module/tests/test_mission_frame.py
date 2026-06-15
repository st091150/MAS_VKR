#!/usr/bin/env python3
"""Тесты перевода GPS в кадр map."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest

from navigation_module.mission.mission_frame import (
    gps_to_map_xy,
    load_mission_frame,
    mission_frame_from_dict,
)


def test_gps_at_origin_is_zero():
    raw = {"origin": {"latitude": 55.0, "longitude": 37.0, "yaw": 0.0}, "waypoints": []}
    frame = mission_frame_from_dict(raw)
    x, y = gps_to_map_xy(55.0, 37.0, frame)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_gps_to_map_xy_matches_waypoint_loader():
    from navigation_module.mission.waypoint_loader import load_waypoints

    raw = {
        "origin": {"latitude": 55.0, "longitude": 37.0, "yaw": 0.0},
        "waypoints": [
            {"id": "wp_1", "latitude": 55.0, "longitude": 37.00019575},
        ],
    }
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "route.json"
        path.write_text(json.dumps(raw), encoding="utf-8")
        frame = load_mission_frame(path)
        wps = load_waypoints(path)
        x, y = gps_to_map_xy(55.0, 37.00019575, frame)
        assert x == pytest.approx(wps[0].x, abs=1e-4)
        assert y == pytest.approx(wps[0].y, abs=1e-4)


def test_origin_yaw_rotates_local_gps():
    raw = {"origin": {"latitude": 55.0, "longitude": 37.0, "yaw": 1.57079632679}, "waypoints": []}
    frame = mission_frame_from_dict(raw)
    x0, y0 = gps_to_map_xy(55.0, 37.0, frame)
    x1, y1 = gps_to_map_xy(55.0001, 37.0, frame)
    assert x0 == pytest.approx(0.0, abs=1e-6)
    assert y0 == pytest.approx(0.0, abs=1e-6)
    assert abs(x1) + abs(y1) > 0.5

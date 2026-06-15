"""Тесты sim_waypoint."""

from navigation_module.config.sim_settings import SimWaypointSettings
from navigation_module.core.sim_waypoint import (
    needs_course_realign,
    reached_waypoint,
    segment_metrics,
)


def test_reached_waypoint():
    assert reached_waypoint(0.0, 0.0, 0.5, 0.0, 1.0)
    assert not reached_waypoint(0.0, 0.0, 5.0, 0.0, 1.0)


def test_needs_realign_uses_generate_command():
    s = SimWaypointSettings(heading_realign_rad=0.3)
    assert needs_course_realign(0.5, s)
    assert not needs_course_realign(0.1, s)


def test_segment_metrics():
    dist, bearing, err = segment_metrics(0.0, 0.0, 0.0, 1.0, 0.0)
    assert abs(dist - 1.0) < 1e-9
    assert abs(bearing) < 1e-9
    assert abs(err) < 1e-9

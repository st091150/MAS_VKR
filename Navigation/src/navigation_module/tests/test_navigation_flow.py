"""Проверки generate_command и фильтра GPS."""

import math

from navigation_module.core.filters import LowPassFilter
from navigation_module.core.navigation import generate_command


def test_generate_command_turn_priority():
    cmd = generate_command(
        distance_m=10.0,
        angle_error_rad=math.radians(40),
        angle_turn_threshold_rad=math.radians(10),
        distance_move_threshold_m=0.5,
    )
    assert cmd["type"] == "turn"
    assert "angle_deg" in cmd


def test_generate_command_move():
    cmd = generate_command(
        distance_m=5.0,
        angle_error_rad=0.01,
        angle_turn_threshold_rad=0.2,
        distance_move_threshold_m=0.5,
    )
    assert cmd["type"] == "move"
    assert cmd["distance_m"] == 5.0


def test_generate_command_stop():
    cmd = generate_command(
        distance_m=0.2,
        angle_error_rad=0.01,
        angle_turn_threshold_rad=0.2,
        distance_move_threshold_m=0.5,
    )
    assert cmd == {"type": "stop"}


def test_low_pass_smoothing():
    f = LowPassFilter(alpha=0.5, initial_lat=0.0, initial_lon=0.0)
    la, lo = f.filter_point(1.0, 1.0)
    assert la == 0.5 and lo == 0.5

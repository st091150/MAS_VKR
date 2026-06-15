"""Тесты функций для углов и yaw из кватерниона."""

import math

from navigation_module.core.geometry import clamp, normalize_angle
from navigation_module.core.navigation import angle_error, bearing_to_target
from navigation_module.core.orientation import quaternion_to_yaw, yaw_to_quaternion_z


def test_normalize_angle_wrap():
    assert math.isclose(normalize_angle(4 * math.pi), 0.0, abs_tol=1e-12)
    assert math.isclose(abs(normalize_angle(3 * math.pi)), math.pi)


def test_angle_error_uses_atan2_form():
    err = angle_error(0.0, math.pi / 2)
    assert math.isclose(err, math.pi / 2)
    err2 = angle_error(math.pi / 2, 0.0)
    assert math.isclose(err2, -math.pi / 2)


def test_bearing_eastish_track():
    # Отрезок примерно на восток у экватора
    b = bearing_to_target((0.0, 0.0), (0.0, 0.01))
    assert abs(b) < 0.2


def test_quaternion_pure_yaw():
    q = yaw_to_quaternion_z(math.pi / 6)
    yaw = quaternion_to_yaw(*q)
    assert math.isclose(yaw, math.pi / 6)


def test_quaternion_identity_zero_yaw():
    yaw = quaternion_to_yaw(0.0, 0.0, 0.0, 1.0)
    assert math.isclose(yaw, 0.0)


def test_clamp():
    assert clamp(5.0, 0.0, 1.0) == 1.0
    assert clamp(-1.0, 0.0, 1.0) == 0.0

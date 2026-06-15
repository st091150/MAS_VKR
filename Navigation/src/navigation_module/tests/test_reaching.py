"""Тесты радиуса допуска достижения точки."""

from navigation_module.core.navigation import reached_target


def test_inside_radius():
    cur = (52.0, 13.0)
    tgt = (52.000005, 13.0)
    assert reached_target(cur, tgt, radius=50.0) is True


def test_outside_radius():
    cur = (52.0, 13.0)
    tgt = (52.1, 13.0)
    assert reached_target(cur, tgt, radius=1.0) is False


def test_on_boundary_tolerance():
    cur = (52.0, 13.0)
    tgt = (52.0, 13.0)
    assert reached_target(cur, tgt, radius=1.0) is True

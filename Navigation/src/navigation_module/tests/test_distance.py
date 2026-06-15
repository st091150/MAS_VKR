"""Тесты функций, связанных с расстоянием."""

import math

from navigation_module.core.geometry import normalize_vector
from navigation_module.core.gps_utils import gps_distance_meters, make_origin
from navigation_module.core.navigation import distance_to_target


def test_gps_distance_zero():
    assert gps_distance_meters(52.0, 13.0, 52.0, 13.0) == 0.0


def test_gps_distance_short_hop():
    d = gps_distance_meters(52.0, 13.0, 52.001, 13.0)
    assert 110 < d < 120


def test_distance_to_target_tuple():
    cur = (52.0, 13.0)
    tgt = (52.001, 13.0)
    assert abs(distance_to_target(cur, tgt) - gps_distance_meters(*cur, *tgt)) < 1e-9


def test_normalize_vector_unit():
    ux, uy = normalize_vector(3.0, 4.0)
    assert math.isclose(ux, 0.6) and math.isclose(uy, 0.8)


def test_normalize_vector_zero():
    assert normalize_vector(0.0, 0.0) == (0.0, 0.0)


def test_make_origin_cos_lat():
    o = make_origin(0.0, 0.0)
    assert o.cos_lat0 == 1.0

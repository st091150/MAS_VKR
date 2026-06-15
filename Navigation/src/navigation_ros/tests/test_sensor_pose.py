#!/usr/bin/env python3
"""Unit tests for GPS+IMU pose estimation."""

from __future__ import annotations

import math
import random

import pytest
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from navigation_module.mission.mission_frame import mission_frame_from_dict
from navigation_ros.sensor_pose import GpsImuPoseEstimator


def _fix(lat: float, lon: float, status: int = NavSatStatus.STATUS_FIX) -> NavSatFix:
    msg = NavSatFix()
    msg.status.status = status
    msg.latitude = lat
    msg.longitude = lon
    msg.header.stamp.sec = 10
    return msg


def _imu_yaw(yaw: float) -> Imu:
    msg = Imu()
    msg.orientation.z = math.sin(yaw * 0.5)
    msg.orientation.w = math.cos(yaw * 0.5)
    msg.header.stamp.sec = 10
    return msg


def test_estimator_rejects_no_fix() -> None:
    frame = mission_frame_from_dict(
        {"origin": {"latitude": 55.0, "longitude": 37.0}, "waypoints": []}
    )
    estimator = GpsImuPoseEstimator(frame, min_gps_fix_status=0)
    estimator.update_gps(_fix(55.0, 37.0, status=NavSatStatus.STATUS_NO_FIX))
    assert not estimator.have_gps


def test_estimator_applies_mission_yaw_to_imu() -> None:
    frame = mission_frame_from_dict(
        {"origin": {"latitude": 55.0, "longitude": 37.0, "yaw": 0.25}, "waypoints": []}
    )
    estimator = GpsImuPoseEstimator(frame, min_gps_fix_status=0)
    estimator.update_gps(_fix(55.0, 37.0))
    estimator.update_imu(_imu_yaw(0.5))
    assert estimator.have_pose
    assert estimator.yaw == pytest.approx(0.75)


def test_estimator_applies_yaw_noise() -> None:
    frame = mission_frame_from_dict(
        {"origin": {"latitude": 55.0, "longitude": 37.0}, "waypoints": []}
    )
    random.seed(0)
    estimator = GpsImuPoseEstimator(
        frame, min_gps_fix_status=0, yaw_noise_stddev_rad=0.1
    )
    estimator.update_gps(_fix(55.0, 37.0))
    estimator.update_imu(_imu_yaw(0.0))
    assert estimator.have_pose
    assert estimator.yaw != pytest.approx(0.0, abs=1e-9)


def test_estimator_rejects_imu_without_orientation() -> None:
    frame = mission_frame_from_dict(
        {"origin": {"latitude": 55.0, "longitude": 37.0}, "waypoints": []}
    )
    estimator = GpsImuPoseEstimator(frame, min_gps_fix_status=0)
    msg = _imu_yaw(0.0)
    msg.orientation_covariance[0] = -1.0
    estimator.update_imu(msg)
    assert not estimator.have_imu

#!/usr/bin/env python3
"""Оценка позы робота по GPS + IMU (контракт для симуляции и реального робота)."""

from __future__ import annotations

import math
import random

from sensor_msgs.msg import Imu, NavSatFix

from navigation_module.core.filters import LowPassFilter
from navigation_module.core.geometry import normalize_angle
from navigation_module.core.orientation import quaternion_to_yaw
from navigation_module.mission.mission_frame import MissionFrame, gps_to_map_xy


class GpsImuPoseEstimator:
    """Поза в map: XY из /gps/fix, курс и ω из /imu/data, v — из производной GPS."""

    def __init__(
        self,
        frame: MissionFrame,
        *,
        gps_filter_alpha: float = 0.15,
        min_gps_fix_status: int = 0,
        yaw_noise_stddev_rad: float = 0.0,
    ) -> None:
        self._frame = frame
        self._gps_filter = LowPassFilter(gps_filter_alpha)
        self._min_gps_fix_status = min_gps_fix_status
        self._yaw_noise_stddev = max(0.0, yaw_noise_stddev_rad)
        self._have_gps = False
        self._have_imu = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v_meas = 0.0
        self._w_meas = 0.0
        self._last_gps_t: float | None = None
        self._last_imu_t: float | None = None
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._v_filt = 0.0
        self._v_filter_alpha = 0.25

    @property
    def have_gps(self) -> bool:
        return self._have_gps

    @property
    def have_imu(self) -> bool:
        return self._have_imu

    @property
    def have_pose(self) -> bool:
        return self._have_gps and self._have_imu

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw(self) -> float:
        return self._yaw

    @property
    def v_meas(self) -> float:
        return self._v_meas

    @property
    def w_meas(self) -> float:
        return self._w_meas

    def has_fresh_pose(
        self,
        now_sec: float,
        *,
        gps_timeout_sec: float = 0.0,
        imu_timeout_sec: float = 0.0,
    ) -> bool:
        if not self.have_pose:
            return False
        if gps_timeout_sec > 0.0 and self._last_gps_t is not None:
            if now_sec - self._last_gps_t > gps_timeout_sec:
                self._have_gps = False
                return False
        if imu_timeout_sec > 0.0 and self._last_imu_t is not None:
            if now_sec - self._last_imu_t > imu_timeout_sec:
                self._have_imu = False
                return False
        return True

    def update_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < self._min_gps_fix_status:
            self._have_gps = False
            self._v_meas = 0.0
            self._v_filt = 0.0
            self._last_gps_t = None
            self._last_x = None
            self._last_y = None
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            self._have_gps = False
            self._v_meas = 0.0
            self._v_filt = 0.0
            self._last_gps_t = None
            self._last_x = None
            self._last_y = None
            return

        lat, lon = self._gps_filter.filter_point(msg.latitude, msg.longitude)
        x, y = gps_to_map_xy(lat, lon, self._frame)
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if self._last_gps_t is not None and self._last_x is not None and self._last_y is not None:
            dt = t - self._last_gps_t
            if dt > 1e-6:
                dx = x - self._last_x
                dy = y - self._last_y
                v_raw = min(2.0, math.hypot(dx, dy) / dt)
                a = self._v_filter_alpha
                self._v_meas = a * v_raw + (1.0 - a) * self._v_filt
                self._v_filt = self._v_meas

        self._last_gps_t = t
        self._last_x = x
        self._last_y = y
        self._x = x
        self._y = y
        self._have_gps = True

    def update_imu(self, msg: Imu) -> None:
        if msg.orientation_covariance[0] < 0.0:
            self._have_imu = False
            self._w_meas = 0.0
            self._last_imu_t = None
            return
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6 or not math.isfinite(norm):
            self._have_imu = False
            self._w_meas = 0.0
            self._last_imu_t = None
            return
        yaw = normalize_angle(quaternion_to_yaw(q.x, q.y, q.z, q.w) + self._frame.origin_yaw)
        if self._yaw_noise_stddev > 0.0:
            yaw = normalize_angle(yaw + random.gauss(0.0, self._yaw_noise_stddev))
        self._yaw = yaw
        self._w_meas = float(msg.angular_velocity.z)
        self._last_imu_t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._have_imu = True

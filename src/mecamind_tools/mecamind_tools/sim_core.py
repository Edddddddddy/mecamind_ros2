from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, List

from .world_geometry import WorldGeometry


@dataclass
class SimPose:
    x: float
    y: float
    yaw: float


@dataclass
class SimTwist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _ramp(current: float, target: float, limit: float) -> float:
    delta = max(-limit, min(limit, target - current))
    return current + delta


class MecaMindSimModel:
    def __init__(
        self,
        geometry: WorldGeometry,
        initial_pose: SimPose | None = None,
        robot_radius: float = 0.22,
        max_linear_x: float = 0.45,
        max_linear_y: float = 0.35,
        max_angular_z: float = 1.5,
        max_accel_x: float = 0.9,
        max_accel_y: float = 0.9,
        max_accel_z: float = 1.8,
        sensor_offset_x: float = 0.16,
        sensor_offset_y: float = 0.0,
        lidar_range_max: float = 8.0,
        lidar_beams: int = 360,
        lidar_angle_min: float = -math.pi,
        lidar_angle_max: float = math.pi,
        lidar_noise_std: float = 0.0,
        lidar_range_bias: float = 0.0,
        lidar_dropout_prob: float = 0.0,
        random_seed: int = 8,
    ) -> None:
        self.geometry = geometry
        self.pose = initial_pose or SimPose(-3.2, -2.4, 0.0)
        self.velocity = SimTwist()
        self.command = SimTwist()
        self.robot_radius = robot_radius
        self.max_linear_x = max_linear_x
        self.max_linear_y = max_linear_y
        self.max_angular_z = max_angular_z
        self.max_accel_x = max_accel_x
        self.max_accel_y = max_accel_y
        self.max_accel_z = max_accel_z
        self.sensor_offset_x = sensor_offset_x
        self.sensor_offset_y = sensor_offset_y
        self.lidar_range_max = lidar_range_max
        self.lidar_beams = max(1, lidar_beams)
        self.lidar_angle_min = lidar_angle_min
        self.lidar_angle_max = lidar_angle_max
        self.lidar_noise_std = max(0.0, lidar_noise_std)
        self.lidar_range_bias = lidar_range_bias
        self.lidar_dropout_prob = min(1.0, max(0.0, lidar_dropout_prob))
        self._random = random.Random(random_seed)
        self.sim_time = 0.0

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        self.command.vx = max(-self.max_linear_x, min(self.max_linear_x, vx))
        self.command.vy = max(-self.max_linear_y, min(self.max_linear_y, vy))
        self.command.wz = max(-self.max_angular_z, min(self.max_angular_z, wz))

    def _sensor_origin(self) -> tuple[float, float]:
        offset_x = self.sensor_offset_x
        offset_y = self.sensor_offset_y
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        return (
            self.pose.x + offset_x * cos_yaw - offset_y * sin_yaw,
            self.pose.y + offset_x * sin_yaw + offset_y * cos_yaw,
        )

    def _collision(self, x: float, y: float) -> bool:
        return self.geometry.is_occupied(x, y, margin=self.robot_radius)

    def _integrate_candidate(self, dt: float) -> None:
        base_x = self.pose.x
        base_y = self.pose.y
        base_yaw = self.pose.yaw
        dx = (
            self.velocity.vx * math.cos(base_yaw) - self.velocity.vy * math.sin(base_yaw)
        ) * dt
        dy = (
            self.velocity.vx * math.sin(base_yaw) + self.velocity.vy * math.cos(base_yaw)
        ) * dt
        yaw = _normalize_angle(base_yaw + self.velocity.wz * dt)

        candidates = [
            (base_x + dx, base_y + dy, yaw),
            (base_x + dx, base_y, yaw),
            (base_x, base_y + dy, yaw),
            (base_x, base_y, yaw),
        ]
        for new_x, new_y, new_yaw in candidates:
            if not self._collision(new_x, new_y):
                self.pose = SimPose(new_x, new_y, new_yaw)
                return

        self.velocity = SimTwist()

    def step(self, dt: float) -> None:
        self.sim_time += dt
        self.velocity.vx = _ramp(self.velocity.vx, self.command.vx, self.max_accel_x * dt)
        self.velocity.vy = _ramp(self.velocity.vy, self.command.vy, self.max_accel_y * dt)
        self.velocity.wz = _ramp(self.velocity.wz, self.command.wz, self.max_accel_z * dt)
        self._integrate_candidate(dt)

    def scan_ranges(self) -> List[float]:
        origin_x, origin_y = self._sensor_origin()
        if self.lidar_beams == 1:
            headings = [self.pose.yaw]
        else:
            increment = (self.lidar_angle_max - self.lidar_angle_min) / (self.lidar_beams - 1)
            headings = [
                self.pose.yaw + self.lidar_angle_min + index * increment
                for index in range(self.lidar_beams)
            ]
        ranges = []
        for heading in headings:
            value = self.geometry.raycast(origin_x, origin_y, heading, self.lidar_range_max)
            if self.lidar_dropout_prob > 0.0 and self._random.random() < self.lidar_dropout_prob:
                ranges.append(self.lidar_range_max)
                continue
            if self.lidar_noise_std > 0.0:
                value += self._random.gauss(0.0, self.lidar_noise_std)
            value += self.lidar_range_bias
            ranges.append(max(0.12, min(self.lidar_range_max, value)))
        return ranges

    def scan_increment(self) -> float:
        if self.lidar_beams <= 1:
            return 0.0
        return (self.lidar_angle_max - self.lidar_angle_min) / (self.lidar_beams - 1)

    def linear_acceleration(self) -> tuple[float, float, float]:
        return 0.0, 0.0, 0.0

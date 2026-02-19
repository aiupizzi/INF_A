import math
from typing import List, Optional

from infa_safety.mission_state_machine import SafetyEventType


class VirtualCageAvoidance:
    def __init__(self, pushback_radius_m: float = 1.0, max_linear_velocity_mps: float = 0.8):
        self._pushback_radius_m = pushback_radius_m
        self._max_linear_velocity_mps = max_linear_velocity_mps
        self._ranges: List[float] = []
        self._angle_min = 0.0
        self._angle_increment = 0.0

    def update_lidar(self, ranges: List[float], angle_min: float, angle_increment: float) -> None:
        self._ranges = ranges
        self._angle_min = angle_min
        self._angle_increment = angle_increment

    def evaluate(self) -> Optional[dict]:
        if not self._ranges:
            return None

        nearest_idx = min(range(len(self._ranges)), key=self._ranges.__getitem__)
        nearest_dist = self._ranges[nearest_idx]
        if nearest_dist >= self._pushback_radius_m:
            return None

        nearest_angle = self._angle_min + nearest_idx * self._angle_increment
        push_mag = min(1.0, (self._pushback_radius_m - nearest_dist) / self._pushback_radius_m)
        vel_limit = max(0.1, self._max_linear_velocity_mps * (1.0 - push_mag))
        pushback_x = -math.cos(nearest_angle) * push_mag
        pushback_y = -math.sin(nearest_angle) * push_mag

        return {
            'event_type': SafetyEventType.VIRTUAL_CAGE_PUSHBACK,
            'reason': f'Obstacle within cage (d={nearest_dist:.2f}m, angle={nearest_angle:.2f}rad)',
            'action': 'PUSHBACK_AND_LIMIT_VELOCITY',
            'operator_alert': 'WARNING: Virtual cage pushback active.',
            'pushback_vector': {'x': pushback_x, 'y': pushback_y},
            'velocity_limit_mps': vel_limit,
        }

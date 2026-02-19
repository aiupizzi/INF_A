from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass
class StructureLockConfig:
    standoff_m: float = 2.0
    speed_mps: float = 0.4
    abort_distance_error_m: float = 1.0


class StructureLockController:
    """Computes motion toward constant-distance standoff from a plane."""

    def __init__(self, config: StructureLockConfig) -> None:
        self.config = config

    @staticmethod
    def _distance_to_plane(position: Tuple[float, float, float], plane: Sequence[float]) -> float:
        a, b, c, d = plane
        x, y, z = position
        norm = max((a * a + b * b + c * c) ** 0.5, 1e-6)
        return (a * x + b * y + c * z + d) / norm

    def compute_command(self, position: Tuple[float, float, float], plane: Sequence[float]) -> Tuple[float, float]:
        signed_dist = self._distance_to_plane(position, plane)
        error = signed_dist - self.config.standoff_m
        if abs(error) > self.config.abort_distance_error_m:
            raise RuntimeError(f'Standoff error {error:.3f} exceeds abort threshold')

        normal = plane[:3]
        norm = max((normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) ** 0.5, 1e-6)
        ux, uy = normal[0] / norm, normal[1] / norm

        gain = min(abs(error), 1.0)
        speed = self.config.speed_mps * gain
        direction = -1.0 if error > 0.0 else 1.0
        vx = direction * ux * speed
        vy = direction * uy * speed
        return vx, vy


@dataclass
class LawnmowerConfig:
    lane_spacing_m: float = 0.6
    overlap_ratio: float = 0.2
    speed_mps: float = 0.4


class LawnmowerPlanner:
    """Generates boustrophedon sweep points over a 2D patch (u, v)."""

    def __init__(self, config: LawnmowerConfig) -> None:
        self.config = config

    def generate_patch_waypoints(
        self,
        width_m: float,
        height_m: float,
        u0: float = 0.0,
        v0: float = 0.0,
    ) -> List[Tuple[float, float]]:
        effective_lane = max(self.config.lane_spacing_m * (1.0 - self.config.overlap_ratio), 0.05)
        lane_count = max(int(height_m / effective_lane) + 1, 2)

        waypoints: List[Tuple[float, float]] = []
        for lane in range(lane_count):
            v = min(v0 + lane * effective_lane, v0 + height_m)
            if lane % 2 == 0:
                waypoints.append((u0, v))
                waypoints.append((u0 + width_m, v))
            else:
                waypoints.append((u0 + width_m, v))
                waypoints.append((u0, v))
        return waypoints

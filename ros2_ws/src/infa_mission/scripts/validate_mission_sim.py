#!/usr/bin/env python3
"""Simulation-style validation for standoff lock and sweep coverage over a mock wall."""

from math import hypot
from statistics import fmean
from typing import List, Tuple

from infa_mission.mission_logic import LawnmowerConfig, LawnmowerPlanner, StructureLockConfig, StructureLockController


def validate_lock() -> Tuple[float, float]:
    """Return mean and max standoff error while converging from multiple start points."""
    controller = StructureLockController(
        StructureLockConfig(standoff_m=2.0, speed_mps=0.6, abort_distance_error_m=5.0)
    )
    plane = (-1.0, 0.0, 0.0, 5.0)  # wall at x = 5m
    dt = 0.1
    start_positions = [
        (1.0, 0.0, 0.0),
        (2.5, 1.0, 0.0),
        (4.0, -1.5, 0.0),
    ]
    standoff_errors: List[float] = []

    for start_x, start_y, start_z in start_positions:
        x, y, z = start_x, start_y, start_z
        run_errors: List[float] = []
        for _ in range(250):
            signed_dist = _distance_to_plane((x, y, z), plane)
            run_errors.append(abs(signed_dist - 2.0))

            vx, vy = controller.compute_command((x, y, z), plane)
            x += vx * dt
            y += vy * dt
            if hypot(vx, vy) < 0.01:
                break

        steady_state_window = run_errors[-20:] if len(run_errors) > 20 else run_errors
        standoff_errors.extend(steady_state_window)

    if not standoff_errors:
        return 0.0, 0.0

    return fmean(standoff_errors), max(standoff_errors)


def _distance_to_plane(position, plane):
    x, y, z = position
    a, b, c, d = plane
    norm = max((a * a + b * b + c * c) ** 0.5, 1e-6)
    return (a * x + b * y + c * z + d) / norm


def validate_coverage() -> float:
    width, height = 6.0, 4.0
    planner = LawnmowerPlanner(LawnmowerConfig(lane_spacing_m=0.8, overlap_ratio=0.2, speed_mps=0.5))
    waypoints = planner.generate_patch_waypoints(width_m=width, height_m=height)

    # Approximate each segment as a swept stripe with radius lane_spacing/2.
    lane_radius = 0.8 / 2.0
    grid_res = 0.1
    total = 0
    covered = 0

    for iu in range(int(width / grid_res) + 1):
        for iv in range(int(height / grid_res) + 1):
            p = (iu * grid_res, iv * grid_res)
            total += 1
            if _point_covered(p, waypoints, lane_radius):
                covered += 1

    return 100.0 * covered / max(total, 1)


def _point_covered(point, waypoints, lane_radius):
    px, py = point
    for idx in range(1, len(waypoints)):
        x1, y1 = waypoints[idx - 1]
        x2, y2 = waypoints[idx]
        if _distance_point_to_segment(px, py, x1, y1, x2, y2) <= lane_radius:
            return True
    return False


def _distance_point_to_segment(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-9:
        return hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = min(1.0, max(0.0, t))
    qx = x1 + t * dx
    qy = y1 + t * dy
    return hypot(px - qx, py - qy)


def main() -> None:
    mean_error, max_error = validate_lock()
    coverage_pct = validate_coverage()

    print(f'Mean standoff error: {mean_error:.3f} m')
    print(f'Max standoff deviation: {max_error:.3f} m')
    print(f'Surface coverage: {coverage_pct:.1f} %')

    if max_error > 0.15:
        raise SystemExit('FAIL: standoff error exceeded 0.15m')
    if coverage_pct < 90.0:
        raise SystemExit('FAIL: coverage below 90%')
    print('PASS: mission simulation validation')


if __name__ == '__main__':
    main()

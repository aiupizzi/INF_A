from dataclasses import dataclass
from typing import Optional

from infa_safety.mission_state_machine import SafetyEventType


@dataclass
class VioStatus:
    quality: float = 1.0
    pose_age_s: float = 0.0


class VioHealthMonitor:
    def __init__(self, degraded_quality: float = 0.45, lost_quality: float = 0.2, lost_pose_age_s: float = 0.5):
        self._degraded_quality = degraded_quality
        self._lost_quality = lost_quality
        self._lost_pose_age_s = lost_pose_age_s
        self._status = VioStatus()

    def update_quality(self, quality: float) -> None:
        self._status.quality = max(0.0, min(1.0, quality))

    def update_pose_age(self, pose_age_s: float) -> None:
        self._status.pose_age_s = max(0.0, pose_age_s)

    def evaluate(self) -> Optional[dict]:
        if self._status.quality <= self._lost_quality or self._status.pose_age_s >= self._lost_pose_age_s:
            return {
                'event_type': SafetyEventType.VIO_LOST,
                'reason': f'VIO lost (quality={self._status.quality:.2f}, pose_age={self._status.pose_age_s:.2f}s)',
                'action': 'POSITION_HOLD_OPTICAL_FLOW',
                'operator_alert': 'CRITICAL: VIO tracking lost; switched to optical-flow position hold.',
            }
        if self._status.quality <= self._degraded_quality:
            return {
                'event_type': SafetyEventType.VIO_DEGRADED,
                'reason': f'VIO degraded (quality={self._status.quality:.2f})',
                'action': 'POSITION_HOLD_OPTICAL_FLOW',
                'operator_alert': 'WARNING: VIO degraded; fallback to optical-flow hold armed.',
            }
        return None

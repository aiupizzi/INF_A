from dataclasses import dataclass
from enum import Enum
from typing import Dict


class MissionPhase(str, Enum):
    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    TRANSIT = 'TRANSIT'
    TASK = 'TASK'
    RETURN = 'RETURN'
    LAND = 'LAND'
    SAFETY_HOLD = 'SAFETY_HOLD'
    ABORT = 'ABORT'


class SafetyEventType(str, Enum):
    VIO_DEGRADED = 'VIO_DEGRADED'
    VIO_LOST = 'VIO_LOST'
    POWER_ABORT = 'POWER_ABORT'
    VIRTUAL_CAGE_PUSHBACK = 'VIRTUAL_CAGE_PUSHBACK'


EVENT_PRIORITY: Dict[SafetyEventType, int] = {
    SafetyEventType.POWER_ABORT: 100,
    SafetyEventType.VIO_LOST: 80,
    SafetyEventType.VIO_DEGRADED: 60,
    SafetyEventType.VIRTUAL_CAGE_PUSHBACK: 40,
}


@dataclass(frozen=True)
class SafetyEvent:
    event_type: SafetyEventType
    reason: str
    action: str
    seq: int


class MissionStateMachine:
    """Deterministic mission preemption from safety events."""

    def __init__(self) -> None:
        self.current_phase = MissionPhase.IDLE
        self._last_priority = -1
        self._last_seq = -1

    def update_mission_phase(self, phase: str) -> MissionPhase:
        try:
            self.current_phase = MissionPhase(phase)
        except ValueError:
            # Keep last known valid mission phase
            pass
        return self.current_phase

    def preempt_with_safety(self, event: SafetyEvent) -> bool:
        priority = EVENT_PRIORITY[event.event_type]
        if priority < self._last_priority:
            return False
        if priority == self._last_priority and event.seq <= self._last_seq:
            return False

        self._last_priority = priority
        self._last_seq = event.seq

        if event.event_type == SafetyEventType.POWER_ABORT:
            self.current_phase = MissionPhase.ABORT
        else:
            self.current_phase = MissionPhase.SAFETY_HOLD
        return True

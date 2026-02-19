from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

from infa_safety.mission_state_machine import SafetyEventType


@dataclass
class PowerSample:
    voltage_v: float
    current_a: float


class PowerAbortMonitor:
    def __init__(
        self,
        min_voltage_v: float = 14.2,
        max_internal_res_ohm: float = 0.25,
        min_thrust_margin: float = 0.15,
        max_samples: int = 30,
    ):
        self._min_voltage_v = min_voltage_v
        self._max_internal_res_ohm = max_internal_res_ohm
        self._min_thrust_margin = min_thrust_margin
        self._wind_load_proxy = 0.0
        self._samples: Deque[PowerSample] = deque(maxlen=max_samples)

    def update_battery(self, voltage_v: float, current_a: float) -> None:
        self._samples.append(PowerSample(voltage_v=max(0.0, voltage_v), current_a=max(0.0, current_a)))

    def update_wind_load_proxy(self, wind_load_proxy: float) -> None:
        self._wind_load_proxy = max(0.0, wind_load_proxy)

    def _estimate_internal_resistance(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        dv = self._samples[0].voltage_v - self._samples[-1].voltage_v
        di = max(0.1, self._samples[-1].current_a - self._samples[0].current_a)
        return max(0.0, dv / di)

    def _estimate_thrust_margin(self, voltage_v: float) -> float:
        nominal_hover_voltage = 16.8
        wind_penalty = min(0.5, self._wind_load_proxy * 0.2)
        return max(0.0, min(1.0, (voltage_v / nominal_hover_voltage) - 0.6 - wind_penalty))

    def evaluate(self) -> Optional[dict]:
        if not self._samples:
            return None

        latest = self._samples[-1]
        est_res = self._estimate_internal_resistance()
        thrust_margin = self._estimate_thrust_margin(latest.voltage_v)

        if latest.voltage_v <= self._min_voltage_v or est_res >= self._max_internal_res_ohm or thrust_margin <= self._min_thrust_margin:
            return {
                'event_type': SafetyEventType.POWER_ABORT,
                'reason': (
                    f'Power abort (V={latest.voltage_v:.2f}V, Rint={est_res:.3f}Ω, '
                    f'thrust_margin={thrust_margin:.2f}, wind={self._wind_load_proxy:.2f})'
                ),
                'action': 'ABORT_TO_SAFE_ZONE',
                'operator_alert': 'CRITICAL: Power reserve insufficient. Aborting mission to Safe Zone.',
            }
        return None

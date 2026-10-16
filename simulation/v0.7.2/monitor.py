"""BUSA-1M-001: isolated, read-only software state model.

This is NOT the approved K1-K4 control algorithm. It cannot operate hardware.
All proposed actuator states are permanently False, regardless of inputs.
"""
from dataclasses import dataclass, field
from typing import Optional
import math

NAMES = ("Вода", "Колонна", "Куб", "Верхняя точка колонны")

@dataclass
class Sample:
    value: Optional[float] = None
    valid: bool = False
    age_ms: Optional[int] = None

@dataclass
class Monitor:
    samples: list[Sample] = field(default_factory=lambda: [Sample() for _ in range(4)])
    bridge_online: bool = False
    keypad_online: bool = False
    last_key: Optional[str] = None
    key_seq: int = 0
    events: list[dict] = field(default_factory=list)
    stream_id: str = "simulation-1"

    def update(self, channel: int, value: Optional[float], age_ms: Optional[int], valid: bool) -> None:
        if not 0 <= channel < 4:
            raise ValueError("Channel must be T1..T4")
        ok = (valid and self.bridge_online and value is not None
              and isinstance(value, (int, float)) and math.isfinite(value)
              and -55 <= value <= 125 and isinstance(age_ms, int)
              and not isinstance(age_ms, bool) and 0 <= age_ms <= 3000)
        self.samples[channel] = Sample(float(value), True, age_ms) if ok else Sample()

    def press(self, key: str) -> None:
        if not self.keypad_online or not isinstance(key, str) or not key:
            return
        self.key_seq += 1
        self.last_key = key
        self.events.append({"seq": self.key_seq, "key": key})
        self.events = self.events[-64:]

    def capabilities(self) -> dict:
        return {"device": "BUSA-1M-001", "version": "v0.7.2-sim",
                "api_read_only": True, "physical_outputs": False,
                "temperature_channels": 4, "keyboard": "4x4 PCF8575",
                "display": False}

    def telemetry(self) -> dict:
        # Fail closed on bridge loss even if previous samples were valid.
        good = [s if self.bridge_online and s.valid else Sample() for s in self.samples]
        return {"device": "BUSA-1M-001", "version": "v0.7.2-sim",
                "mode": "MONITOR_ONLY", "outputs_enabled": False,
                "bridge_online": self.bridge_online, "keypad_online": self.keypad_online,
                "temperatures": [s.value for s in good],
                "channels": [{"valid": s.valid, "age_ms": s.age_ms} for s in good],
                "last_key": self.last_key, "key_seq": self.key_seq,
                "key_stream_id": self.stream_id,
                "simulation": True,
                "k_states": {f"K{i}": False for i in range(1, 5)}}

    def keys(self) -> dict:
        return {"stream_id": self.stream_id, "events": list(self.events)}

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class ScenarioEvent:
    at_s: float
    action: str
    value: Any
    note: str = ""


class Scenario:
    def __init__(self, name: str, duration_s: float, events: list[ScenarioEvent]) -> None:
        self.name = name
        self.duration_s = duration_s
        self.events = sorted(events, key=lambda event: event.at_s)
        self._next = 0

    @classmethod
    def load(cls, path: str | Path) -> Scenario:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            name=data["name"],
            duration_s=float(data["duration_s"]),
            events=[ScenarioEvent(**event) for event in data["events"]],
        )

    def due(self, elapsed_s: float) -> list[ScenarioEvent]:
        events: list[ScenarioEvent] = []
        while self._next < len(self.events) and self.events[self._next].at_s <= elapsed_s:
            events.append(self.events[self._next])
            self._next += 1
        return events

    def reset(self) -> None:
        self._next = 0


def bundled_scenario(name: str) -> Path:
    root = Path(__file__).resolve().parents[2]
    path = root / "scenarios" / f"{name}.json"
    if not path.exists():
        choices = ", ".join(item.stem for item in sorted((root / "scenarios").glob("*.json")))
        raise ValueError(f"unknown scenario {name!r}; choose: {choices}")
    return path

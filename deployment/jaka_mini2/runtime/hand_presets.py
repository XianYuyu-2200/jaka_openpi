"""Keyboard-selectable O6 hand pose labels for demonstration collection.

The values are native O6 device positions (six integers in ``0..255``), not
robot-arm joint angles.  A preset must be explicitly calibrated before it can
be selected.  This module never sends a command to the hand.
"""

from __future__ import annotations

from dataclasses import dataclass
import pathlib
from typing import Any

import yaml


@dataclass(frozen=True)
class HandPreset:
    key: str
    name: str
    target: tuple[float, ...] | None
    description: str = ""

    @property
    def calibrated(self) -> bool:
        return self.target is not None


def _parse_target(raw: Any, *, key: str) -> tuple[float, ...] | None:
    if raw is None:
        return None
    if not isinstance(raw, list | tuple) or len(raw) != 6:
        raise ValueError(f"preset {key!r} target must contain six values or be null")
    target = tuple(float(value) for value in raw)
    if any(value < 0 or value > 255 for value in target):
        raise ValueError(f"preset {key!r} target values must be within 0..255")
    return target


def load_hand_presets(path: pathlib.Path) -> dict[str, HandPreset]:
    """Load and validate keyboard presets from YAML."""
    with path.open(encoding="utf-8") as file:
        document = yaml.safe_load(file) or {}
    entries = document.get("presets")
    if not isinstance(entries, dict) or not entries:
        raise ValueError("preset config must contain a non-empty 'presets' mapping")
    presets: dict[str, HandPreset] = {}
    for raw_key, raw in entries.items():
        key = str(raw_key)
        if not isinstance(raw, dict):
            raise ValueError(f"preset {key!r} must be a mapping")
        presets[key] = HandPreset(
            key=key,
            name=str(raw.get("name", key)),
            target=_parse_target(raw.get("target"), key=key),
            description=str(raw.get("description", "")),
        )
    return presets


class HandPresetState:
    """Maintain the active O6 action label during a 5 Hz recording loop."""

    def __init__(self, presets: dict[str, HandPreset], *, initial_key: str = "space") -> None:
        self.presets = presets
        self.active_key: str | None = initial_key if initial_key in presets else None
        self.event_index = 0

    @property
    def active(self) -> HandPreset | None:
        return self.presets.get(self.active_key) if self.active_key is not None else None

    def select(self, key: str, *, current_state: tuple[float, ...] | None = None) -> HandPreset:
        """Select a calibrated preset, or bind Space to the current hand state."""
        if key == "space":
            if current_state is None or len(current_state) != 6:
                raise ValueError("space requires the current six-value O6 state")
            preset = HandPreset("space", "hold_current", tuple(float(v) for v in current_state))
            self.presets["space"] = preset
        else:
            if key not in self.presets:
                raise KeyError(f"unknown O6 preset key {key!r}")
            preset = self.presets[key]
            if not preset.calibrated:
                raise ValueError(f"O6 preset {key!r} ({preset.name}) is not calibrated")
        self.active_key = key
        self.event_index += 1
        return preset

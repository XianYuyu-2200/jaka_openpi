"""Runtime safety envelope derived from deployment/local/robot_config.yaml."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from dataclasses import field
import math
import pathlib

import yaml

ARM_DOF = 6
ACTION_DIMENSION = 12


class SafetyViolationError(ValueError):
    """An action or observation is outside the configured safe envelope."""


@dataclass(frozen=True)
class SafetyEnvelope:
    joint_position_limits: tuple[tuple[float, float], ...]
    joint_velocity_limits: tuple[float, ...]
    hand_position_limits: tuple[float, float] = (0.0, 255.0)
    workspace_limits_mm: dict[str, tuple[float, float]] = field(default_factory=dict)
    hold_timeout_s: float = 0.100
    policy_timeout_s: float = 1.000

    def validate_action(
        self,
        action: Sequence[float],
        *,
        current_position: Sequence[float] | None = None,
        dt_s: float | None = None,
        tcp_xyz_mm: Sequence[float] | None = None,
    ) -> tuple[float, ...]:
        if len(action) != ACTION_DIMENSION:
            raise SafetyViolationError(f"action must have {ACTION_DIMENSION} values")
        values = tuple(float(value) for value in action)
        for index, (value, (lower, upper)) in enumerate(zip(values[:ARM_DOF], self.joint_position_limits, strict=True)):
            if not lower <= value <= upper:
                raise SafetyViolationError(f"J{index + 1} position {value} outside [{lower}, {upper}]")
        hand_low, hand_high = self.hand_position_limits
        for index, value in enumerate(values[ARM_DOF:], start=1):
            if not hand_low <= value <= hand_high:
                raise SafetyViolationError(f"hand joint {index} position {value} outside [{hand_low}, {hand_high}]")
        if current_position is not None:
            if len(current_position) != ACTION_DIMENSION:
                raise SafetyViolationError("current_position must have 12 values")
            if not all(math.isfinite(float(value)) for value in current_position):
                raise SafetyViolationError("current_position contains a non-finite value")
            if dt_s is None or dt_s <= 0:
                raise SafetyViolationError("positive dt_s is required for velocity checking")
            for index, (new, old, limit) in enumerate(
                zip(
                    values[:ARM_DOF],
                    current_position[:ARM_DOF],
                    self.joint_velocity_limits,
                    strict=True,
                ),
                start=1,
            ):
                velocity = abs(new - float(old)) / dt_s
                if velocity > limit:
                    raise SafetyViolationError(f"J{index} velocity {velocity:.6f} rad/s exceeds {limit:.6f}")
        if self.workspace_limits_mm and tcp_xyz_mm is None:
            raise SafetyViolationError("target TCP is required for workspace checking")
        if tcp_xyz_mm is not None:
            if len(tcp_xyz_mm) != 3:
                raise SafetyViolationError("tcp_xyz_mm must contain X, Y, Z")
            for axis, value in zip(("x", "y", "z"), tcp_xyz_mm, strict=True):
                if axis in self.workspace_limits_mm:
                    lower, upper = self.workspace_limits_mm[axis]
                    if not lower <= float(value) <= upper:
                        raise SafetyViolationError(f"TCP {axis} {value} mm outside [{lower}, {upper}]")
        return values

    def check_freshness(self, age_s: float, *, timeout_s: float | None = None) -> None:
        if age_s < 0:
            raise SafetyViolationError("observation age cannot be negative")
        threshold = self.hold_timeout_s if timeout_s is None else timeout_s
        if threshold <= 0:
            raise SafetyViolationError("freshness timeout must be positive")
        if age_s >= threshold:
            raise SafetyViolationError(f"observation timeout: age={age_s:.3f}s")


def default_envelope(config_path: pathlib.Path | None = None) -> SafetyEnvelope:
    """Load the safety envelope from the authoritative robot configuration."""
    if config_path is None:
        config_path = pathlib.Path(__file__).resolve().parents[2] / "local" / "robot_config.yaml"
    with config_path.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    safety = config["safety"]
    gripper_range = config["gripper"]["command"]["range"]
    position = safety["joint_position_limits"]
    velocity = safety["joint_velocity_limits"]
    workspace = safety["workspace_limits"]
    return SafetyEnvelope(
        joint_position_limits=tuple(tuple(float(value) for value in position[f"J{index}"]) for index in range(1, 7)),
        joint_velocity_limits=tuple(float(velocity[f"J{index}"]) for index in range(1, 7)),
        hand_position_limits=tuple(float(value) for value in gripper_range),
        workspace_limits_mm={axis: tuple(float(value) for value in workspace[axis]) for axis in ("x", "y", "z")},
        hold_timeout_s=float(safety["command_timeout_ms"]["low_level_servo"]) / 1000.0,
        policy_timeout_s=float(safety["command_timeout_ms"]["policy_inference"]) / 1000.0,
    )

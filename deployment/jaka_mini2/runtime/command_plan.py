"""Turn one safe 12-D policy action into arm and hand command streams."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from deployment.jaka_mini2.safety.envelope import SafetyEnvelope

from .interface import split_vector
from .interpolation import interpolate_position


@dataclass(frozen=True)
class CommandPlan:
    arm_servo_targets: tuple[tuple[float, ...], ...]
    hand_target: tuple[float, ...]


def prepare_command(
    current_position: Sequence[float],
    target_action: Sequence[float],
    safety: SafetyEnvelope,
    *,
    policy_period_s: float = 0.2,
    interpolation_steps: int = 25,
    tcp_xyz_mm: Sequence[float] | None = None,
) -> CommandPlan:
    """Validate one action before producing any hardware command.

    The arm receives 25 interpolated absolute joint targets at 125 Hz. The O6
    receives one native six-value target at policy rate, below its 30 Hz topic
    command limit.
    """
    validated = safety.validate_action(
        target_action,
        current_position=current_position,
        dt_s=policy_period_s,
        tcp_xyz_mm=tcp_xyz_mm,
    )
    current_arm, _ = split_vector(current_position, name="current_position")
    target_arm, target_hand = split_vector(validated, name="target_action")
    arm_targets = interpolate_position(current_arm, target_arm, interpolation_steps)
    return CommandPlan(tuple(arm_targets), target_hand)

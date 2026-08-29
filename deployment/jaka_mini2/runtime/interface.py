"""The 12-D observation/action contract declared in robot_config.yaml."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

ARM_DOF = 6
HAND_DOF = 6
STATE_DIMENSION = ARM_DOF + HAND_DOF
ACTION_DIMENSION = STATE_DIMENSION


def _vector(values: Sequence[float], *, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != STATE_DIMENSION:
        raise ValueError(f"{name} must contain {STATE_DIMENSION} values, got {len(result)}")
    return result


def compose_vector(arm: Sequence[float], hand: Sequence[float], *, name: str = "vector") -> tuple[float, ...]:
    """Combine six arm values followed by six native O6 values."""
    if len(arm) != ARM_DOF or len(hand) != HAND_DOF:
        raise ValueError("arm and hand vectors must each contain 6 values")
    return _vector(tuple(arm) + tuple(hand), name=name)


def split_vector(values: Sequence[float], *, name: str = "vector") -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Split a 12-D vector into (arm[6], hand[6])."""
    vector = _vector(values, name=name)
    return vector[:ARM_DOF], vector[ARM_DOF:]


def build_observation(
    arm_position: Sequence[float],
    hand_position: Sequence[float],
    images: Mapping[str, Any],
    prompt: str,
) -> dict[str, Any]:
    """Build the policy observation with the configured camera role names.

    OpenPI image tensors are intentionally not reshaped here. The camera adapter
    owns conversion to uint8 HWC/CHW as required by the selected policy.
    """
    state = compose_vector(arm_position, hand_position, name="state")
    required = {"base_0_rgb", "left_wrist_0_rgb"}
    missing = required.difference(images)
    if missing:
        raise ValueError(f"missing required camera streams: {sorted(missing)}")
    return {
        "observation/state": state,
        "observation/joint_position": tuple(state[:ARM_DOF]),
        "observation/gripper_position": tuple(state[ARM_DOF:]),
        "observation/base_0_rgb": images["base_0_rgb"],
        "observation/left_wrist_0_rgb": images["left_wrist_0_rgb"],
        "prompt": prompt,
    }

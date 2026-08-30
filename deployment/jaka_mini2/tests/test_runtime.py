from __future__ import annotations

import time

import numpy as np
import pytest

from deployment.jaka_mini2.runtime.command_plan import prepare_command
from deployment.jaka_mini2.runtime.environment import OpenPiJakaEnvironment
from deployment.jaka_mini2.runtime.interface import build_observation
from deployment.jaka_mini2.runtime.interface import compose_vector
from deployment.jaka_mini2.runtime.interface import split_vector
from deployment.jaka_mini2.runtime.interpolation import interpolate_position
from deployment.jaka_mini2.runtime.mock_backends import MockArm
from deployment.jaka_mini2.runtime.mock_backends import MockCameras
from deployment.jaka_mini2.runtime.mock_backends import MockHand
from deployment.jaka_mini2.runtime.policy_adapter import extract_action_chunk
from deployment.jaka_mini2.safety.envelope import SafetyViolationError
from deployment.jaka_mini2.safety.envelope import default_envelope


def test_compose_and_split_12d_contract() -> None:
    vector = compose_vector(range(6), range(10, 16))
    assert vector == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0)
    assert split_vector(vector) == (vector[:6], vector[6:])


def test_build_observation_requires_both_camera_roles() -> None:
    observation = build_observation(
        [0] * 6,
        [100] * 6,
        {"base_0_rgb": "external", "left_wrist_0_rgb": "wrist"},
        "拿起水杯",
    )
    assert len(observation["state"]) == 12
    assert observation["image"]["base_0_rgb"] == "external"
    with pytest.raises(ValueError, match="left_wrist_0_rgb"):
        build_observation([0] * 6, [0] * 6, {"base_0_rgb": object()}, "test")


def test_policy_action_chunk_validation() -> None:
    chunk = extract_action_chunk({"actions": [[0] * 12, [1] * 12]})
    assert len(chunk) == 2
    assert chunk[1][-1] == 1.0
    with pytest.raises(ValueError, match="12"):
        extract_action_chunk({"actions": [[0] * 7]})


def test_5hz_to_125hz_interpolation() -> None:
    points = interpolate_position([0] * 6, [1] * 6, 25)
    assert len(points) == 25
    assert points[0] == pytest.approx([0.04] * 6)
    assert points[-1] == pytest.approx([1.0] * 6)


def test_safety_envelope_is_loaded_from_robot_config() -> None:
    safety = default_envelope()
    assert safety.joint_position_limits[1] == (-1.99, 1.99)
    assert safety.joint_velocity_limits[-1] == 0.35
    assert safety.workspace_limits_mm["z"] == (43.48, 490.15)
    assert safety.hand_position_limits == (0.0, 255.0)


def test_prepare_command_separates_arm_and_hand_rates() -> None:
    safety = default_envelope()
    current = [0] * 6 + [100] * 6
    target = [0.05] * 6 + [110] * 6
    plan = prepare_command(current, target, safety, tcp_xyz_mm=[100, -400, 100])
    assert len(plan.arm_servo_targets) == 25
    assert plan.arm_servo_targets[-1] == pytest.approx([0.05] * 6)
    assert plan.hand_target == (110.0,) * 6


@pytest.mark.parametrize(
    ("target", "message"),
    [
        ([0, 2.0, 0, 0, 0, 0] + [100] * 6, "J2 position"),
        ([0] * 6 + [256, 0, 0, 0, 0, 0], "hand joint 1"),
        ([0.2] * 6 + [100] * 6, "velocity"),
    ],
)
def test_unsafe_actions_are_rejected(target: list[float], message: str) -> None:
    with pytest.raises(SafetyViolationError, match=message):
        prepare_command([0] * 6 + [100] * 6, target, default_envelope())


def test_workspace_and_timeout_are_fail_closed() -> None:
    safety = default_envelope()
    with pytest.raises(SafetyViolationError, match="target TCP"):
        safety.validate_action([0] * 6 + [100] * 6)
    with pytest.raises(SafetyViolationError, match="TCP x"):
        safety.validate_action([0] * 6 + [100] * 6, tcp_xyz_mm=[400, -400, 100])
    with pytest.raises(SafetyViolationError, match="timeout"):
        safety.check_freshness(0.1)


def test_environment_read_only_validates_without_sending() -> None:
    arm = MockArm()
    hand = MockHand()
    environment = OpenPiJakaEnvironment(arm, hand, MockCameras(width=8, height=8), prompt="test")
    observation = environment.get_observation()
    assert observation["hand_state_valid"] is False
    environment.apply_action({"actions": np.asarray([0.05] * 6 + [110] * 6)})
    assert arm.sent == []
    assert hand.sent == []


def test_environment_mock_executes_only_mock_backends() -> None:
    arm = MockArm()
    hand = MockHand()
    environment = OpenPiJakaEnvironment(arm, hand, MockCameras(width=8, height=8), prompt="test", mode="mock")
    environment.get_observation()
    environment.apply_action({"actions": np.asarray([0.05] * 6 + [110] * 6)})
    assert arm.sent == []
    assert hand.sent == [(110.0,) * 6]


def test_environment_hardware_rejects_unverified_hand() -> None:
    arm = MockArm()
    environment = OpenPiJakaEnvironment(
        arm,
        MockHand(state_verified=False),
        MockCameras(width=8, height=8),
        prompt="test",
        mode="hardware",
        motion_confirmation="ENABLE_POLICY_CONTROL",
    )
    environment.get_observation()
    with pytest.raises(PermissionError, match="verified O6"):
        environment.apply_action({"actions": np.asarray([0.05] * 6 + [110] * 6)})
    assert arm.stopped is True
    assert arm.armed is False
    assert environment.is_episode_complete() is True


def test_environment_rejects_stale_policy_action() -> None:
    safety = default_envelope()
    environment = OpenPiJakaEnvironment(MockArm(), MockHand(), MockCameras(width=8, height=8), prompt="test")
    environment.get_observation()
    environment._last_observation_ns = (  # noqa: SLF001 - inject a stale timestamp for fail-closed testing.
        time.monotonic_ns() - int((safety.policy_timeout_s + 0.1) * 1e9)
    )

    with pytest.raises(SafetyViolationError, match="observation timeout"):
        environment.apply_action({"actions": np.asarray([0.05] * 6 + [110] * 6)})


def test_environment_hardware_stops_after_send_failure() -> None:
    class FailingArm(MockArm):
        def send_joint_position(self, target: tuple[float, ...]) -> None:
            raise RuntimeError("simulated servo failure")

    arm = FailingArm()
    environment = OpenPiJakaEnvironment(
        arm,
        MockHand(state_verified=True),
        MockCameras(width=8, height=8),
        prompt="test",
        mode="hardware",
        motion_confirmation="ENABLE_POLICY_CONTROL",
    )
    environment.get_observation()

    with pytest.raises(RuntimeError, match="simulated servo failure"):
        environment.apply_action({"actions": np.asarray([0.05] * 6 + [110] * 6)})
    assert arm.stopped is True
    assert arm.armed is False
    assert environment.is_episode_complete() is True

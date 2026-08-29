"""OpenPI Runtime Environment for the JAKA Mini 2/O6 cell.

Hardware implementations are injected through small protocols so the policy
loop can be tested with recorded or mock sources. The default mode is
``read_only`` and refuses to send either arm or hand commands.
"""

from __future__ import annotations

from collections.abc import Sequence
import contextlib
import time
from typing import Any, Literal, Protocol

from openpi_client.runtime.environment import Environment

from deployment.jaka_mini2.safety.envelope import SafetyEnvelope
from deployment.jaka_mini2.safety.envelope import default_envelope

from .command_plan import prepare_command
from .interface import build_observation
from .policy_adapter import extract_action_chunk


class ArmSource(Protocol):
    def read(self) -> Any: ...

    def forward_kinematics(self, joint_position: tuple[float, ...]) -> tuple[float, ...]: ...

    def arm_motion(self, confirmation: str) -> None: ...

    def send_joint_position(self, target: tuple[float, ...]) -> None: ...

    def stop(self) -> None: ...


class HandSource(Protocol):
    state_verified: bool

    def read(self) -> Sequence[float]: ...

    def send_position(self, target: tuple[float, ...]) -> None: ...


class CameraSource(Protocol):
    def capture(self) -> tuple[dict[str, Any], int]: ...


class OpenPiJakaEnvironment(Environment):
    """Policy-facing environment with read-only, mock, and hardware modes."""

    def __init__(
        self,
        arm: ArmSource,
        hand: HandSource,
        cameras: CameraSource,
        *,
        prompt: str,
        safety: SafetyEnvelope | None = None,
        mode: Literal["read_only", "mock", "hardware"] = "read_only",
        motion_confirmation: str | None = None,
        interpolation_steps: int = 25,
    ) -> None:
        self.arm = arm
        self.hand = hand
        self.cameras = cameras
        self.prompt = prompt
        self.safety = safety or default_envelope()
        self.mode = mode
        self.motion_confirmation = motion_confirmation
        self.interpolation_steps = interpolation_steps
        self._last_arm: tuple[float, ...] | None = None
        self._last_hand: tuple[float, ...] | None = None
        self._last_observation_ns: int | None = None
        self._episode_complete = False

    def reset(self) -> None:
        self._episode_complete = False
        if self.mode == "hardware":
            self.arm.stop()
        self._last_arm = None
        self._last_hand = None
        self._last_observation_ns = None

    def is_episode_complete(self) -> bool:
        return self._episode_complete

    def get_observation(self) -> dict[str, Any]:
        arm_observation = self.arm.read()
        arm_position = tuple(float(value) for value in arm_observation.joint_position)
        if len(arm_position) != 6:
            raise RuntimeError("JAKA backend did not return six joint positions")
        hand_position = tuple(float(value) for value in self.hand.read())
        images, camera_timestamp_ns = self.cameras.capture()
        observation = build_observation(arm_position, hand_position, images, self.prompt)
        observation["timestamp_ns"] = max(camera_timestamp_ns, int(arm_observation.monotonic_ns))
        observation["hand_state_valid"] = self.hand.state_verified
        self._last_arm = arm_position
        self._last_hand = hand_position
        self._last_observation_ns = time.monotonic_ns()
        return observation

    def apply_action(self, action: dict[str, Any]) -> None:
        if self._last_arm is None or self._last_hand is None:
            raise RuntimeError("get_observation() must be called before apply_action()")
        try:
            if self._last_observation_ns is None:
                raise RuntimeError("observation timestamp is unavailable")
            observation_age_s = (time.monotonic_ns() - self._last_observation_ns) / 1e9
            self.safety.check_freshness(observation_age_s, timeout_s=self.safety.policy_timeout_s)
            target = extract_action_chunk(action)[0]
            tcp_pose = self.arm.forward_kinematics(tuple(target[:6]))
            plan = prepare_command(
                self._last_arm + self._last_hand,
                target,
                self.safety,
                tcp_xyz_mm=tcp_pose[:3],
                interpolation_steps=self.interpolation_steps,
            )
            if self.mode == "read_only":
                return
            if self.mode == "mock":
                self._last_arm = plan.arm_servo_targets[-1]
                self._last_hand = plan.hand_target
                self.hand.send_position(plan.hand_target)
                return
            if self.motion_confirmation != "ENABLE_POLICY_CONTROL":
                raise PermissionError("hardware mode requires ENABLE_POLICY_CONTROL")
            if not self.hand.state_verified:
                raise PermissionError("hardware mode requires a verified O6 state source")
            self.arm.arm_motion(self.motion_confirmation)
            for servo_target in plan.arm_servo_targets:
                self.arm.send_joint_position(servo_target)
                time.sleep(0.008)
            self.hand.send_position(plan.hand_target)
            self._last_arm = plan.arm_servo_targets[-1]
            self._last_hand = plan.hand_target
        except Exception:
            if self.mode == "hardware":
                self._episode_complete = True
                with contextlib.suppress(Exception):
                    self.arm.stop()
            raise

    def stop(self) -> None:
        self._episode_complete = True
        self.arm.stop()

"""Deterministic hardware substitutes for offline policy and recorder tests."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .arm_backend import ArmObservation


class MockArm:
    def __init__(self, position: tuple[float, ...] = (0.0,) * 6) -> None:
        self.position = position
        self.sent: list[tuple[float, ...]] = []
        self.stopped = False
        self.armed = False

    def read(self) -> ArmObservation:
        return ArmObservation(self.position, time.monotonic_ns())

    def forward_kinematics(self, joint_position: tuple[float, ...]) -> tuple[float, ...]:
        # A deterministic in-envelope pose for software-only tests.
        return (100.0, -400.0, 100.0, 0.0, 0.0, 0.0)

    def send_joint_position(self, target: tuple[float, ...]) -> None:
        self.position = target
        self.sent.append(target)

    def arm_motion(self, confirmation: str) -> None:
        if confirmation != "ENABLE_POLICY_CONTROL":
            raise PermissionError("mock arm requires ENABLE_POLICY_CONTROL")
        self.armed = True

    def stop(self) -> None:
        self.stopped = True
        self.armed = False


class MockHand:
    def __init__(self, position: tuple[float, ...] = (100.0,) * 6, *, state_verified: bool = False) -> None:
        self.position = position
        self.state_verified = state_verified
        self.sent: list[tuple[float, ...]] = []

    def read(self) -> tuple[float, ...]:
        return self.position

    def send_position(self, target: tuple[float, ...]) -> None:
        self.position = target
        self.sent.append(target)


@dataclass
class MockCameras:
    width: int = 1280
    height: int = 800

    def capture(self) -> tuple[dict[str, np.ndarray], int]:
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return {"base_0_rgb": image.copy(), "left_wrist_0_rgb": image.copy()}, time.monotonic_ns()

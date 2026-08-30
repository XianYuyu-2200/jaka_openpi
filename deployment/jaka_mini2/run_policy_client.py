#!/usr/bin/env python3
"""Run the JAKA/OpenPI client in read-only, mock, or explicitly armed mode."""

from __future__ import annotations

import dataclasses
import pathlib

from openpi_client import action_chunk_broker
from openpi_client import websocket_client_policy
from openpi_client.runtime import runtime
from openpi_client.runtime.agents import policy_agent
import tyro

from deployment.jaka_mini2.camera_adapter import OrbbecDualCameraCapture
from deployment.jaka_mini2.runtime.arm_backend import JakaArmBackend
from deployment.jaka_mini2.runtime.environment import OpenPiJakaEnvironment
from deployment.jaka_mini2.runtime.mock_backends import MockArm
from deployment.jaka_mini2.runtime.mock_backends import MockCameras
from deployment.jaka_mini2.runtime.mock_backends import MockHand


@dataclasses.dataclass
class Args:
    host: str = "127.0.0.1"
    port: int = 8000
    mode: str = "mock"
    arm_library: pathlib.Path = pathlib.Path("/tmp/jaka_mini2_adapter-hardware/libjaka_arm_c_api.so")
    camera_library: pathlib.Path = pathlib.Path("/tmp/jaka-orbbec-camera/liborbbec_dual_camera.so")
    arm_ip: str = "192.168.0.102"
    max_episode_steps: int = 10
    motion_confirmation: str | None = None


class JakaTioHand:
    """Read-only O6 state via JAKA TIO RS485 signal cache.

    The write side intentionally raises: O6 commands remain disabled during
    commissioning even after FC04 state verification succeeds.
    """

    state_verified = True

    def __init__(self, arm: JakaArmBackend) -> None:
        self.arm = arm

    def read(self) -> tuple[float, ...]:
        return self.arm.read_o6_state()

    def send_position(self, target: tuple[float, ...]) -> None:
        raise PermissionError("O6 write path is disabled; JAKA TIO adapter is read-only")


def make_cameras(library: pathlib.Path) -> OrbbecDualCameraCapture:
    return OrbbecDualCameraCapture(library)


def main(args: Args) -> None:
    if args.mode not in {"mock", "read_only", "hardware"}:
        raise ValueError("mode must be mock, read_only, or hardware")
    if args.mode == "mock":
        arm = MockArm()
        hand = MockHand(state_verified=False)
        cameras = MockCameras()
    else:
        arm = JakaArmBackend(args.arm_library, args.arm_ip, allow_motion=args.mode == "hardware")
        arm.connect()
        hand = JakaTioHand(arm)
        cameras = make_cameras(args.camera_library)
        cameras.open()
    try:
        if args.mode == "hardware":
            raise RuntimeError("hardware mode remains disabled while the O6 write path is read-only")
        environment = OpenPiJakaEnvironment(
            arm,
            hand,
            cameras,
            prompt="拿起水杯，将水杯放到咖啡机出水口正下方，用手指按下出水按钮，等待约5秒，然后拿起水杯并递给面前的人。",  # noqa: RUF001
            mode=args.mode,
            motion_confirmation=args.motion_confirmation,
        )
        policy = websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
        agent = policy_agent.PolicyAgent(action_chunk_broker.ActionChunkBroker(policy, action_horizon=16))
        runtime.Runtime(
            environment=environment,
            agent=agent,
            subscribers=[],
            max_hz=5,
            num_episodes=1,
            max_episode_steps=args.max_episode_steps,
        ).run()
    finally:
        if hasattr(cameras, "close"):
            cameras.close()
        if hasattr(arm, "close"):
            arm.close()


if __name__ == "__main__":
    main(tyro.cli(Args))

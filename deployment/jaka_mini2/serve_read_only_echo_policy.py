#!/usr/bin/env python3
"""Serve a zero-write policy-chain check for real JAKA/O6/camera observations.

This is deliberately not a trained model. It validates the exact inference
input contract and repeats the observed state as an absolute-position action
chunk. The read-only client then runs FK and the safety envelope but discards
the action without sending arm or O6 commands.
"""

from __future__ import annotations

import dataclasses
import logging

import numpy as np
from openpi_client import base_policy
import tyro

from openpi.serving import websocket_policy_server

logger = logging.getLogger(__name__)


@dataclasses.dataclass
class Args:
    host: str = "127.0.0.1"
    port: int = 8000
    horizon: int = 16
    width: int = 1280
    height: int = 800


class ReadOnlyEchoPolicy(base_policy.BasePolicy):
    """Validate real observations and return a stationary absolute action."""

    def __init__(self, *, horizon: int = 16, width: int = 1280, height: int = 800) -> None:
        if horizon <= 0:
            raise ValueError("horizon must be positive")
        self.horizon = horizon
        self.image_shape = (height, width, 3)
        self.inference_count = 0

    def infer(self, obs: dict) -> dict:
        state = np.asarray(obs["state"], dtype=np.float32)
        if state.shape != (12,):
            raise ValueError(f"expected real JAKA/O6 state shape (12,), got {state.shape}")
        if not np.isfinite(state).all():
            raise ValueError("real JAKA/O6 state contains a non-finite value")
        if not bool(obs.get("hand_state_valid", False)):
            raise ValueError("O6 state is not verified")

        images = obs["image"]
        image_summaries: list[str] = []
        for role in ("base_0_rgb", "left_wrist_0_rgb"):
            image = np.asarray(images[role])
            if image.shape != self.image_shape or image.dtype != np.uint8:
                raise ValueError(
                    f"{role} must be uint8 {self.image_shape}, got {image.dtype} {image.shape}"
                )
            image_summaries.append(f"{role}={image.shape}/{image.dtype}")

        self.inference_count += 1
        logger.info(
            "validated inference %d: arm=%s O6=%s %s",
            self.inference_count,
            np.array2string(state[:6], precision=5, separator=", "),
            np.array2string(state[6:], precision=1, separator=", "),
            " ".join(image_summaries),
        )
        return {"actions": np.repeat(state[np.newaxis, :], self.horizon, axis=0)}


def main(args: Args) -> None:
    policy = ReadOnlyEchoPolicy(horizon=args.horizon, width=args.width, height=args.height)
    server = websocket_policy_server.WebsocketPolicyServer(
        policy,
        host=args.host,
        port=args.port,
        metadata={
            "policy": "read_only_observation_echo",
            "trained_model": False,
            "action_dim": 12,
            "action_horizon": args.horizon,
            "writes_enabled": False,
        },
    )
    logger.info("Serving read-only observation echo policy on ws://%s:%d", args.host, args.port)
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main(tyro.cli(Args))

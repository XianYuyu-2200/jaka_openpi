from __future__ import annotations

import numpy as np
import pytest

from deployment.jaka_mini2.serve_read_only_echo_policy import ReadOnlyEchoPolicy


def make_observation() -> dict:
    image = np.zeros((8, 10, 3), dtype=np.uint8)
    return {
        "state": np.asarray([0.0] * 6 + [254.0] * 6),
        "image": {"base_0_rgb": image, "left_wrist_0_rgb": image.copy()},
        "hand_state_valid": True,
        "prompt": "test",
    }


def test_echo_policy_validates_and_holds_real_state() -> None:
    policy = ReadOnlyEchoPolicy(horizon=16, width=10, height=8)
    result = policy.infer(make_observation())
    assert result["actions"].shape == (16, 12)
    assert result["actions"][0] == pytest.approx(make_observation()["state"])


def test_echo_policy_rejects_unverified_o6() -> None:
    policy = ReadOnlyEchoPolicy(width=10, height=8)
    observation = make_observation()
    observation["hand_state_valid"] = False
    with pytest.raises(ValueError, match="O6 state is not verified"):
        policy.infer(observation)

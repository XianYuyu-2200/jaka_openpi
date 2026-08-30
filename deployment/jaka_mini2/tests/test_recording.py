from __future__ import annotations

import json
import pathlib
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from deployment.jaka_mini2.runtime.recording import EpisodeRecorder


class FakeDataset:
    def __init__(self, image_root: pathlib.Path) -> None:
        self.image_root = image_root
        self.meta = SimpleNamespace(
            camera_keys=("observation.images.base_0_rgb", "observation.images.left_wrist_0_rgb")
        )
        self.episode_buffer: dict[str, Any] = self._empty_buffer()
        self.saved = 0
        self.cleared = 0
        self.flushed = 0

    def _empty_buffer(self) -> dict[str, Any]:
        return {
            "size": 0,
            "task": [],
            "episode_index": 0,
            "observation.images.base_0_rgb": [],
            "observation.images.left_wrist_0_rgb": [],
            "observation.state": [],
            "action": [],
            "timestamp": [],
        }

    def add_frame(self, frame: dict[str, Any]) -> None:
        index = int(self.episode_buffer["size"])
        self.episode_buffer["size"] = index + 1
        self.episode_buffer["task"].append(frame["task"])
        for key in ("observation.state", "action"):
            self.episode_buffer[key].append(frame[key])
        self.episode_buffer["timestamp"].append(index / 5)
        for key in self.meta.camera_keys:
            path = self.image_root / f"{index}_{key.rsplit('.', maxsplit=1)[-1]}.bin"
            path.write_bytes(np.asarray(frame[key]).tobytes())
            self.episode_buffer[key].append(path)

    def save_episode(self) -> None:
        self.saved += 1

    def clear_episode_buffer(self) -> None:
        self.cleared += 1
        self.episode_buffer = self._empty_buffer()

    def _wait_image_writer(self) -> None:
        self.flushed += 1


def make_observation(*, hand_state_valid: bool) -> dict[str, Any]:
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    return {
        "state": np.asarray([0.0] * 6 + [100.0] * 6),
        "image": {"base_0_rgb": image, "left_wrist_0_rgb": image.copy()},
        "hand_state_valid": hand_state_valid,
    }


def make_recorder(tmp_path: pathlib.Path) -> tuple[EpisodeRecorder, FakeDataset, pathlib.Path]:
    dataset = FakeDataset(tmp_path / "temporary_images")
    dataset.image_root.mkdir()
    status_log = tmp_path / "episode_status.jsonl"
    return EpisodeRecorder(dataset, status_log, "coffee task"), dataset, status_log


def test_successful_verified_episode_is_saved(tmp_path: pathlib.Path) -> None:
    recorder, dataset, status_log = make_recorder(tmp_path)
    recorder.add_frame(make_observation(hand_state_valid=True), np.zeros(12), 1.25)

    assert recorder.finish("success") is True
    assert dataset.saved == 1
    assert dataset.cleared == 0
    assert json.loads(status_log.read_text())["training_eligible"] is True


def test_success_with_unverified_hand_is_excluded(tmp_path: pathlib.Path) -> None:
    recorder, dataset, _ = make_recorder(tmp_path)
    recorder.add_frame(make_observation(hand_state_valid=False), np.zeros(12), 1.25)

    assert recorder.finish("success") is False
    assert dataset.saved == 0
    assert dataset.cleared == 1
    manifests = list((tmp_path / "excluded_episodes" / "success").glob("*/manifest.json"))
    assert len(manifests) == 1
    assert json.loads(manifests[0].read_text())["all_hand_states_valid"] is False


@pytest.mark.parametrize("status", ["failed", "interrupted", "safety_intervention"])
def test_unsuccessful_episode_is_excluded(tmp_path: pathlib.Path, status: str) -> None:
    recorder, dataset, _ = make_recorder(tmp_path)
    recorder.add_frame(make_observation(hand_state_valid=True), np.zeros(12), 1.25)

    assert recorder.finish(status) is False
    assert dataset.saved == 0
    assert dataset.cleared == 1
    assert len(list((tmp_path / "excluded_episodes" / status).glob("*/manifest.json"))) == 1


@pytest.mark.parametrize(
    ("state", "action"),
    [
        (np.zeros(11), np.zeros(12)),
        (np.zeros(12), np.zeros(11)),
    ],
)
def test_invalid_frame_dimensions_are_rejected(tmp_path: pathlib.Path, state: np.ndarray, action: np.ndarray) -> None:
    recorder, dataset, _ = make_recorder(tmp_path)
    observation = make_observation(hand_state_valid=True)
    observation["state"] = state

    with pytest.raises(ValueError, match=r"shape \(12,\)"):
        recorder.add_frame(observation, action, 1.25)
    assert dataset.episode_buffer["size"] == 0


def test_capture_timestamps_must_be_finite_and_monotonic(tmp_path: pathlib.Path) -> None:
    recorder, dataset, _ = make_recorder(tmp_path)
    observation = make_observation(hand_state_valid=True)
    recorder.add_frame(observation, np.zeros(12), 10.0)

    with pytest.raises(ValueError, match="increase monotonically"):
        recorder.add_frame(observation, np.zeros(12), 10.0)
    with pytest.raises(ValueError, match="finite"):
        recorder.add_frame(observation, np.zeros(12), float("nan"))
    assert dataset.episode_buffer["timestamp"] == [0.0]

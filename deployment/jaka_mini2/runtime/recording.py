"""LeRobot episode writer for synchronized JAKA/O6/camera frames."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import pathlib
import shutil
import time
from typing import Any, Literal

import numpy as np

EpisodeStatus = Literal["success", "failed", "interrupted", "safety_intervention"]


@dataclass(frozen=True)
class DatasetSpec:
    repo_id: str = "jaka_mini2_linker_o6_coffee_service_v001"
    fps: int = 5
    robot_type: str = "jaka_mini2_o6"
    width: int = 1280
    height: int = 800

    @property
    def features(self) -> dict[str, dict[str, Any]]:
        image = {"dtype": "image", "shape": (self.height, self.width, 3), "names": ["height", "width", "channel"]}
        return {
            "observation.images.base_0_rgb": image,
            "observation.images.left_wrist_0_rgb": image,
            "observation.state": {"dtype": "float32", "shape": (12,), "names": ["state"]},
            "action": {"dtype": "float32", "shape": (12,), "names": ["action"]},
        }


class EpisodeRecorder:
    """Write successful episodes to LeRobot and log excluded episodes separately."""

    def __init__(
        self,
        dataset: Any,
        status_log: pathlib.Path,
        task: str,
        *,
        excluded_root: pathlib.Path | None = None,
    ) -> None:
        self.dataset = dataset
        self.status_log = status_log
        self.task = task
        self.excluded_root = excluded_root or status_log.parent / "excluded_episodes"
        self.frame_count = 0
        self._all_hand_states_valid = True
        self._last_capture_timestamp_s: float | None = None

    def add_frame(self, observation: dict[str, Any], action: Any, timestamp_s: float) -> None:
        state = np.asarray(observation["state"], dtype=np.float32)
        action_array = np.asarray(action, dtype=np.float32)
        if state.shape != (12,) or action_array.shape != (12,):
            raise ValueError("JAKA/O6 dataset frames require state and action shape (12,)")
        capture_timestamp_s = float(timestamp_s)
        if not math.isfinite(capture_timestamp_s):
            raise ValueError("capture timestamp must be finite")
        if self._last_capture_timestamp_s is not None and capture_timestamp_s <= self._last_capture_timestamp_s:
            raise ValueError("capture timestamps must increase monotonically")
        images = observation["image"]
        self.dataset.add_frame(
            {
                "observation.images.base_0_rgb": np.asarray(images["base_0_rgb"], dtype=np.uint8),
                "observation.images.left_wrist_0_rgb": np.asarray(images["left_wrist_0_rgb"], dtype=np.uint8),
                "observation.state": state,
                "action": action_array,
                "task": self.task,
            }
        )
        self.frame_count += 1
        self._last_capture_timestamp_s = capture_timestamp_s
        self._all_hand_states_valid = self._all_hand_states_valid and bool(observation.get("hand_state_valid", False))

    def finish(self, status: EpisodeStatus) -> bool:
        eligible = status == "success" and self.frame_count > 0 and self._all_hand_states_valid
        if eligible:
            self.dataset.save_episode()
        else:
            if self.frame_count > 0:
                self._archive_excluded(status)
            self.dataset.clear_episode_buffer()
        self.status_log.parent.mkdir(parents=True, exist_ok=True)
        with self.status_log.open("a", encoding="utf-8") as file:
            file.write(json.dumps({"status": status, "frames": self.frame_count, "training_eligible": eligible}) + "\n")
        self.frame_count = 0
        self._all_hand_states_valid = True
        self._last_capture_timestamp_s = None
        return eligible

    def _archive_excluded(self, status: EpisodeStatus) -> None:
        """Preserve failed/interrupted data outside the training dataset."""
        self.dataset._wait_image_writer()  # noqa: SLF001 - LeRobot exposes no public flush API.
        buffer = self.dataset.episode_buffer
        episode_name = f"{time.time_ns()}_{status}"
        destination = self.excluded_root / status / episode_name
        destination.mkdir(parents=True, exist_ok=False)
        arrays: dict[str, np.ndarray] = {}
        for key, values in buffer.items():
            if key in {"size", "task", "episode_index"} or key in self.dataset.meta.camera_keys:
                continue
            if values:
                arrays[key] = np.asarray(values)
        np.savez_compressed(destination / "data.npz", **arrays)
        for key in self.dataset.meta.camera_keys:
            image_dir = destination / key
            image_dir.mkdir(parents=True, exist_ok=True)
            for source in buffer[key]:
                source_path = pathlib.Path(source)
                shutil.copy2(source_path, image_dir / source_path.name)
                source_path.unlink(missing_ok=True)
        (destination / "manifest.json").write_text(
            json.dumps(
                {
                    "status": status,
                    "frames": self.frame_count,
                    "task": self.task,
                    "training_eligible": False,
                    "all_hand_states_valid": self._all_hand_states_valid,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )


def create_lerobot_dataset(
    spec: DatasetSpec,
    root: pathlib.Path,
    *,
    use_videos: bool = True,
    image_writer_threads: int = 0,
) -> Any:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    return LeRobotDataset.create(
        repo_id=spec.repo_id,
        root=root,
        fps=spec.fps,
        robot_type=spec.robot_type,
        features=spec.features,
        use_videos=use_videos,
        image_writer_threads=image_writer_threads,
    )


def open_or_create_lerobot_dataset(
    spec: DatasetSpec,
    root: pathlib.Path,
    *,
    use_videos: bool = True,
    image_writer_threads: int = 0,
) -> Any:
    """Open an existing local dataset or create it on the first session."""
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        info = json.loads(info_path.read_text(encoding="utf-8"))
        if int(info.get("total_episodes", 0)) == 0:
            image_root = root / "images"
            if (
                any(root.rglob("*.parquet"))
                or any(root.rglob("*.mp4"))
                or (image_root.exists() and any(path.is_file() for path in image_root.rglob("*")))
            ):
                raise RuntimeError(f"empty dataset metadata has recoverable files; inspect before retrying: {root}")
            shutil.rmtree(root)
            return create_lerobot_dataset(
                spec,
                root,
                use_videos=use_videos,
                image_writer_threads=image_writer_threads,
            )
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

        dataset = LeRobotDataset(spec.repo_id, root=root)
        if image_writer_threads:
            dataset.start_image_writer(num_threads=image_writer_threads)
        return dataset
    return create_lerobot_dataset(
        spec,
        root,
        use_videos=use_videos,
        image_writer_threads=image_writer_threads,
    )

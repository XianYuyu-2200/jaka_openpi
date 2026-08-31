#!/usr/bin/env python3
"""Record synchronized .101 -> .102/O6 teleoperation demonstrations.

The follower owns the only .102 JAKA SDK session and publishes snapshots over
Unix datagrams. This process owns the cameras, episode lifecycle, and dataset;
it never logs into either robot.
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import select
import signal
import socket
import sys
import termios
import time
import tty

from deployment.jaka_mini2.camera_adapter import OrbbecDualCameraCapture
from deployment.jaka_mini2.runtime.hand_presets import HandPresetState
from deployment.jaka_mini2.runtime.hand_presets import load_hand_presets
from deployment.jaka_mini2.runtime.recording import DatasetSpec
from deployment.jaka_mini2.runtime.recording import EpisodeRecorder
from deployment.jaka_mini2.runtime.recording import open_or_create_lerobot_dataset
from deployment.jaka_mini2.runtime.teleop_recording_ipc import SnapshotReceiver
from deployment.jaka_mini2.runtime.teleop_recording_ipc import encode_o6_command

LOGGER = logging.getLogger(__name__)
TASK = "拿起水杯，将水杯放到咖啡机出水口正下方，用手指按下出水按钮，等待约5秒，然后拿起水杯并递给面前的人。"  # noqa: RUF001


def _stop(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record-socket", type=pathlib.Path, default=pathlib.Path("/tmp/jaka_teleop_record.sock"))
    parser.add_argument("--command-socket", type=pathlib.Path, required=True)
    parser.add_argument(
        "--camera-library", type=pathlib.Path, default=pathlib.Path("/tmp/jaka-orbbec-camera/liborbbec_dual_camera.so")
    )
    parser.add_argument("--dataset-root", type=pathlib.Path, default=pathlib.Path("local_runtime/datasets"))
    parser.add_argument("--repo-id", default=DatasetSpec().repo_id)
    parser.add_argument("--task", default=TASK)
    parser.add_argument(
        "--event-log", type=pathlib.Path, default=pathlib.Path("local_runtime/logs/teleop_record_events.jsonl")
    )
    parser.add_argument(
        "--status-log", type=pathlib.Path, default=pathlib.Path("local_runtime/logs/teleop_episode_status.jsonl")
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    presets = load_hand_presets(pathlib.Path("deployment/jaka_mini2/config/o6_hand_presets.yaml"))
    preset_state = HandPresetState(presets)
    spec = DatasetSpec(repo_id=args.repo_id)
    dataset = open_or_create_lerobot_dataset(
        spec,
        args.dataset_root / args.repo_id,
        use_videos=True,
        image_writer_threads=4,
    )
    recorder = EpisodeRecorder(dataset, args.status_log, args.task)
    args.event_log.parent.mkdir(parents=True, exist_ok=True)
    event_log = args.event_log.open("a", encoding="utf-8")
    command_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    cameras = OrbbecDualCameraCapture(args.camera_library)
    receiver = SnapshotReceiver(args.record_socket)
    cameras.open()
    old_terminal = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    recording = False
    frame_index = 0
    latest = None

    def log_event(event: dict[str, object]) -> None:
        event["timestamp_ns"] = time.time_ns()
        event_log.write(json.dumps(event, ensure_ascii=False) + "\n")
        event_log.flush()

    def finish(status: str) -> None:
        nonlocal recording, frame_index
        if not recording:
            LOGGER.info("no active episode")
            return
        eligible = recorder.finish(status)  # type: ignore[arg-type]
        LOGGER.info("episode %s: frames=%d training_eligible=%s", status, frame_index, eligible)
        log_event({"event": "episode_finish", "status": status, "frames": frame_index, "training_eligible": eligible})
        recording = False
        frame_index = 0

    LOGGER.info("Recording controls: r start, s success, f failed, 1-5 O6, Space hold-current, q quit")
    LOGGER.info("Dataset: %s", args.dataset_root / args.repo_id)
    try:
        next_capture = time.monotonic()
        while True:
            wait_s = max(0.0, next_capture - time.monotonic())
            readable, _, _ = select.select([sys.stdin], [], [], min(wait_s, 0.05))
            if readable:
                key = sys.stdin.read(1)
                if key.lower() == "q":
                    break
                if key == "r":
                    if recording:
                        LOGGER.warning("episode already active")
                    elif latest is None:
                        LOGGER.warning("episode not started: wait for the first real robot snapshot")
                    elif (time.monotonic_ns() - latest.monotonic_ns) / 1e9 > 0.6:
                        LOGGER.warning("episode not started: real robot snapshot is stale")
                    elif not latest.arm_state_valid or not latest.hand_state_valid:
                        LOGGER.warning("episode not started: real arm/O6 state is invalid")
                    else:
                        recording = True
                        frame_index = 0
                        log_event({"event": "episode_start"})
                        LOGGER.info("episode started")
                    continue
                if key == "s":
                    finish("success")
                    continue
                if key == "f":
                    finish("failed")
                    continue
                preset = presets.get(key)
                if key == " ":
                    if latest is None:
                        LOGGER.warning("Space ignored: no O6 state snapshot yet")
                        continue
                    preset = preset_state.select("space", current_state=latest.actual_hand)
                elif preset is not None:
                    try:
                        preset = preset_state.select(key)
                    except ValueError as exc:
                        LOGGER.warning("%s", exc)
                        continue
                if preset is not None and preset.target is not None:
                    command_socket.sendto(
                        encode_o6_command(preset.target, time.monotonic_ns()), str(args.command_socket)
                    )
                    log_event(
                        {
                            "event": "o6_command",
                            "key": key,
                            "preset": preset.name,
                            "target": list(preset.target),
                            "episode_active": recording,
                        }
                    )
                    LOGGER.info("O6 %s (%s): %s", key, preset.name, preset.target)

            now = time.monotonic()
            if now < next_capture:
                continue
            next_capture += 0.2
            if next_capture < now:
                next_capture = now + 0.2
            snapshot = receiver.latest()
            if snapshot is not None:
                latest = snapshot
            if not recording or latest is None:
                continue
            snapshot_age_s = (time.monotonic_ns() - latest.monotonic_ns) / 1e9
            if snapshot_age_s > 0.6:
                raise RuntimeError(f"teleoperation snapshot timeout: {snapshot_age_s:.3f}s")
            if not latest.arm_state_valid or not latest.hand_state_valid:
                raise RuntimeError(
                    "invalid real robot snapshot: "
                    f"arm_valid={latest.arm_state_valid} hand_valid={latest.hand_state_valid}"
                )
            images, _camera_timestamp_ns = cameras.capture()
            observation = {
                "state": tuple(latest.actual_arm) + tuple(latest.actual_hand),
                "image": images,
                "hand_state_valid": latest.hand_state_valid,
            }
            action = tuple(latest.action_arm) + tuple(latest.action_hand)
            recorder.add_frame(observation, action, frame_index / spec.fps)
            frame_index += 1
            LOGGER.info(
                "episode frame=%d snapshot=%d dragging=%s", frame_index, latest.sequence, latest.operator_dragging
            )
    except KeyboardInterrupt:
        LOGGER.info("recording interrupted")
    finally:
        if recording:
            finish("interrupted")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        cameras.close()
        receiver.close()
        command_socket.close()
        event_log.close()
        if getattr(dataset, "image_writer", None) is not None:
            dataset.stop_image_writer()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main()

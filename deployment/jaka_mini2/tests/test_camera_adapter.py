from __future__ import annotations

import pathlib

import numpy as np
import pytest

from deployment.jaka_mini2.camera_adapter.v4l2_capture import CameraFrame
from deployment.jaka_mini2.camera_adapter.v4l2_capture import DualCameraCapture
from deployment.jaka_mini2.camera_adapter.v4l2_capture import video_nodes_by_serial


class FakeCamera:
    def __init__(self, role: str, timestamp_ns: int, *, fail_open: bool = False) -> None:
        self.role = role
        self.timestamp_ns = timestamp_ns
        self.fail_open = fail_open
        self.opened = False
        self.closed = False

    def open(self) -> None:
        if self.fail_open:
            raise RuntimeError("simulated camera open failure")
        self.opened = True

    def read(self) -> CameraFrame:
        return CameraFrame(self.role, np.zeros((8, 8, 3), dtype=np.uint8), self.timestamp_ns)

    def grab(self) -> int:
        return self.timestamp_ns

    def retrieve(self, timestamp_ns: int) -> CameraFrame:
        return CameraFrame(self.role, np.zeros((8, 8, 3), dtype=np.uint8), timestamp_ns)

    def close(self) -> None:
        self.closed = True


def test_video_nodes_are_grouped_by_usb_serial(tmp_path: pathlib.Path) -> None:
    usb = tmp_path / "devices" / "2-3"
    target = usb / "video4linux" / "video7"
    target.mkdir(parents=True)
    (usb / "serial").write_text("CP28563000AJ\n")
    sys_video = tmp_path / "class" / "video4linux"
    sys_video.mkdir(parents=True)
    (sys_video / "video7").symlink_to(target)

    grouped = video_nodes_by_serial(sys_video)

    assert grouped == {"CP28563000AJ": (pathlib.Path("/dev/video7"),)}


def test_dual_capture_returns_both_roles_with_bounded_skew() -> None:
    cameras = (
        FakeCamera("base_0_rgb", 1_000_000_000),
        FakeCamera("left_wrist_0_rgb", 1_010_000_000),
    )
    capture = DualCameraCapture(cameras, max_skew_ms=20.0)  # type: ignore[arg-type]

    images, timestamp_ns = capture.capture()

    assert set(images) == {"base_0_rgb", "left_wrist_0_rgb"}
    assert timestamp_ns == 1_010_000_000
    assert capture.last_skew_ns == 10_000_000


def test_dual_capture_rejects_excessive_skew() -> None:
    cameras = (
        FakeCamera("base_0_rgb", 1_000_000_000),
        FakeCamera("left_wrist_0_rgb", 1_025_000_000),
    )
    capture = DualCameraCapture(cameras, max_skew_ms=20.0)  # type: ignore[arg-type]

    with pytest.raises(RuntimeError, match="timestamp skew 25.000 ms"):
        capture.capture()


def test_dual_camera_open_rolls_back_first_camera() -> None:
    first = FakeCamera("base_0_rgb", 0)
    second = FakeCamera("left_wrist_0_rgb", 0, fail_open=True)
    capture = DualCameraCapture((first, second))  # type: ignore[arg-type]

    with pytest.raises(RuntimeError, match="simulated camera open failure"):
        capture.open()
    assert first.opened is True
    assert first.closed is True

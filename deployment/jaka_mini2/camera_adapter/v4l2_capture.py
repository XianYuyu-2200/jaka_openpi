"""Serial-bound V4L2 capture for the two Orbbec color streams.

This adapter deliberately uses OpenCV's read-only capture API. It never changes
camera controls; resolution/FPS are checked against the returned frames and
reported to the caller.
"""

from __future__ import annotations

import concurrent.futures
from dataclasses import dataclass
import pathlib
import time
from typing import Any

import cv2
import numpy as np

from deployment.jaka_mini2.runtime.interface import build_observation

SYS_VIDEO = pathlib.Path("/sys/class/video4linux")
DEV = pathlib.Path("/dev")


@dataclass(frozen=True)
class CameraSpec:
    role: str
    model: str
    serial_number: str
    width: int = 1280
    height: int = 800
    fps: int = 30
    pixel_format: str = "MJPG"


@dataclass(frozen=True)
class CameraFrame:
    role: str
    image_rgb: np.ndarray
    monotonic_ns: int


def _read(path: pathlib.Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return None


def video_nodes_by_serial(sys_video: pathlib.Path = SYS_VIDEO) -> dict[str, tuple[pathlib.Path, ...]]:
    """Return video nodes grouped by their parent USB serial number."""
    grouped: dict[str, list[pathlib.Path]] = {}
    for entry in sys_video.glob("video*") if sys_video.exists() else ():
        usb_device = entry.resolve()
        while usb_device != usb_device.parent:
            serial = _read(usb_device / "serial")
            if serial:
                grouped.setdefault(serial, []).append(DEV / entry.name)
                break
            usb_device = usb_device.parent
    return {
        serial: tuple(sorted(nodes, key=lambda node: int(node.name.removeprefix("video"))))
        for serial, nodes in grouped.items()
    }


class SerialBoundCamera:
    """Open the first V4L2 node belonging to a configured USB serial."""

    def __init__(self, spec: CameraSpec, *, sys_video: pathlib.Path = SYS_VIDEO) -> None:
        self.spec = spec
        nodes = video_nodes_by_serial(sys_video).get(spec.serial_number, ())
        if not nodes:
            raise FileNotFoundError(f"no V4L2 node found for {spec.model} {spec.serial_number}")
        self.candidate_devices = nodes
        self.device: pathlib.Path | None = None
        self._capture: Any = None

    def open(self) -> None:
        errors: list[str] = []
        for device in self.candidate_devices:
            capture = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                errors.append(f"{device}: open failed")
                continue
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.spec.pixel_format))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
            capture.set(cv2.CAP_PROP_FPS, self.spec.fps)
            ok, image = capture.read()
            actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace")
            actual_fps = capture.get(cv2.CAP_PROP_FPS)
            if (
                ok
                and image is not None
                and image.shape == (self.spec.height, self.spec.width, 3)
                and actual_fourcc == self.spec.pixel_format
                and abs(actual_fps - self.spec.fps) < 0.5
            ):
                self.device = device
                self._capture = capture
                return
            shape = None if image is None else image.shape
            errors.append(f"{device}: incompatible frame {shape}, format={actual_fourcc!r}, fps={actual_fps:g}")
            capture.release()
        raise RuntimeError(f"no compatible color stream for {self.spec.role}: {'; '.join(errors)}")

    def read(self) -> CameraFrame:
        timestamp_ns = self.grab()
        return self.retrieve(timestamp_ns)

    def grab(self) -> int:
        if self._capture is None:
            raise RuntimeError("camera is not open")
        if not self._capture.grab():
            raise RuntimeError(f"failed to grab frame from {self.device}")
        return time.monotonic_ns()

    def retrieve(self, timestamp_ns: int) -> CameraFrame:
        if self._capture is None:
            raise RuntimeError("camera is not open")
        ok, image_bgr = self._capture.retrieve()
        if not ok or image_bgr is None:
            raise RuntimeError(f"failed to retrieve frame from {self.device}")
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        if image_rgb.shape[:2] != (self.spec.height, self.spec.width):
            raise RuntimeError(
                f"{self.spec.role} returned {image_rgb.shape[1]}x{image_rgb.shape[0]}, "
                f"expected {self.spec.width}x{self.spec.height}"
            )
        return CameraFrame(self.spec.role, image_rgb, timestamp_ns)

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
            self.device = None


class DualCameraCapture:
    """Synchronous two-camera capture with a bounded timestamp skew."""

    def __init__(self, cameras: tuple[SerialBoundCamera, SerialBoundCamera], max_skew_ms: float = 20.0) -> None:
        self.cameras = cameras
        self.max_skew_ns = int(max_skew_ms * 1e6)
        self.last_skew_ns: int | None = None
        self._executor: concurrent.futures.ThreadPoolExecutor | None = None

    def open(self, *, warmup_frames: int = 5) -> None:
        opened: list[SerialBoundCamera] = []
        try:
            for camera in self.cameras:
                camera.open()
                opened.append(camera)
            for _ in range(warmup_frames):
                self._read_parallel()
        except Exception:
            for camera in reversed(opened):
                camera.close()
            raise

    def _read_parallel(self) -> list[CameraFrame]:
        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="orbbec-capture")
        grab_futures = [self._executor.submit(camera.grab) for camera in self.cameras]
        timestamps = [future.result() for future in grab_futures]
        retrieve_futures = [
            self._executor.submit(camera.retrieve, timestamp_ns)
            for camera, timestamp_ns in zip(self.cameras, timestamps, strict=True)
        ]
        return [future.result() for future in retrieve_futures]

    def capture(self) -> tuple[dict[str, np.ndarray], int]:
        frames = self._read_parallel()
        timestamps = [frame.monotonic_ns for frame in frames]
        skew = max(timestamps) - min(timestamps)
        self.last_skew_ns = skew
        if skew > self.max_skew_ns:
            raise RuntimeError(f"camera timestamp skew {skew / 1e6:.3f} ms exceeds limit")
        return {frame.role: frame.image_rgb for frame in frames}, max(timestamps)

    def close(self) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._executor = None
        for camera in self.cameras:
            camera.close()

    def __enter__(self) -> DualCameraCapture:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def build_camera_observation(
    arm_position: tuple[float, ...],
    hand_position: tuple[float, ...],
    images: dict[str, np.ndarray],
    prompt: str,
) -> dict[str, Any]:
    return build_observation(arm_position, hand_position, images, prompt)

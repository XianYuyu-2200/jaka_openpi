"""ctypes wrapper around the official Orbbec SDK dual-color backend."""

from __future__ import annotations

import ctypes
import pathlib
import time

import numpy as np


class OrbbecCameraError(RuntimeError):
    pass


class OrbbecDualCameraCapture:
    """Serial-bound true-color capture with timestamp-based frame pairing."""

    def __init__(
        self,
        library: pathlib.Path,
        *,
        base_serial: str = "CP28563000AJ",
        wrist_serial: str = "CV2L761000KT",
        width: int = 1280,
        height: int = 800,
        fps: int = 30,
        max_skew_ms: float = 20.0,
    ) -> None:
        self.width = width
        self.height = height
        self.last_skew_ms: float | None = None
        self.last_system_timestamp_us: int | None = None
        self._lib = ctypes.CDLL(str(library))
        self._configure_api()
        self._handle = self._lib.orbbec_dual_camera_create(
            base_serial.encode(),
            wrist_serial.encode(),
            width,
            height,
            fps,
            int(max_skew_ms * 1000),
        )
        if not self._handle:
            raise OrbbecCameraError("failed to create Orbbec dual-camera backend")
        self._opened = False

    def _configure_api(self) -> None:
        byte_pointer = ctypes.POINTER(ctypes.c_uint8)
        self._lib.orbbec_dual_camera_create.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint64,
        ]
        self._lib.orbbec_dual_camera_create.restype = ctypes.c_void_p
        self._lib.orbbec_dual_camera_destroy.argtypes = [ctypes.c_void_p]
        self._lib.orbbec_dual_camera_open.argtypes = [ctypes.c_void_p]
        self._lib.orbbec_dual_camera_close.argtypes = [ctypes.c_void_p]
        self._lib.orbbec_dual_camera_capture.argtypes = [
            ctypes.c_void_p,
            byte_pointer,
            ctypes.c_size_t,
            byte_pointer,
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.orbbec_dual_camera_last_error.argtypes = [ctypes.c_void_p]
        self._lib.orbbec_dual_camera_last_error.restype = ctypes.c_char_p

    def _check(self, result: int, operation: str) -> None:
        if result == 0:
            return
        message = self._lib.orbbec_dual_camera_last_error(self._handle)
        detail = message.decode(errors="replace") if message else "unknown error"
        raise OrbbecCameraError(f"{operation} failed: {detail}")

    def open(self) -> None:
        if self._opened:
            return
        self._check(self._lib.orbbec_dual_camera_open(self._handle), "open")
        self._opened = True

    def capture(self) -> tuple[dict[str, np.ndarray], int]:
        if not self._opened:
            raise OrbbecCameraError("dual camera is not open")
        shape = (self.height, self.width, 3)
        base = np.empty(shape, dtype=np.uint8)
        wrist = np.empty(shape, dtype=np.uint8)
        timestamp_us = ctypes.c_uint64()
        skew_us = ctypes.c_uint64()
        self._check(
            self._lib.orbbec_dual_camera_capture(
                self._handle,
                base.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                base.nbytes,
                wrist.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                wrist.nbytes,
                ctypes.byref(timestamp_us),
                ctypes.byref(skew_us),
            ),
            "capture",
        )
        self.last_system_timestamp_us = timestamp_us.value
        self.last_skew_ms = skew_us.value / 1000.0
        return {"base_0_rgb": base, "left_wrist_0_rgb": wrist}, time.monotonic_ns()

    def close(self) -> None:
        if not self._handle:
            return
        if self._opened:
            self._check(self._lib.orbbec_dual_camera_close(self._handle), "close")
            self._opened = False
        self._lib.orbbec_dual_camera_destroy(self._handle)
        self._handle = None

    def __enter__(self) -> OrbbecDualCameraCapture:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.orbbec_dual_camera_destroy(self._handle)
            self._handle = None

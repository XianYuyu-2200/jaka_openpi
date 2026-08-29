#!/usr/bin/env python3
"""Open both configured Orbbec RGB streams and verify synchronized frames."""

from __future__ import annotations

import argparse
import pathlib

import numpy as np

from deployment.jaka_mini2.camera_adapter.orbbec_capture import OrbbecDualCameraCapture


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--max-skew-ms", type=float, default=20.0)
    parser.add_argument(
        "--library",
        type=pathlib.Path,
        default=pathlib.Path("/tmp/jaka-orbbec-camera/liborbbec_dual_camera.so"),
    )
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    max_observed_skew_ns = 0
    with OrbbecDualCameraCapture(args.library, max_skew_ms=args.max_skew_ms) as capture:
        for _ in range(args.frames):
            images, _ = capture.capture()
            for role, image in images.items():
                if image.shape != (800, 1280, 3) or image.dtype != np.uint8:
                    raise RuntimeError(f"{role} returned {image.shape} {image.dtype}, expected 800x1280 RGB uint8")
            max_observed_skew_ns = max(max_observed_skew_ns, int((capture.last_skew_ms or 0) * 1e6))
        print("base_0_rgb: Orbbec Gemini 335L CP28563000AJ, MJPG -> RGB")
        print("left_wrist_0_rgb: Orbbec Gemini 305 CV2L761000KT, YUYV -> RGB")
    print(f"capture verification: {args.frames} frame pairs passed")
    print(f"maximum observed timestamp skew: {max_observed_skew_ns / 1e6:.3f} ms")


if __name__ == "__main__":
    main()

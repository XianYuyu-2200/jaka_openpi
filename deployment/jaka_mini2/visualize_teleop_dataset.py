#!/usr/bin/env python3
"""Play a recorded JAKA/O6 episode with both camera views and state/action."""

from __future__ import annotations

import argparse
import pathlib

import cv2
import numpy as np
import pyarrow.parquet as pq


def image_from_cell(cell: object) -> np.ndarray:
    value = cell.as_py() if hasattr(cell, "as_py") else cell
    if not isinstance(value, dict):
        raise TypeError(f"unexpected image cell: {type(value)!r}")
    encoded = value.get("bytes")
    if encoded is None:
        path = value.get("path")
        if not path:
            raise RuntimeError("image cell has neither bytes nor path")
        encoded = pathlib.Path(path).read_bytes()
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("could not decode image")
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=pathlib.Path,
                        default=pathlib.Path("local_runtime/datasets/jaka_mini2_linker_o6_coffee_service_v001"))
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--scale", type=float, default=0.5)
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    if args.episode < 0 or args.fps <= 0 or args.scale <= 0:
        parser.error("episode must be non-negative; fps and scale must be positive")
    path = args.dataset_root / "data" / "chunk-000" / f"episode_{args.episode:06d}.parquet"
    if not path.is_file():
        raise SystemExit(f"episode file not found: {path}")
    table = pq.read_table(path)
    window = "JAKA/O6 episode viewer | Space pause | n next | q/Esc quit"
    paused = False
    index = 0
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while index < table.num_rows:
            base = image_from_cell(table["observation.images.base_0_rgb"][index])
            wrist = image_from_cell(table["observation.images.left_wrist_0_rgb"][index])
            base = cv2.resize(base, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
            wrist = cv2.resize(wrist, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
            height = max(base.shape[0], wrist.shape[0])
            if base.shape[0] != height:
                base = cv2.copyMakeBorder(base, 0, height - base.shape[0], 0, 0, cv2.BORDER_CONSTANT)
            if wrist.shape[0] != height:
                wrist = cv2.copyMakeBorder(wrist, 0, height - wrist.shape[0], 0, 0, cv2.BORDER_CONSTANT)
            frame = np.hstack((base, wrist))
            state = np.asarray(table["observation.state"][index].as_py(), dtype=np.float32)
            action = np.asarray(table["action"][index].as_py(), dtype=np.float32)
            lines = [
                f"episode={args.episode} frame={index}/{table.num_rows - 1} time={float(table['timestamp'][index].as_py()):.1f}s",
                "state arm(rad): " + " ".join(f"{x:.2f}" for x in state[:6]),
                "state O6:        " + " ".join(f"{x:.0f}" for x in state[6:]),
                "action arm(rad): " + " ".join(f"{x:.2f}" for x in action[:6]),
                "action O6:       " + " ".join(f"{x:.0f}" for x in action[6:]),
            ]
            for line_no, line in enumerate(lines):
                cv2.putText(frame, line, (12, 25 + line_no * 23), cv2.FONT_HERSHEY_SIMPLEX,
                             0.55, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow(window, frame)
            key = cv2.waitKey(max(1, int(1000 / args.fps))) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paused = not paused
            elif key == ord("n"):
                index = min(index + 1, table.num_rows - 1)
            elif not paused:
                index += 1
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

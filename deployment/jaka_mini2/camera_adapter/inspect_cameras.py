#!/usr/bin/env python3
"""Read-only sysfs inspection for the configured Orbbec cameras."""

from __future__ import annotations

from dataclasses import dataclass
import pathlib

SYS_USB = pathlib.Path("/sys/bus/usb/devices")
SYS_VIDEO = pathlib.Path("/sys/class/video4linux")
DEV = pathlib.Path("/dev")


@dataclass(frozen=True)
class ExpectedCamera:
    role: str
    model: str
    vendor: str
    product: str
    serial: str


EXPECTED = (
    ExpectedCamera("base_0_rgb", "Orbbec Gemini 335L", "2bc5", "0804", "CP28563000AJ"),
    ExpectedCamera("left_wrist_0_rgb", "Orbbec Gemini 305", "2bc5", "0840", "CV2L761000KT"),
)


def read_text(path: pathlib.Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return None


def video_nodes_for_usb_path(usb_path: str) -> tuple[str, ...]:
    nodes: list[str] = []
    if not SYS_VIDEO.exists():
        return ()
    for video in SYS_VIDEO.glob("video*"):
        try:
            target = video.resolve()
        except OSError:
            continue
        if f"/{usb_path}/" in f"{target}/":
            nodes.append(video.name)
    return tuple(sorted(nodes, key=lambda value: int(value.removeprefix("video"))))


def main() -> int:
    found: dict[str, pathlib.Path] = {}
    for device in SYS_USB.iterdir() if SYS_USB.exists() else ():
        serial = read_text(device / "serial")
        if serial:
            found[serial] = device

    failed = False
    for expected in EXPECTED:
        device = found.get(expected.serial)
        if device is None:
            print(f"MISSING role={expected.role} model={expected.model} serial={expected.serial}")
            failed = True
            continue
        vendor = read_text(device / "idVendor")
        product = read_text(device / "idProduct")
        speed = read_text(device / "speed") or "unknown"
        identity_ok = vendor == expected.vendor and product == expected.product
        nodes = video_nodes_for_usb_path(device.name)
        accessible = tuple(node for node in nodes if (DEV / node).exists())
        status = "ENUMERATED" if identity_ok else "IDENTITY_MISMATCH"
        print(
            f"{status} role={expected.role} model={expected.model} serial={expected.serial} "
            f"usb={device.name} speed_mbps={speed} sysfs_video={','.join(nodes) or '-'} "
            f"accessible_video={','.join(accessible) or '-'}"
        )
        failed = failed or not identity_ok

    if failed:
        print("camera enumeration status: FAILED")
        return 1
    print("camera enumeration status: READY; frame capture is not tested")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

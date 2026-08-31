"""Local IPC for synchronized teleoperation demonstration snapshots."""

from __future__ import annotations

from dataclasses import dataclass
import pathlib
import socket
import struct

SNAPSHOT_MAGIC = 0x31434552
SNAPSHOT_VERSION = 1
SNAPSHOT_FORMAT = "<IHHQQ6d6d6H6HiiB3x"
SNAPSHOT_SIZE = struct.calcsize(SNAPSHOT_FORMAT)

O6_COMMAND_MAGIC = 0x364F434A
O6_COMMAND_VERSION = 1
O6_COMMAND_FORMAT = "<IHHQ6H"


@dataclass(frozen=True)
class TeleopSnapshot:
    sequence: int
    monotonic_ns: int
    actual_arm: tuple[float, ...]
    action_arm: tuple[float, ...]
    actual_hand: tuple[float, ...]
    action_hand: tuple[float, ...]
    arm_state_valid: bool
    hand_state_valid: bool
    operator_dragging: bool


def decode_snapshot(packet: bytes) -> TeleopSnapshot:
    if len(packet) != SNAPSHOT_SIZE:
        raise ValueError(f"snapshot packet must contain {SNAPSHOT_SIZE} bytes")
    values = struct.unpack(SNAPSHOT_FORMAT, packet)
    magic, version, size, sequence, monotonic_ns = values[:5]
    if magic != SNAPSHOT_MAGIC or version != SNAPSHOT_VERSION or size != SNAPSHOT_SIZE:
        raise ValueError("invalid teleoperation snapshot header")
    return TeleopSnapshot(
        sequence=sequence,
        monotonic_ns=monotonic_ns,
        actual_arm=tuple(values[5:11]),
        action_arm=tuple(values[11:17]),
        actual_hand=tuple(float(value) for value in values[17:23]),
        action_hand=tuple(float(value) for value in values[23:29]),
        arm_state_valid=values[29] == 0,
        hand_state_valid=values[30] == 0,
        operator_dragging=bool(values[31]),
    )


class SnapshotReceiver:
    def __init__(self, path: pathlib.Path) -> None:
        self.path = path
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.path.unlink(missing_ok=True)
        self.socket.bind(str(self.path))
        self.socket.setblocking(False)  # noqa: FBT003

    def latest(self) -> TeleopSnapshot | None:
        latest = None
        while True:
            try:
                packet = self.socket.recv(SNAPSHOT_SIZE + 1)
            except BlockingIOError:
                break
            latest = decode_snapshot(packet)
        return latest

    def close(self) -> None:
        self.socket.close()
        self.path.unlink(missing_ok=True)

    def __enter__(self) -> SnapshotReceiver:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def encode_o6_command(target: tuple[float, ...], monotonic_ns: int) -> bytes:
    if len(target) != 6:
        raise ValueError("O6 target must contain six values")
    integer_target = tuple(int(value) for value in target)
    if any(
        float(integer) != value or integer < 0 or integer > 255
        for integer, value in zip(integer_target, target, strict=True)
    ):
        raise ValueError("O6 targets must be integer register values in 0..255")
    return struct.pack(
        O6_COMMAND_FORMAT,
        O6_COMMAND_MAGIC,
        O6_COMMAND_VERSION,
        struct.calcsize(O6_COMMAND_FORMAT),
        monotonic_ns,
        *integer_target,
    )

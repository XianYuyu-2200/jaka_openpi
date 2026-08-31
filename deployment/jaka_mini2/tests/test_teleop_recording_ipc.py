from __future__ import annotations

import pathlib
import socket
import struct

import pytest

from deployment.jaka_mini2.runtime.teleop_recording_ipc import SNAPSHOT_FORMAT
from deployment.jaka_mini2.runtime.teleop_recording_ipc import SNAPSHOT_MAGIC
from deployment.jaka_mini2.runtime.teleop_recording_ipc import SNAPSHOT_SIZE
from deployment.jaka_mini2.runtime.teleop_recording_ipc import SNAPSHOT_VERSION
from deployment.jaka_mini2.runtime.teleop_recording_ipc import SnapshotReceiver
from deployment.jaka_mini2.runtime.teleop_recording_ipc import decode_snapshot
from deployment.jaka_mini2.runtime.teleop_recording_ipc import encode_o6_command


def make_packet(sequence: int = 7) -> bytes:
    return struct.pack(
        SNAPSHOT_FORMAT,
        SNAPSHOT_MAGIC,
        SNAPSHOT_VERSION,
        SNAPSHOT_SIZE,
        sequence,
        123456,
        *range(6),
        *range(10, 16),
        *range(20, 26),
        *range(30, 36),
        0,
        0,
        1,
    )


def test_decode_snapshot_contract() -> None:
    snapshot = decode_snapshot(make_packet())
    assert snapshot.sequence == 7
    assert snapshot.actual_arm == pytest.approx(tuple(range(6)))
    assert snapshot.action_arm == pytest.approx(tuple(range(10, 16)))
    assert snapshot.actual_hand == pytest.approx(tuple(range(20, 26)))
    assert snapshot.action_hand == pytest.approx(tuple(range(30, 36)))
    assert snapshot.arm_state_valid is True
    assert snapshot.hand_state_valid is True
    assert snapshot.operator_dragging is True


def test_receiver_keeps_latest_datagram(tmp_path: pathlib.Path) -> None:
    path = tmp_path / "record.sock"
    with SnapshotReceiver(path) as receiver:
        sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sender.sendto(make_packet(1), str(path))
        sender.sendto(make_packet(2), str(path))
        assert receiver.latest().sequence == 2  # type: ignore[union-attr]
        sender.close()
    assert not path.exists()


def test_o6_command_validation() -> None:
    assert len(encode_o6_command((1, 2, 3, 4, 5, 6), 99)) == 28
    with pytest.raises(ValueError, match="integer register"):
        encode_o6_command((1, 2, 3, 4, 5, 6.5), 99)

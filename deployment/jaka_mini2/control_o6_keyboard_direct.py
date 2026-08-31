#!/usr/bin/env python3
"""Direct keyboard-to-O6 preset control through JAKA TIO FC16.

Each key press sends one six-register FC16 frame immediately. There is no
interpolation and no automatic return to the previous pose.
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import signal
import socket
import struct
import sys
import termios
import time
import tty

from deployment.jaka_mini2.runtime.arm_backend import JakaArmBackend
from deployment.jaka_mini2.runtime.hand_presets import load_hand_presets

LOGGER = logging.getLogger(__name__)
O6_COMMAND_MAGIC = 0x364F434A
O6_COMMAND_VERSION = 1
O6_COMMAND_FORMAT = "<IHHQ6H"


def _request_stop(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--arm-library",
        type=pathlib.Path,
        default=pathlib.Path("/tmp/jaka_mini2_adapter-hardware/libjaka_arm_c_api.so"),
    )
    parser.add_argument("--event-log", type=pathlib.Path)
    parser.add_argument(
        "--command-socket",
        type=pathlib.Path,
        help="Send O6 presets to the active arm follower instead of opening a second JAKA session.",
    )
    parser.add_argument("--arm-ip", default="192.168.0.102")
    parser.add_argument(
        "--preset-config",
        type=pathlib.Path,
        default=pathlib.Path("deployment/jaka_mini2/config/o6_hand_presets.yaml"),
    )
    args = parser.parse_args()

    # Bash starts asynchronous jobs with SIGINT ignored. Reset both signals so
    # the combined launcher can stop this process and still run the terminal
    # restoration in the finally block.
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    presets = load_hand_presets(args.preset_config)
    arm = None
    command_socket = None
    if args.command_socket is None:
        arm = JakaArmBackend(args.arm_library, args.arm_ip, allow_motion=False)
        arm.connect()
    else:
        command_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    event_log = None
    if args.event_log is not None:
        args.event_log.parent.mkdir(parents=True, exist_ok=True)
        event_log = args.event_log.open("a", encoding="utf-8")
    LOGGER.info("DIRECT O6 control: 1-5 send preset immediately; q quits")
    try:
        while True:
            key = sys.stdin.read(1)
            if key.lower() == "q":
                break
            preset = presets.get(key)
            if preset is None or preset.target is None:
                LOGGER.warning("ignored key %r: no calibrated preset", key)
                continue
            before = arm.read_o6_state() if arm is not None else None
            if arm is not None:
                arm.send_o6_position_direct(preset.target)
                time.sleep(0.3)
                after = arm.read_o6_state()
            else:
                integer_target = tuple(int(value) for value in preset.target)
                if any(float(integer) != value for integer, value in zip(integer_target, preset.target, strict=True)):
                    raise ValueError("O6 socket targets must be integer register values")
                packet = struct.pack(
                    O6_COMMAND_FORMAT,
                    O6_COMMAND_MAGIC,
                    O6_COMMAND_VERSION,
                    struct.calcsize(O6_COMMAND_FORMAT),
                    time.monotonic_ns(),
                    *integer_target,
                )
                command_socket.sendto(packet, str(args.command_socket))
                after = None
            LOGGER.info("sent %s (%s): %s; feedback %s -> %s", key, preset.name, preset.target, before, after)
            if event_log is not None:
                event_log.write(
                    json.dumps(
                        {
                            "timestamp_ns": time.time_ns(),
                            "key": key,
                            "preset": preset.name,
                            "target": list(preset.target),
                            "before": list(before) if before is not None else None,
                            "feedback": list(after) if after is not None else None,
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                event_log.flush()
    except KeyboardInterrupt:
        LOGGER.info("O6 keyboard control stopped")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if event_log is not None:
            event_log.close()
        if command_socket is not None:
            command_socket.close()
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main()

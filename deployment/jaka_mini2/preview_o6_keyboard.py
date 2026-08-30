#!/usr/bin/env python3
"""Read-only keyboard label preview for O6 demonstration collection.

This first version reads the real O6 state through JAKA TIO and writes a JSONL
event log.  It deliberately has no O6 write API.  Use it to calibrate and
review the event workflow before enabling any FC16 commissioning procedure.
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import select
import sys
import termios
import time
import tty

from deployment.jaka_mini2.runtime.arm_backend import JakaArmBackend
from deployment.jaka_mini2.runtime.hand_presets import HandPresetState
from deployment.jaka_mini2.runtime.hand_presets import load_hand_presets

LOGGER = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm-library", type=pathlib.Path, default=pathlib.Path("/tmp/jaka_mini2_adapter-hardware/libjaka_arm_c_api.so"))
    parser.add_argument("--arm-ip", default="192.168.0.102")
    parser.add_argument("--preset-config", type=pathlib.Path, default=pathlib.Path("deployment/jaka_mini2/config/o6_hand_presets.yaml"))
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("local_runtime/logs/o6_keyboard_events.jsonl"))
    args = parser.parse_args()

    presets = load_hand_presets(args.preset_config)
    state_machine = HandPresetState(presets)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    arm = JakaArmBackend(args.arm_library, args.arm_ip, allow_motion=False)
    arm.connect()
    initial_hand_state = tuple(float(value) for value in arm.read_o6_state())
    state_machine.select("space", current_state=initial_hand_state)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    LOGGER.info("Read-only O6 keyboard preview. Keys: 1-5 presets, Space hold current, q quit")
    try:
        with args.output.open("a", encoding="utf-8") as log:
            while True:
                sample_time = time.time_ns()
                arm_state = arm.read()
                hand_state = tuple(float(value) for value in arm.read_o6_state())
                event_key: str | None = None
                if select.select([sys.stdin], [], [], 0.2)[0]:
                    char = sys.stdin.read(1)
                    if char.lower() == "q":
                        break
                    event_key = "space" if char == " " else char
                    try:
                        selected = state_machine.select(event_key, current_state=hand_state)
                        LOGGER.info("selected %s (%s): %s", selected.key, selected.name, selected.target)
                    except (KeyError, ValueError) as error:
                        LOGGER.warning("ignored key %r: %s", char, error)
                        event_key = None
                active = state_machine.active
                record = {
                    "timestamp_ns": sample_time,
                    "arm_joint_position_rad": list(arm_state.joint_position),
                    "o6_state": list(hand_state),
                    "active_preset_key": state_machine.active_key,
                    "active_o6_target": list(active.target) if active and active.target else None,
                    "event_key": event_key,
                    "event_index": state_machine.event_index,
                    "writes_enabled": False,
                }
                log.write(json.dumps(record, ensure_ascii=False) + "\n")
                log.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        arm.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main()

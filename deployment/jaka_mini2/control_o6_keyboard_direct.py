#!/usr/bin/env python3
"""Direct keyboard-to-O6 preset control through JAKA TIO FC16.

Each key press sends one six-register FC16 frame immediately. There is no
interpolation and no automatic return to the previous pose.
"""

from __future__ import annotations

import argparse
import logging
import pathlib
import sys
import termios
import time
import tty

from deployment.jaka_mini2.runtime.arm_backend import JakaArmBackend
from deployment.jaka_mini2.runtime.hand_presets import load_hand_presets

LOGGER = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--arm-library",
        type=pathlib.Path,
        default=pathlib.Path("/tmp/jaka_mini2_adapter-hardware/libjaka_arm_c_api.so"),
    )
    parser.add_argument("--arm-ip", default="192.168.0.102")
    parser.add_argument(
        "--preset-config",
        type=pathlib.Path,
        default=pathlib.Path("deployment/jaka_mini2/config/o6_hand_presets.yaml"),
    )
    args = parser.parse_args()

    presets = load_hand_presets(args.preset_config)
    arm = JakaArmBackend(args.arm_library, args.arm_ip, allow_motion=False)
    arm.connect()
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
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
            before = arm.read_o6_state()
            arm.send_o6_position_direct(preset.target)
            time.sleep(0.3)
            after = arm.read_o6_state()
            LOGGER.info("sent %s (%s): %s; feedback %s -> %s", key, preset.name, preset.target, before, after)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        arm.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main()

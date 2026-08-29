#!/usr/bin/env python3
# ruff: noqa: RUF001

from __future__ import annotations

import argparse
import pathlib
from typing import Any

import yaml

DEFAULT_CONFIG = pathlib.Path(__file__).with_name("robot_config.yaml")

# This order is also the order in which missing parameters should be collected.
REQUIRED_FIELDS: tuple[tuple[str, str], ...] = (
    ("robot.model", "机械臂制造商和精确型号"),
    ("robot.control_interface", "机器人控制接口、SDK 或 ROS 包"),
    ("runtime.control_frequency_hz", "采集与执行动作的控制频率（Hz）"),
    ("observation.state_fields", "状态向量各维的顺序和含义"),
    ("observation.cameras", "用于策略的相机列表、视角和设备标识"),
    ("action.type", "动作类型，例如 joint_position、joint_velocity 或 ee_delta"),
    ("action.dimension", "单步动作维度（包含夹爪）"),
    ("action.components", "动作各维的顺序和含义"),
    ("action.units", "动作各维单位"),
    ("action.representation", "动作是 absolute 还是 delta/relative"),
    ("action.horizon", "模型一次预测的动作步数"),
    ("gripper.present", "是否配置夹爪"),
    ("dataset.id", "本地数据集名称与版本标识"),
    ("dataset.episode_success_policy", "episode 成功/失败样本的收录规则"),
    ("safety.emergency_stop_available", "是否有可用的物理急停"),
    ("safety.command_timeout_ms", "控制命令超时时间（毫秒）"),
    ("safety.joint_position_limits", "关节位置安全限制"),
    ("safety.joint_velocity_limits", "关节速度安全限制"),
    ("safety.workspace_limits", "末端工作空间安全限制"),
)

GRIPPER_FIELDS: tuple[tuple[str, str], ...] = (
    ("gripper.component_name", "夹爪在 state/action 中的字段名称"),
    ("gripper.open_value", "夹爪完全打开时的数值"),
    ("gripper.closed_value", "夹爪完全闭合时的数值"),
)


def lookup(config: dict[str, Any], dotted_path: str) -> Any:
    value: Any = config
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def is_missing(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, str | list | dict):
        return len(value) == 0
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate the local robot deployment contract.")
    parser.add_argument("--config", type=pathlib.Path, default=DEFAULT_CONFIG)
    parser.add_argument("--all", action="store_true", help="Print every missing field instead of only the next one.")
    args = parser.parse_args()

    with args.config.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        print(f"ERROR: {args.config} must contain a YAML mapping")
        return 1

    fields = list(REQUIRED_FIELDS)
    if lookup(config, "gripper.present") is True:
        insert_at = next(i for i, item in enumerate(fields) if item[0] == "dataset.id")
        gripper_fields = list(GRIPPER_FIELDS)
        if lookup(config, "gripper.policy_interface.representation") == "native_6d":
            gripper_fields = [item for item in gripper_fields if item[0] == "gripper.component_name"]
        fields[insert_at:insert_at] = gripper_fields

    missing = [(path, description) for path, description in fields if is_missing(lookup(config, path))]
    if not missing:
        print("robot configuration status: COMPLETE")
        return 0

    shown = missing if args.all else missing[:1]
    for path, description in shown:
        print(f"MISSING {path}: {description}")
    print(f"remaining required fields: {len(missing)}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

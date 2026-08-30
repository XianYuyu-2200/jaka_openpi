# O6 hand adapter

The production driver remains in:

```text
~/codex/codex-jaka_mini_2/linkerhand-ros2-sdk
```

`linkerhand_rs485_readonly.patch` captures the current local configuration and
safety changes. It is stored here for review and reproducibility, not applied
automatically.

Current hardware result:

- protocol: Modbus RTU through JAKA Mini2 TIO RS485-1;
- tool pins: 4 = RS485A+, 5 = RS485B-;
- slave ID: right O6 `0x27` / decimal `39`;
- state source: JAKA TIO signal cache (`o6_r_pos0` through `o6_r_pos5`);
- candidate ROS2 topic if a bridge is added later: `/cb_right_hand_state`;
- expected message: `sensor_msgs/msg/JointState`, six positions in `0..255`;
- `initialize_pose=false`, `read_device_info=false`, `is_touch=false`;
- do not publish `/cb_right_hand_control_cmd` during the read-only check.

The generic `linker_hand.py` path supports Modbus. The separate
`linker_hand_advanced_o6.py` path rejects Modbus and also sends an initial pose,
so it must not be used for the first RS485 validation.

The first read-only validation passed through JAKA TIO: FC04 input registers
0-5 returned `[254, 255, 254, 254, 254, 254]` repeatedly. The OpenPI runtime
uses the JAKA TIO signal cache for state reads; the O6 write path remains
disabled.

# O6 hand adapter

The production driver remains in:

```text
~/codex/codex-jaka_mini_2/linkerhand-ros2-sdk
```

`linkerhand_rs485_readonly.patch` captures the current local configuration and
safety changes. It is stored here for review and reproducibility, not applied
automatically.

Expected first hardware check:

- protocol: Modbus RTU over USB-RS485;
- candidate device: `/dev/ttyUSB0` (replace with the enumerated path);
- slave ID: left O6 `0x28` / decimal `40`;
- state topic: `/cb_left_hand_state`;
- expected message: `sensor_msgs/msg/JointState`, six positions in `0..255`;
- `initialize_pose=false`, `read_device_info=false`, `is_touch=false`;
- do not publish `/cb_left_hand_control_cmd` during the read-only check.

The generic `linker_hand.py` path supports Modbus. The separate
`linker_hand_advanced_o6.py` path rejects Modbus and also sends an initial pose,
so it must not be used for the first RS485 validation.

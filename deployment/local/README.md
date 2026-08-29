# openpi 本地部署

该目录负责本机的 JAX 推理环境、运行目录和机器人接口参数。已验证的 JAKA
底层代码及 OpenPI 侧的 12 维接口、安全检查和插值模块已集成到
`deployment/jaka_mini2/`。两台 Orbbec 相机已物理连接并通过 USB 枚举确认；彩色帧并发采集、
LeRobot 数据写入、训练配置和最终真机后端仍需继续接入。

离线验证方式及迁移来源见 `deployment/jaka_mini2/README.md`。

## 已部署的运行结构

执行以下命令会创建被 Git 忽略的本地目录：

```bash
source deployment/local/runtime_env.sh
```

该命令也会把当前用户安装的 `uv` 加入本终端的 `PATH`。

目录结构：

```text
local_runtime/
├── cache/openpi/       # 官方及自训练模型缓存
├── datasets/           # 现场采集的 LeRobot 数据集
├── logs/               # 推理延迟和运行日志
├── models/current/     # 当前部署 checkpoint
├── models/previous/    # 上一个可回滚 checkpoint
└── policy_records/     # 可选的策略输入输出记录
```

## 环境检查

```bash
uv run deployment/local/check_environment.py
```

## 官方 checkpoint 冒烟测试

第一次运行会从 `gs://openpi-assets` 下载 π0.5-DROID checkpoint，并缓存到 `local_runtime/cache/openpi`。

终端一：

```bash
deployment/local/start_official_smoke_server.sh
```

终端二：

```bash
deployment/local/run_official_smoke_client.sh
```

这只是验证本地模型服务，不会向机器人发送动作。

## 逐项填写机器人参数

查看下一个待确认项：

```bash
uv run deployment/local/validate_robot_config.py
```

查看所有待确认项：

```bash
uv run deployment/local/validate_robot_config.py --all
```

当前相机配置和序列号绑定如下（实际 `/dev/videoN` 节点仍需运行时按序列号发现）：

```yaml
observation:
  cameras:
    - name: external
      serial_number: "CP28563000AJ"
      role: base_0_rgb
      color_order: RGB
      resolution: [1280, 800]
    - name: wrist
      serial_number: "CV2L761000KT"
      role: left_wrist_0_rgb
      color_order: RGB
      resolution: [1280, 800]
```

两台相机已经通过 sysfs 完成 USB 枚举确认；彩色帧格式和同步采集仍需现场验证。

## 关节位置安全范围

`robot_config.yaml` 中的 `safety.joint_position_limits` 是供 OpenPI/VLA
运行时使用的软件安全范围，不是 JAKA 控制器实际软限位。当前数值来自 JAKA
ROS 包中的 `jaka_minicobo.urdf` 标称范围，并在上下界各预留 `0.10 rad`
（约 `5.7°`）：

```text
      URDF 标称范围 (rad)    运行安全范围 (rad)
J1    [-6.28,  6.28]         [-6.18,  6.18]
J2    [-2.09,  2.09]         [-1.99,  1.99]
J3    [-2.27,  2.27]         [-2.17,  2.17]
J4    [-6.28,  6.28]         [-6.18,  6.18]
J5    [-2.09,  2.09]         [-1.99,  1.99]
J6    [-6.28,  6.28]         [-6.18,  6.18]
```

JAKA SDK 目前只能通过 `is_on_limit()` 判断是否已触发软限位，没有读取
J1–J6 控制器上下限的公开接口。`jaka_minicobo.urdf` 与现场 JAKA Mini 2
的对应关系也尚未通过控制器参数页面确认，因此配置保留
`controller_limits_confirmed: false`。在完成现场核对前，越界处理必须保持为
`hold_and_stop`，不得把上述范围当作经控制器确认的机械臂极限。

## 关节速度与 TCP 工作空间

`safety.joint_velocity_limits` 使用保守的应用层初始值：J1–J3 为
`0.50 rad/s`，J4–J6 为 `0.35 rad/s`。随 ROS 包的 URDF 对 J1–J6 均给出
`1.57 rad/s` 标称速度，但该数值不是现场控制器软限位，因此保留
`joint_velocity_limits_confirmed: false`。

`safety.workspace_limits` 来自 `P1–P11` 记录的 TCP 位置包围盒，并在 X、Y、Z
三个方向各预留 `10 mm`：

```text
X: [43.26, 353.53] mm
Y: [-558.89, -279.32] mm
Z: [43.48, 490.15] mm
```

这些坐标使用当前控制器用户坐标系和当前工具 TCP；任一坐标系、工具 TCP、咖啡机
位置或桌面布置改变后必须重新记录。该轴对齐包围盒不是咖啡机碰撞模型，越界处理
保持为 `hold_and_stop`，并保留 `workspace_limits_confirmed: false`。

## Linker Hand O6 通信方式

当前 O6 配置采用 USB-RS485 转换器连接，协议为 Modbus RTU，不使用 `can0`：

```yaml
hand_joint: O6
modbus: /dev/ttyUSB0
baudrate: 115200
serial: 8N1, no parity
slave_id: 0x28  # 左手，十进制 40
```

协议依据仓库根目录的
`O6机械手485协议简要说明_1783907046316.xlsx`：功能码 `04` 用于读取输入寄存器，
功能码 `16` 用于写保持寄存器；位置寄存器为 `0–5`，数值范围为 `0–255`。
实际接入后应把 `/dev/ttyUSB0` 替换成系统枚举出的真实设备路径。

目前启动配置将 `initialize_pose` 和 `read_device_info` 设为 `false`：首次接入
只验证状态话题，不自动发送初始手型，也不读取 Excel 未明确覆盖的扩展设备信息区。
压感扩展暂不作为 OpenPI 状态输入，待现场确认固件寄存器后再启用。

## 部署自训练 checkpoint

自定义训练配置和 checkpoint 完成后，使用：

```bash
deployment/local/start_custom_policy.sh \
  pi05_myrobot_full \
  local_runtime/models/current/checkpoint
```

真机客户端默认只连接 `127.0.0.1:8000`。将端口暴露到局域网或公网前，需要额外配置认证、访问控制和安全停机机制。

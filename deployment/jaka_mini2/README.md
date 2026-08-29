# JAKA Mini 2 + Linker Hand O6 integration

This directory is the integration boundary between OpenPI and the existing
robot work in `~/codex/codex-jaka_mini_2`. It preserves the proven low-level
JAKA code while keeping model, dataset and policy-facing code in this OpenPI
repository.

## What has been migrated

- JAKA 125 Hz joint-state publisher and Unix datagram packet contract.
- Six-joint relative follower used during hardware commissioning.
- Readiness, watchdog, timing and latest-packet primitives.
- Read-only waypoint recorder and the captured P1-P11 coffee waypoints.
- Offline C++ tests for the migrated primitives.
- The local LinkerHand O6 RS485/read-only patch, without copying the full
  third-party ROS2 repository.

The original repository remains unchanged. Logs, executables, `.git`, ROS2
build/install trees and the proprietary JAKA SDK were not copied.

## OpenPI integration layer

The Python modules add the policy-facing parts that did not exist in the
teleoperation program:

- a fixed 12-D state/action contract: JAKA J1-J6 followed by O6 native J1-J6;
- policy response validation;
- 5 Hz to 125 Hz absolute arm-position interpolation;
- one O6 command per policy step (5 Hz, below the driver's 30 Hz limit);
- fail-closed joint, velocity, hand-range, workspace and timeout checks loaded
  from `deployment/local/robot_config.yaml`.
- a JAKA SDK backend for joint reads, forward kinematics, `servo_j` and abort;
- serial-bound dual Orbbec RGB capture with a timestamp-skew check;
- an OpenPI `Environment` with `read_only`, `mock` and explicitly armed
  `hardware` modes;
- a LeRobot episode writer that admits only successful episodes whose O6 state
  was verified on every frame;
- JAKA/O6 input/output transforms and the `pi05_jaka_mini2_o6` training config.

Because the policy action is expressed in joint space, command planning also
requires the target TCP position computed by forward kinematics. If target FK
is unavailable, workspace validation rejects the action instead of bypassing
the check.

The default runtime mode is non-writing. Hardware motion requires both a
backend constructed with motion permission and the exact runtime confirmation
`ENABLE_POLICY_CONTROL`. It is additionally blocked until the O6 state source
has passed the real RS485 read-only validation.

## Offline verification

```bash
cmake -S deployment/jaka_mini2/arm_adapter \
  -B /tmp/jaka_mini2_adapter-build
cmake --build /tmp/jaka_mini2_adapter-build
ctest --test-dir /tmp/jaka_mini2_adapter-build --output-on-failure

.venv/bin/python -m pytest -q \
  deployment/jaka_mini2/tests \
  src/openpi/policies/jaka_mini2_policy_test.py
```

To build the preserved hardware tools, explicitly provide the external JAKA
SDK location:

```bash
cmake -S deployment/jaka_mini2/arm_adapter \
  -B /tmp/jaka_mini2_adapter-hardware \
  -DBUILD_JAKA_HARDWARE_TOOLS=ON \
  -DJAKA_SDK_ROOT="$HOME/codex/codex-jaka_mini_2/jaka_ros2/src/jaka_driver"
cmake --build /tmp/jaka_mini2_adapter-hardware
ctest --test-dir /tmp/jaka_mini2_adapter-hardware --output-on-failure
```

Building does not execute any robot command. Do not run
`six_joint_follow_test` during OpenPI integration; it is retained only as the
verified commissioning reference and still consumes operator-arm IPC input.

## Camera verification

The Gemini 335L and Gemini 305 are bound by USB serial number, never by the
unstable `/dev/videoN` index. Run the read-only enumeration check with:

```bash
.venv/bin/python deployment/jaka_mini2/camera_adapter/inspect_cameras.py

ORBBEC_SDK_ROOT="$HOME/下载/OrbbecViewer_v1.10.35_202601290127_linux_x64_release/orbbec_v2.9.3/OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_x86_64"
cmake -S deployment/jaka_mini2/camera_adapter \
  -B /tmp/jaka-orbbec-camera \
  -DORBBEC_SDK_ROOT="$ORBBEC_SDK_ROOT"
cmake --build /tmp/jaka-orbbec-camera
.venv/bin/python -m deployment.jaka_mini2.camera_adapter.verify_capture \
  --frames 30 \
  --max-skew-ms 20
```

The runtime uses the official Orbbec SDK backend. It requests only the true
color stream, converts MJPG/YUYV to RGB, discards stale startup frames and
pairs frames using the SDK system timestamp. A hardware run passed 30 pairs at
1280x800@30; the worst observed skew across repeated runs was 19.684 ms. The V4L2 path is retained only as a
strict diagnostic fallback.

## Policy runtime

Start the policy server in another terminal, then exercise the client without
hardware using:

```bash
.venv/bin/python -m deployment.jaka_mini2.run_policy_client \
  --mode mock \
  --max-episode-steps 10
```

`read_only` opens the JAKA state/FK backend and both cameras but validates each
policy action without sending it. `hardware` remains intentionally unusable
while `run_policy_client.py` supplies `UnavailableHand(state_verified=False)`.
Do not replace that gate until the RS485 state verification is complete.

## Dataset and training

`runtime/recording.py` defines the LeRobot schema and the episode admission
rule. Frames contain:

- `observation.images.base_0_rgb` and
  `observation.images.left_wrist_0_rgb`, both RGB uint8;
- `observation.state`, 12-D JAKA J1-J6 followed by native O6 J1-J6;
- `action`, with the same 12-D order.

Only `success` episodes with `hand_state_valid=True` on every frame call
`save_episode()`. Failed, interrupted, safety-intervention and unverified-hand
episodes are archived under `excluded_episodes/` and do not enter training.
Capture timestamps must be finite and increasing; the stored LeRobot timeline
uses the exact 5 Hz frame grid required by its timestamp validator.
The local dataset root is set by `deployment/local/runtime_env.sh` through
`HF_LEROBOT_HOME`.

The fine-tuning config is `pi05_jaka_mini2_o6`:

```bash
. deployment/local/runtime_env.sh
.venv/bin/python scripts/compute_norm_stats.py \
  --config-name pi05_jaka_mini2_o6

XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
  .venv/bin/python scripts/train.py pi05_jaka_mini2_o6 \
  --exp-name=coffee-v001 \
  --overwrite
```

The model keeps the pretrained π0.5 width of 32 and pads the 12-D state/action
internally. Only the first 12 output dimensions are exposed to the robot. Arm
J1-J6 are converted to delta targets for training; O6 J1-J6 stay absolute.

## Hardware gates still closed

- O6 `/cb_left_hand_state` must be verified read-only over the actual RS485
  device path. Do not publish the hand control topic during this check.
- Camera serial binding, RGB order, true color delivery at 1280x800@30 and
  cross-device pairing below 20 ms are confirmed with the official SDK.
- Controller joint limits, application velocity limits and the P1-P11 TCP
  workspace remain marked unconfirmed in `robot_config.yaml`.
- The O6 ROS2 state/command backend is deliberately not connected to the
  policy runtime until the first RS485 read-only validation is complete.

## Directory map

```text
arm_adapter/   preserved C++ robot code and offline tests
config/        migrated coffee waypoints
camera_adapter/serial-number binding and read-only enumeration check
hand_adapter/  reproducible LinkerHand RS485/read-only patch
runtime/       Environment, JAKA backend, recording, mocks and 12-D contract
safety/        fail-closed action checks
tests/         offline Python integration tests
```

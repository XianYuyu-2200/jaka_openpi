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

The default runtime mode is non-writing. The right O6 state source has passed
real FC04 validation through JAKA TIO RS485-1. Hardware motion remains blocked
because the O6 write path has not yet been commissioned.

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

Building does not execute any robot command. The commissioning follower now
uses a fixed 125 Hz servo clock, latest-sample reception, low-pass and
acceleration limiting, the native one-cycle (`8 ms`) JAKA interpolation horizon, a
40 ms packet hold threshold and a 100 ms data-timeout abort. A lost operator
drag signal holds immediately and exits only if it persists for 300 ms. The
real-time command loop does not perform blocking state reads.
It remains separate from OpenPI policy execution.

For a short supervised `.101` -> `.102` teleoperation check, put `.101` in
drag mode and run both processes from one terminal:

```bash
deployment/jaka_mini2/run_six_joint_teleop.sh 30
```

The optional second argument selects the JAKA controller filter:

```bash
deployment/jaka_mini2/run_six_joint_teleop.sh 20 baseline  # current host-side smoothing only
deployment/jaka_mini2/run_six_joint_teleop.sh 20 nlf       # controller speed/accel/jerk limiting
deployment/jaka_mini2/run_six_joint_teleop.sh 20 lpf 1.0   # lower-latency LPF
deployment/jaka_mini2/run_six_joint_teleop.sh 20 lpf 0.5   # JAKA example LPF
```

Each run explicitly configures the filter, so a previous run cannot leave a
controller-side filter active. The NLF trial uses 45 deg/s, 600 deg/s² and
6000 deg/s³; these are responsive starting values and should be evaluated
with small motions first.

LPF cutoff is the optional third argument. A higher cutoff responds faster but
filters less; use `1.0`, then `1.5`, and only then `2.0` when tuning for lower
latency. The default LPF cutoff is `1.0`.

The script starts the `.102` follower first, then the `.101` publisher. The
follower, publisher and `.101` SDK sampler use separate performance CPU cores
(8, 10 and 6 by default). The sampler exposes a joint sample immediately after
each SDK read; the publisher wakes on new samples and uses an 8 ms heartbeat
only while the SDK is blocked. Readiness queries therefore no longer make a
fresh joint sample old. Repeated samples retain their original acquisition
timestamp; the follower holds at 40 ms and stops at 100 ms of source age. The
script stops all processes together on `Ctrl+C` or when either process exits.
To run the programs separately for diagnosis, use two terminals:

The interpolation horizon can be tested without rebuilding by setting
`JAKA_SERVO_STEP_NUM` from `1` through `4`; the default is `1`:

```bash
JAKA_SERVO_STEP_NUM=2 deployment/jaka_mini2/run_six_joint_teleop.sh 20
```

```bash
# Terminal 1: tracking arm (.102), stop automatically after 30 seconds
/tmp/jaka_mini2_adapter-hardware/six_joint_follow_test \
  192.168.0.102 /tmp/jaka_six_joint.sock START_SIX_JOINTS 30

# Terminal 2: operator arm (.101)
/tmp/jaka_mini2_adapter-hardware/operator_joint_publisher \
  192.168.0.101 /tmp/jaka_six_joint.sock 30
```

Press `Ctrl+C` in either terminal to stop early. Loss of operator packets makes
the follower hold after 40 ms and abort after 100 ms. The final follower line
reports `missed_servo_periods`; a nonzero or increasing value indicates host or
SDK timing stalls that can still be felt as motion jitter.

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

Before a robot-specific trained checkpoint exists, the complete real-input
WebSocket and safety path can be verified with the observation echo policy.
It validates the real 12-D state and both RGB frames, then returns the current
state as an absolute action. In `read_only` the action is checked and discarded;
no arm command and no O6 FC16 write is sent.

Terminal 1:

```bash
.venv/bin/python -m deployment.jaka_mini2.serve_read_only_echo_policy
```

Terminal 2:

```bash
.venv/bin/python -m deployment.jaka_mini2.run_policy_client \
  --mode read_only \
  --max-episode-steps 3
```

For a software-only check, use:

```bash
.venv/bin/python -m deployment.jaka_mini2.run_policy_client \
  --mode mock \
  --max-episode-steps 10
```

`read_only` opens the JAKA state/FK backend, reads the right O6 through the JAKA
TIO signal cache, and captures both cameras without sending actions.
`hardware` remains intentionally unusable until the O6 FC16 write path has a
separate low-risk commissioning procedure and explicit operator approval.

## Dataset and training

### Keyboard O6 demonstration labels

The first collection-stage tool is intentionally read-only. It reads the real
`.102` arm and O6 state at about 5 Hz and logs keyboard-selected O6 targets to
`local_runtime/logs/o6_keyboard_events.jsonl`. It contains no FC16 call and
does not create a training episode.

```bash
.venv/bin/python -m deployment.jaka_mini2.preview_o6_keyboard
```

Keys `1` through `5` select the configured hand poses, Space latches the
current real O6 state as a hold target, and `q` exits. The poses are six native
O6 device positions in `0..255`, not JAKA arm joint angles. Configure them in
`config/o6_hand_presets.yaml`. Uncalibrated (`null`) poses are rejected. Real
O6 commands remain disabled until a separate supervised FC16 small-motion
commissioning procedure is approved.

After approval, the isolated FC16 probe can be run as follows. It changes only
one channel by five counts, reads the result, and restores the original value;
it does not enable JAKA arm servo mode:

```bash
/tmp/jaka_mini2_adapter-fc16/o6_fc16_single_step \
  192.168.0.102 0 -5 ENABLE_O6_FC16_SINGLE_STEP
```

The live probe completed with channel 0 `254 -> 249 -> 254`.

For direct manual preset testing, rebuild the hardware backend and run:

```bash
.venv/bin/python -m deployment.jaka_mini2.control_o6_keyboard_direct
```

Keys `1` through `5` each send one FC16 frame containing the configured six
position values. There is no interpolation and no automatic restore. Press
`q` to exit. This path remains separate from OpenPI policy execution.

### Combined teleoperation and O6 practice

Use the combined practice launcher to continuously teleoperate `.102` from
`.101` while keys `1` through `5` command the real O6 hand:

```bash
deployment/jaka_mini2/run_teleop_o6_practice.sh lpf 1.0
```

The arm processes run without a duration limit. Press `q` in the foreground
keyboard controller (or `Ctrl+C`) to stop the O6 controller and both arm
processes together. Each hand key event is appended to
`local_runtime/logs/o6_practice_events.jsonl` for workflow review; this is a
practice log, not a formal LeRobot episode.

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

## Hardware status and remaining gates

- Right O6 FC04 registers 0-5 are verified through JAKA TIO RS485-1, tool pins
  4/5, slave 39. The runtime reads `o6_r_pos0` through `o6_r_pos5` and reports
  `hand_state_valid=True`.
- Camera serial binding, RGB order, true color delivery at 1280x800@30 and
  cross-device pairing below 20 ms are confirmed with the official SDK.
- Controller joint limits, application velocity limits and the P1-P11 TCP
  workspace remain marked unconfirmed in `robot_config.yaml`.
- O6 writes remain disabled. Do not send FC16 or enable policy hardware mode
  until a supervised small-motion commissioning test is explicitly approved.

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

# Source and dependency notes

## JAKA

The migrated C++ integration sources came from the local workspace
`~/codex/codex-jaka_mini_2/jaka_dual_readonly`. The proprietary vendor SDK is
not included. `JAKAZuRobot.h` and `libjakaAPI.so` remain external build/runtime
dependencies under `~/codex/codex-jaka_mini_2/jaka_ros2/src/jaka_driver`.

Exact hashes of every migrated file are recorded in
`arm_adapter/SOURCE_MANIFEST.md`.

## LinkerHand

The full `linkerhand-ros2-sdk` repository is not copied. The file
`hand_adapter/linkerhand_rs485_readonly.patch` records the local modifications
against upstream commit:

```text
e6a580e29ab0b13ebf6965a176b5cef2e9160e62
```

The upstream LinkerHand repository declares the Apache License 2.0. The patch
selects O6 Modbus/RS485, disables pressure input, disables startup pose writes,
and disables reads of unconfirmed extended device-information registers.

Apply the patch only to the matching source revision and review it again if the
driver is upgraded.

## Orbbec

The Orbbec SDK is not copied into this repository. The dual-camera backend is
built against the externally unpacked SDK 2.9.3 currently located under the
user's `~/下载/` directory. `ORBBEC_SDK_ROOT` must point to a compatible SDK
containing `include/libobsensor/ObSensor.hpp` and `lib/libOrbbecSDK.so`.

The SDK package includes its own `LICENSE.txt` and third-party license bundle.
The integration requests color streams only and does not change camera
firmware or persistent device settings.

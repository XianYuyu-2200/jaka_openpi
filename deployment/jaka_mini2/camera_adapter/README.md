# Dual Orbbec camera binding

Both cameras were detected through Linux sysfs on 2026-08-29:

| Policy role | Model | USB ID | Serial number | Current USB path |
| --- | --- | --- | --- | --- |
| `base_0_rgb` | Orbbec Gemini 335L | `2bc5:0804` | `CP28563000AJ` | `2-3` |
| `left_wrist_0_rgb` | Orbbec Gemini 305 | `2bc5:0840` | `CV2L761000KT` | `2-1` |

Both devices negotiated 5000 Mbps. Runtime binding must use the serial number,
not `/dev/videoN` or the current USB topology path, because those identifiers
can change after reconnecting or rebooting.

The official Orbbec SDK 2.9.3 has verified simultaneous true color delivery at
1280x800@30: the 335L supplies MJPG and the 305 supplies YUYV. The common
30 FPS mode is intentional; 60 FPS is not supported by the 305 at 1280x800.

Run the read-only inspector from a host terminal:

```bash
python deployment/jaka_mini2/camera_adapter/inspect_cameras.py
```

The inspector opens no camera stream and changes no device setting.

After enumeration passes, verify actual RGB delivery and timestamp skew:

```bash
ORBBEC_SDK_ROOT="$HOME/下载/OrbbecViewer_v1.10.35_202601290127_linux_x64_release/orbbec_v2.9.3/OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_x86_64"
cmake -S deployment/jaka_mini2/camera_adapter \
  -B /tmp/jaka-orbbec-camera \
  -DORBBEC_SDK_ROOT="$ORBBEC_SDK_ROOT"
cmake --build /tmp/jaka-orbbec-camera

.venv/bin/python -m deployment.jaka_mini2.camera_adapter.verify_capture \
  --frames 30 \
  --max-skew-ms 20
```

This uses the official SDK backend, opens both true-color streams by serial
number, converts them to RGB uint8 at 1280x800, pairs frames by the Orbbec
system timestamp, and fails if a pair exceeds the configured skew. Repeated
hardware runs passed 30 pairs with a worst observed skew of 19.684 ms. It sends no robot or
hand command. `v4l2_capture.py` remains a diagnostic fallback and rejects a
node unless the actual pixel format, resolution and FPS all match.

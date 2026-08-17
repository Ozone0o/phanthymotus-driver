# PNDbotics Adam driver

This directory contains the Adam driver-side cards. The driver is a client:

```text
Adam driver container --MCP/DDS client--> official adam_demo --gRPC :6666--> robot control
```

The driver must not create a replacement gRPC server. On Adam-SP/Adam-Pro the
vendor service is provided by `adam_demo/bin/run.sh` on the robot NUC. The
documented default path is:

```text
/home/pnd-humanoid/Documents/adam_demo/bin/run.sh
```

Use the included verifier before deploying the driver. It is read-only unless
`--start` is explicitly supplied:

```bash
ADAM_SSH_PASSWORD='...' ./pndbotics/adam/verify_adam_demo.sh
```

Cards:

- `robot_state`: official `RobotControl.GetRobotState` response fields.
- `switch_mode`: official `RobotControl.SetMode` modes.
- `loco`: official `RobotControl.SetSpeed` (`move` / `stop_move`).
- `hand_state`: DDS `rt/handstate`, all 12 finger positions.
- `hand`: DDS `rt/handcmd`, without the old `get_state` action.
- `arm`: periodic DDS `rt/lowcmd` upper-body position control.
- `camera_head`: ZED Mini left RGB as `image/jpeg`, captured directly by the
  Jetson-local ZED SDK.
- `camera_depth`: ZED Mini depth as `image/depth-zlib` (640x480 little-endian
  uint16 millimetres compressed with zlib).
- `camera_info`: ZED Mini connection state, stream settings, calibration and
  left/right camera intrinsics as `data/json`.
- `camera_pointcloud`: optional ZED Mini XYZ stream as `sensor/pointcloud`.
  It is disabled by default to avoid continuous GPU/bandwidth use and can be
  enabled in `config.yaml` or toggled with the card's `start`/`stop` action.

The head-mounted ZED Mini is installed upside down. `camera_flip: true` uses
the ZED SDK's native camera-data flip so RGB, depth and point-cloud outputs
remain aligned; set it to `false` only when the camera is physically upright.
The point-cloud path also applies the fixed `pointcloud.mount_rotation_deg`
calibration after that flip. Its X/Y/Z values are renderer-frame degrees and
should be adjusted if the camera bracket is remounted; it is deliberately a
static mount correction rather than an IMU-based per-frame leveling step. The
following `pointcloud.mount_translation_m` is then applied in the same frame;
its `y` component is the upward offset, so Adam's default `y: 1.8` places the
floor at the renderer's zero-height plane. Adjust it if the camera height or
the desired ground reference changes.

The ZED Mini is a local USB device on the Jetson Orin. The deployment mounts
the host's `/usr/local/zed` SDK and its aarch64 `pyzed` extension into the
privileged container; no ZED network streaming sender is used by this driver.

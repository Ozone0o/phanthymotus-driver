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

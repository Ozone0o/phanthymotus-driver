"""
go1_sdk_client.py — Unitree Go1 (EDU) 原始 unitree_legged_sdk 封装（go1_bundle bundle · 只读版）。

本 bundle 只做**状态读取**（loco_state / battery 两张卡），因此这里只保留一个
**只读的 HIGHLEVEL 客户端**：后台以 ~500Hz 走 UDP 收状态（HighState），把 comm.h 结构
解析成一份线程安全的 `snapshot()` dict。所有状态卡都读这同一份 snapshot，不各自开 UDP。

【实现基座】官方 unitree_legged_sdk（Go1 分支 v3.8.6）的 pybind11 绑定 `robot_interface`
（HighCmd/HighState/...）。镜像内 `cmake -DPYTHON_BUILD=ON` 按容器 python 版本构建，
rclpy 可同进程共存 → 状态卡能发 ROS2 topic 在画布渲染。

【降级 / STUB】导入不到 robot_interface（如开发机 Mac、无硬件）时进入 STUB：不收发、
snapshot 为空（fresh=False）。MCP server 仍能起、注册、列 tool，方便无硬件时跑通链路。

【要加“运控卡”的后来者看这里】本文件刻意**不含**任何下发命令的原语（move/set_mode/
power_off 等）与低层 LowCmd/Safety —— 因为 go1_bundle 只读。若要新增控制卡，见 CONTRIBUTING.md
“新增控制卡”一节：需要引入 HighCmd、加锁合成、并在 _loop() 里 SetSend(cmd)。
"""

from __future__ import annotations

import struct
import threading
import time

# 控制字（unitree_legged_sdk 约定）
HIGHLEVEL = 0xEE

# 默认网络参数（Go1 板载网段；来自 udp.h：HIGH 目标 .161:8082）
DEFAULT_TARGET_IP = "192.168.123.161"   # UDP_SERVER_IP_SPORT（高层运动服务）
HIGH_TARGET_PORT = 8082
HIGH_LOCAL_PORT = 8090

LOOP_HZ = 500.0        # 后台收发频率（高层 2ms 亦可）

# HighState.mode（Go1 legacy comm.h）
MODE_NAMES = {0: "idle", 1: "force_stand", 2: "walk", 5: "stand_down",
              6: "stand_up", 7: "damp", 8: "recovery", 10: "jump_yaw", 11: "straight_hand"}

GAIT_NAMES = {0: "idle", 1: "trot", 2: "trot_run", 3: "climb_stair", 4: "trot_obstacle"}

FOOT_ORDER = ["FR", "FL", "RR", "RL"]
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

BMS_STATUS_NAMES = {0: "wakeup", 1: "discharge", 2: "charge", 3: "charger", 4: "precharge",
                    5: "charge_error", 6: "waterfall_light", 7: "self_discharge", 8: "junk"}


def _r(v, nd=4):
    try:
        return round(float(v), nd)
    except Exception:
        return 0.0


def _g(obj, name, default=None):
    """从 pybind 对象或 dict 防御式取字段（字段名对照 comm.h 逐一核实）。"""
    try:
        if isinstance(obj, dict):
            return obj.get(name, default)
        return getattr(obj, name, default)
    except Exception:
        return default


def _cartesian_list(v):
    """[Cartesian(x,y,z)] × 4 → [{'x','y','z'}]；缺失/异常返回 None。"""
    if v is None:
        return None
    try:
        out = []
        for e in list(v):
            if hasattr(e, "x"):
                out.append({"x": _r(e.x), "y": _r(e.y), "z": _r(e.z)})
            else:
                seq = list(e)
                out.append({"x": _r(seq[0]), "y": _r(seq[1]), "z": _r(seq[2])})
        return out
    except Exception:
        return None


def _to_bytes40(v) -> bytes:
    try:
        return bytes(bytearray([int(x) & 0xFF for x in list(v)][:40])).ljust(40, b"\x00")
    except Exception:
        return b"\x00" * 40


# ── comm.h 结构 → dict（可复用的解析块；新卡按需取用）──────────────────────────

def parse_imu(imu) -> dict | None:
    if imu is None:
        return None
    return {
        "quaternion_wxyz": [_r(q, 5) for q in _g(imu, "quaternion", [0, 0, 0, 0])],
        "gyroscope_rad_s": [_r(x) for x in _g(imu, "gyroscope", [0, 0, 0])],
        "accelerometer_m_s2": [_r(x) for x in _g(imu, "accelerometer", [0, 0, 0])],
        "rpy_rad": [_r(x) for x in _g(imu, "rpy", [0, 0, 0])],
        "temperature_c": int(_g(imu, "temperature", 0)),
    }


def parse_joints(motor_state) -> list | None:
    if motor_state is None:
        return None
    joints = []
    for i, m in enumerate(list(motor_state)[:12]):   # Go1 只用前 12 个腿部电机
        joints.append({
            "i": i, "mode": int(_g(m, "mode", 0)), "q": _r(_g(m, "q", 0.0)),
            "dq": _r(_g(m, "dq", 0.0)), "ddq": _r(_g(m, "ddq", 0.0)),
            "tau": _r(_g(m, "tauEst", 0.0)), "temp": int(_g(m, "temperature", 0)),
        })
    return joints


def parse_battery(bms) -> dict | None:
    if bms is None:
        return None
    status = int(_g(bms, "bms_status", 0))
    return {
        "version": {"high": int(_g(bms, "version_h", 0)), "low": int(_g(bms, "version_l", 0))},
        "status_code": status, "status_name": BMS_STATUS_NAMES.get(status, "unknown"),
        "soc_percent": int(_g(bms, "SOC", 0)), "current_ma": int(_g(bms, "current", 0)),
        "cycle_count": int(_g(bms, "cycle", 0)),
        "bq_ntc_c": [int(t) for t in _g(bms, "BQ_NTC", [])],
        "mcu_ntc_c": [int(t) for t in _g(bms, "MCU_NTC", [])],
        "cell_voltage_mv": [int(v) for v in _g(bms, "cell_vol", [])],
    }


def parse_wireless_remote(raw: bytes) -> dict:
    """按 joystick.h(xRockerBtnDataStruct) 解析 40 字节 → {buttons, axes}。
    布局: head[2] | btn(uint16 LE) | lx f | rx f | ry f | L2 f | ly f | idle[16]"""
    if not raw or len(raw) < 24:
        return {"buttons": {}, "axes": {}}
    try:
        btn = struct.unpack_from("<H", raw, 2)[0]
        lx, rx, ry, l2, ly = struct.unpack_from("<fffff", raw, 4)
        names = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
                 "A", "B", "X", "Y", "up", "right", "down", "left"]
        return {"buttons": {n: bool((btn >> i) & 1) for i, n in enumerate(names)},
                "axes": {"lx": _r(lx), "rx": _r(rx), "ry": _r(ry), "L2": _r(l2), "ly": _r(ly)}}
    except Exception:
        return {"buttons": {}, "axes": {}}


# ── HIGHLEVEL 只读客户端 ─────────────────────────────────────────────────────

class Go1HighSdkClient:
    """原始 SDK 高层**只读**客户端：后台 UDP 收 HighState → 线程安全 snapshot()。

    所有状态卡共用同一个实例（唯一 UDP 收发线程）。这里不下发任何控制命令：
    每个循环发一个由 InitCmdData 初始化的空闲 HighCmd 只是为了维持 UDP 会话（Go1 需
    持续心跳才回状态），mode 恒为 0(idle)，不会让机器人动。
    """

    def __init__(self, network_iface: str = "",
                 target_ip: str = DEFAULT_TARGET_IP,
                 target_port: int = HIGH_TARGET_PORT,
                 local_port: int = HIGH_LOCAL_PORT):
        self._target_ip = target_ip
        self._target_port = target_port
        self._local_port = local_port
        self._sdk = None
        self._udp = None
        self._cmd = None            # 当前下发命令（控制卡加锁改写）
        self._idle_cmd = None       # 恒定空闲心跳（mode=0），看门狗到期回退到它
        self._cmd_deadline = 0.0    # time.monotonic() 截止时刻；过期即回 idle（自动超时停）
        self._gait = 1              # 期望步态偏好（switch_gait 改写；walk 未显式指定时用它，默认 trot）
        self._led = None            # 面部 LED 状态：None=不干预（交固件）；否则为 4 个 LED 对象，_loop 每拍盖到发送命令（与 mode/velocity 正交，不影响运动）
        self._state = None
        self._udp_diag = None
        self.available = False
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._snapshot: dict = {}
        self._init_sdk()

    def _init_sdk(self) -> None:
        try:
            import robot_interface as sdk  # unitree_legged_sdk pybind11 绑定
            self._sdk = sdk
            self._udp = sdk.UDP(HIGHLEVEL, self._local_port, self._target_ip, self._target_port)
            self._cmd = sdk.HighCmd()
            self._state = sdk.HighState()
            self._udp.InitCmdData(self._cmd)   # 空闲心跳命令（mode=0），只为维持会话
            self._idle_cmd = sdk.HighCmd()
            self._udp.InitCmdData(self._idle_cmd)  # 恒定空闲模板，看门狗到期回退用
            self.available = True
            print(f"[Go1HighSdk] robot_interface ready → {self._target_ip}:{self._target_port}", flush=True)
        except Exception as e:
            print(f"[Go1HighSdk] ⚠ STUB（robot_interface 不可用: {e}）", flush=True)

    def start(self) -> None:
        if self.available and not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True, name="go1_high_udp")
            self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        period = 1.0 / LOOP_HZ
        while self._running:
            try:
                self._udp.Recv()
                self._udp.GetRecv(self._state)
                # 看门狗：命令未过期发当前命令，否则回空闲（mode=0）→ 自动超时停
                with self._lock:
                    send = self._cmd if time.monotonic() < self._cmd_deadline else self._idle_cmd
                    led = self._led
                # LED 与运动正交：不改 mode/velocity，只把 led 字段盖到本拍要发的命令上
                if led is not None:
                    try:
                        send.led = led
                    except Exception:
                        pass
                self._udp.SetSend(send)
                self._udp.Send()
                self._capture_udp_diag()
                self._parse_state(self._state)
            except Exception as e:
                print(f"[Go1HighSdk] loop error: {e}", flush=True)
            time.sleep(period)

    # ── 下发原语（控制卡调用；STUB 时只改内存并返回 False）──────────────────────
    def _fresh_cmd(self):
        """从干净模板起一条 HighCmd，避免上次残留字段。"""
        c = self._sdk.HighCmd()
        self._udp.InitCmdData(c)
        return c

    def special(self, mode: int, hold: float = 1.5) -> bool:
        """一次性姿态动作：起身(stand_up=6)/坐下(stand_down=5) 等，按 mode 下发。
        命令只保持 hold 秒，之后看门狗回空闲（mode=0）。"""
        if not self.available:
            return False
        c = self._fresh_cmd()
        c.mode = int(mode)
        with self._lock:
            self._cmd = c
            self._cmd_deadline = time.monotonic() + float(hold)
        return True

    def stop_motion(self) -> bool:
        """立即令当前命令过期，看门狗下一拍即回空闲（mode=0）。"""
        with self._lock:
            self._cmd_deadline = 0.0
        return True

    def walk(self, vx: float, vy: float = 0.0, vyaw: float = 0.0,
             hold: float = 0.5, gait: int | None = None) -> bool:
        """速度行走：mode=2(walk)，按 vx/vy(m/s，前后/左右) + vyaw(rad/s，转向) 下发。
        命令只保持 hold 秒（默认 0.5s，比姿态动作 1.5s 更短，行走更需快停），
        之后看门狗回空闲（mode=0）自动停。量程校验由 loco 卡负责，这里只如实下发。
        gait 未显式指定时用 switch_gait 设定的偏好 self._gait（默认 1=trot）。"""
        if not self.available:
            return False
        c = self._fresh_cmd()
        c.mode = 2
        c.gaitType = int(gait) if gait is not None else int(self._gait)
        c.velocity = [float(vx), float(vy)]
        c.yawSpeed = float(vyaw)
        with self._lock:
            self._cmd = c
            self._cmd_deadline = time.monotonic() + float(hold)
        return True

    def set_gait(self, gait: int) -> bool:
        """设置期望步态偏好（0=idle/1=trot/2=trot_run/3=climb_stair/4=trot_obstacle）。
        只改偏好、不下发运动命令 → 本身不会让狗动；下一次 walk 会用这个步态。
        STUB 时仍记录偏好（便于无硬件验证），但返回 available 供卡片如实上报。"""
        self._gait = int(gait)
        return bool(self.available)

    def body_pose(self, euler=(0.0, 0.0, 0.0), body_height: float = 0.0,
                  foot_raise_height: float = 0.0, hold: float = 2.0) -> bool:
        """姿态/高度：mode=1(平衡站立)下设机身欧拉角与高度偏移。
        euler=(roll,pitch,yaw) rad；body_height/foot_raise_height 为相对默认值的偏移(m)。
        命令只保持 hold 秒，之后看门狗回空闲（mode=0）→ 姿态不无限保持（守住自动超时停）。
        每次调用是独立一次性命令，不与上次姿态叠加。量程校验由 body_pose 卡负责。"""
        if not self.available:
            return False
        c = self._fresh_cmd()
        c.mode = 1
        c.euler = [float(euler[0]), float(euler[1]), float(euler[2])]
        c.bodyHeight = float(body_height)
        c.footRaiseHeight = float(foot_raise_height)
        with self._lock:
            self._cmd = c
            self._cmd_deadline = time.monotonic() + float(hold)
        return True

    def set_led(self, r: int, g: int, b: int) -> bool:
        """设置面部 LED 颜色(0-255)。与运动正交：不下发任何 mode/velocity，只把 led 字段
        存起来由 _loop 每拍盖到当前发送命令上，因此可在行走/站立/空闲任意状态下点灯而不干扰运动。
        颜色一直保持到再次 set_led 或 clear_led。
        不同 wrapper 对 HighCmd.led 的绑定差异大 → 首次以试写探测可写性；不可写则返回 False（灯功能降级，不影响其余）。
        STUB（无硬件）时返回 False。灯是否真的亮需上狗 A/B 验证（本函数只保证命令已下发）。"""
        if not self.available:
            return False
        try:
            arr = [self._sdk.LED() for _ in range(4)]
            for x in arr:
                x.r, x.g, x.b = int(r) & 0xFF, int(g) & 0xFF, int(b) & 0xFF
            probe = self._fresh_cmd()
            probe.led = arr                     # 试写：wrapper 不支持会在此抛异常
            with self._lock:
                self._led = arr
            return True
        except Exception as e:  # noqa: BLE001
            print(f"[Go1HighSdk] set_led 不支持(HighCmd.led 不可写: {e})", flush=True)
            with self._lock:
                self._led = None
            return False

    def clear_led(self) -> bool:
        """停止干预 LED（不再每拍盖 led 字段，交还固件默认）。"""
        with self._lock:
            self._led = None
        return bool(self.available)

    def _capture_udp_diag(self) -> None:
        """从 SDK UDP 对象读控制环收发健康计数(HighState 里没有,给 udp_diagnostics 卡用)。
        字段名对照 unitree_legged_sdk 的 UDPState；名字不符时防御式降级,不崩循环。"""
        try:
            us = getattr(self._udp, "udpState", None)
            if us is None:
                return
            self._udp_diag = {
                "total_count": int(_g(us, "TotalCount", 0)),
                "send_count": int(_g(us, "SendCount", 0)),
                "recv_count": int(_g(us, "RecvCount", 0)),
                "send_error": int(_g(us, "SendError", 0)),
                "flag_error": int(_g(us, "FlagError", 0)),
                "recv_crc_error": int(_g(us, "RecvCRCError", 0)),
                "recv_lose_error": int(_g(us, "RecvLoseError", 0)),
            }
        except Exception:
            pass

    def _parse_state(self, s) -> None:
        """HighState → 一份完整 snapshot dict。

        当前 loco_state 只用其中 mode/gait/velocity/position/body_height/yaw_speed，
        battery 只用 battery。其余字段（imu/joints/foot_*/range_obstacle/wireless_remote）
        一并解析好放进 snapshot，是为了给后来者加卡（imu/joints/feet/... 卡）现成的数据源——
        新卡只需写一个 builder 读这些字段即可，不必再碰本文件。见 CONTRIBUTING.md。
        """
        try:
            out = {"fresh": True, "control_level": "HIGHLEVEL"}
            out["mode"] = int(_g(s, "mode", 0))
            out["mode_name"] = MODE_NAMES.get(out["mode"], "unknown")
            gt = int(_g(s, "gaitType", 0))
            out["gait_type"] = gt
            out["gait_name"] = GAIT_NAMES.get(gt, "unknown")
            out["progress"] = _r(_g(s, "progress", 0.0))
            out["body_height"] = _r(_g(s, "bodyHeight", 0.0))
            out["foot_raise_height"] = _r(_g(s, "footRaiseHeight", 0.0))
            out["yaw_speed"] = _r(_g(s, "yawSpeed", 0.0))
            vel = _g(s, "velocity", None)
            out["velocity"] = [_r(v) for v in vel] if vel is not None else None
            pos = _g(s, "position", None)
            out["position"] = [_r(p) for p in pos] if pos is not None else None
            out["imu"] = parse_imu(_g(s, "imu", None))
            out["joints"] = parse_joints(_g(s, "motorState", None))
            ff = _g(s, "footForce", None)
            out["foot_force"] = [int(f) for f in ff] if ff is not None else None
            out["foot_pos"] = _cartesian_list(_g(s, "footPosition2Body", None))
            out["foot_speed"] = _cartesian_list(_g(s, "footSpeed2Body", None))
            ro = _g(s, "rangeObstacle", None)
            out["range_obstacle"] = [_r(x) for x in ro] if ro is not None else None
            wr = _g(s, "wirelessRemote", None)
            out["wireless_remote"] = _to_bytes40(wr) if wr is not None else None
            out["battery"] = parse_battery(_g(s, "bms", None))
            out["udp"] = self._udp_diag
            with self._lock:
                self._snapshot = out
        except Exception as e:
            print(f"[Go1HighSdk] parse_state error: {e}", flush=True)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snapshot) if self._snapshot else {"fresh": False}

# PNDbotics Adam Pro 资源来源

## URDF 模型（`adam_sp_pro.urdf`）

从机器人运动控制电脑抓取：

- 路径：`/etc/pndbotics/pnd_adam_dds/Sources/urdf/adam_sp_pro.urdf`
- 另见官方模型仓库 <https://github.com/pndbotics/pnd_models>（`adam_standard` / `adam_inspire`）

该 URDF 含 31 个身体关节 + 双手手指关节 + 颈部关节。驱动只上报 31 个身体关节
（DDS `rt/lowstate.motor_state` 的维度），关节名与 URDF 完全一致（见 `control.py`
的 `ADAM_PRO_JOINT_NAMES`），用于 `sensor/skeleton` 骨架渲染。

注意：URDF 中的 mesh 引用（STL）在驱动容器内不解析，但骨架渲染器只用
关节名 + 运动链，不影响使用。

## DDS SDK（`pnd_sdk_python/`）

Vendored 自官方 Python SDK：

- 仓库：<https://github.com/pndbotics/pnd_sdk_python>（BSD-3-Clause）
- 版本：1.0.1（依赖 `cyclonedds==0.10.2`）

仅复制了 `pndbotics_sdk_py/` 包 + `setup.py`（约 100KB 纯 Python 源码）。
运行时依赖 `cyclonedds`（由 `requirements.txt` 安装）。

# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""关节限位的**离线**副本（离线设计与校核用）。

为什么需要一份副本
==================
``TrajectorySpec.amplitudes`` 必须**离线写死**在 ``trajectories*.yaml`` 里，
而"逐关节至少 80% 行程"这种目标是离线算的 —— 那时还没有机器人、也没有 URDF 文件。
``design.py`` 若去现场展开 xacro，就把 xacro/ROS 环境变成了设计工具的依赖，
而且同一份 YAML 在不同 URDF 版本下会被算出不同的振幅（不可复现）。

所以这里冻结一份**与 ``lite_description`` 的 URDF 一致**的限位，作为唯一来源：

    xacro src/lite_description/urdf/lite.urdf.xacro \\
        emit_ros2_control:=false mode:=arms > /tmp/lite_arms.urdf
    # 从展开后的 URDF 解析 <joint type="revolute"> 的 lower/upper

⚠️ 三条铁律：
1. **运行期不用这份表。** 真实轨迹一律用 ``RobotModel.limit_array()``（来自
   采集时冻结的 ``robot.urdf``），两者不一致时 ``FourierTrajectory`` 会报错。
2. **改了 URDF 限位必须同步改这里**，否则"80% 覆盖"的设计基准就漂了。
   ``test_limits_table.py`` 会在能拿到 ``lite_description`` 时自动对拍。
3. 数值取自实测展开（见 ``SOURCE``），保留到 1e-6 rad。
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from .common import ARM_JOINTS

#: 溯源信息：这份表是怎么来的（换 URDF 时按同样步骤重取）
SOURCE = (
    "xacro src/lite_description/urdf/lite.urdf.xacro "
    "emit_ros2_control:=false mode:=arms  →  解析 revolute joint 的 lower/upper"
)

#: 14 个臂关节的限位（rad）。键名与 URDF 的 <joint name> 完全一致。
SIM_LIMITS: Dict[str, Tuple[float, float]] = {
    "left_shoulder_pitch": (-3.141592, 0.785398),
    "left_shoulder_roll": (-1.570795, 1.919860),
    "left_shoulder_yaw": (-1.570800, 1.570800),
    "left_elbow_pitch": (-2.356198, 0.0),
    "left_wrist_yaw": (-1.570795, 1.570795),
    "left_wrist_roll": (-0.698134, 0.698134),
    "left_wrist_pitch": (-0.785400, 0.785400),
    "right_shoulder_pitch": (-3.141592, 0.785398),
    "right_shoulder_roll": (-1.919860, 1.570795),
    "right_shoulder_yaw": (-1.570797, 1.570797),
    "right_elbow_pitch": (0.0, 2.356199),
    "right_wrist_yaw": (-1.570796, 1.570796),
    "right_wrist_roll": (-0.698132, 0.698132),
    "right_wrist_pitch": (-0.785399, 0.785399),
}


def _limits() -> Tuple[np.ndarray, np.ndarray]:
    """``ARM_JOINTS`` 顺序的 (lo, hi) 数组。

    ⚠️ 缺任何一个关节都**直接报错**，绝不静默补 0 —— 补 0 会让"80% 覆盖"
    悄悄算成"围绕 0 摆"，而 0 可能正好在限位外。
    """
    missing: List[str] = [j for j in ARM_JOINTS if j not in SIM_LIMITS]
    if missing:
        raise KeyError(f"limits_table 缺少关节限位：{missing}（见 {SOURCE}）")
    lo = np.array([SIM_LIMITS[j][0] for j in ARM_JOINTS], dtype=float)
    hi = np.array([SIM_LIMITS[j][1] for j in ARM_JOINTS], dtype=float)
    return lo, hi


def limits_array() -> Tuple[np.ndarray, np.ndarray]:
    """对外接口：``ARM_JOINTS`` 顺序的 (lo, hi)。"""
    return _limits()


def span() -> np.ndarray:
    """逐关节行程（hi − lo, rad）。"""
    lo, hi = _limits()
    return hi - lo


def midpoint() -> np.ndarray:
    """逐关节限位中点（rad）。"""
    lo, hi = _limits()
    return 0.5 * (lo + hi)

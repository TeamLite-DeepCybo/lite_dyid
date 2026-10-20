# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""lite_dyid —— 常量、错误类型与路径工具。"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import List

#: `mode:=arms` 下 controller_manager / 桥 / 驱动的关节顺序（**唯一事实来源**）
ARM_JOINTS: List[str] = [
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow_pitch", "right_wrist_yaw", "right_wrist_roll", "right_wrist_pitch",
]

#: 左手 7 关节 → 右手镜像的**符号**（pitch 保号；roll / yaw 取反）
MIRROR_SIGN = (1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0)

# ---------------------------------------------------------------------------
# 逐关节速度上限（大幅档用）
# ---------------------------------------------------------------------------
#: 肩部（含肘）峰值速度上限 rad/s。
#: 实测：肩部质量/惯量最大，摆太快会让 CSP 位置环的跟随误差与堵转保护一起吃紧；
#: 0.2 rad/s ≈ 11.5°/s，与额定力矩余量匹配。
V_CAP_SHOULDER = 0.2
#: 腕部峰值速度上限 rad/s。
#: 腕部 3 关节行程小（wrist_roll 仅 ±40°）、惯量最小，可以摆到 0.4 rad/s。
V_CAP_WRIST = 0.4


def velocity_cap_of(joint: str) -> float:
    """单个关节的峰值速度上限（rad/s）。

    ⚠️ 分类依据是**关节在链上的位置**（shoulder/elbow = 大臂段，wrist = 小臂段），
    不是名字里有没有 "wrist" 之外的任何约定。``*_gripper`` 不参与辨识，
    返回腕部上限只是为了让调用方拿到一个有限值。
    """
    name = str(joint)
    if "wrist" in name:
        return V_CAP_WRIST
    return V_CAP_SHOULDER


def velocity_caps() -> List[float]:
    """``ARM_JOINTS`` 顺序的逐关节速度上限（rad/s）。"""
    return [velocity_cap_of(j) for j in ARM_JOINTS]

LEFT_JOINTS = ARM_JOINTS[:7]
RIGHT_JOINTS = ARM_JOINTS[7:]

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

#: 关节名 → 所属臂（用于日志与分组）
def arm_of(joint: str) -> str:
    return "left" if joint.startswith("left_") else "right"


class DyidError(RuntimeError):
    """本包所有可预期的失败（前置条件不满足 / 数据不足 / 安全中止）。"""


# ---------------------------------------------------------------------------
# 话题 / 服务（与 lite_bridge 的对外接口一致）
# ---------------------------------------------------------------------------

TOPIC_MOTION_COMMAND = "/lite_bridge/motion_command"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_SAFETY_STATUS = "/safety_status"
TOPIC_ROBOT_DESCRIPTION = "/robot_description"

SRV_SET_MODE = "/lite_bridge/set_mode"
SRV_GET_STATUS = "/lite_bridge/get_status"
SRV_STANDBY = "/lite_bridge/standby"
SRV_CLEAR_FAULT = "/lite_bridge/clear_fault"
SRV_SET_SOURCE = "/lite_bridge/set_source"   # 数据源干预（LOCK/HANDOVER/AUTO）

#: MotionCommand.mode
MODE_CSP = 0
MODE_MIT = 1

#: SetMode.protocol
PROTOCOL_MIT = 0
PROTOCOL_CSP = 1
PROTOCOL_KEEP = 255

#: SetMode.mode（CSP 组）
CSP_MONITOR = 5    # 只读，不使能（电机自由）
CSP_HOLD = 6       # 使能，锁在进档姿态
CSP_TRACKING = 7   # 使能，跟随命令

#: GetBridgeStatus.safety_level
SAFETY_OK, SAFETY_WARNING, SAFETY_FAULT, SAFETY_CRITICAL = 0, 1, 2, 3


def share_dir() -> Path:
    """本包的 share 目录（安装后）。开发期回退到源码树。"""
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "share" / "lite_dyid"
        if cand.is_dir():
            return cand
    return here.parent.parent


def config_path(name: str) -> Path:
    """找 config/<name>：优先 share（安装后），其次源码树。"""
    cand = share_dir() / "config" / name
    if cand.is_file():
        return cand
    src = Path(__file__).resolve().parent.parent / "config" / name
    if src.is_file():
        return src
    raise DyidError(f"找不到配置文件 {name}（试过 {cand} 与 {src}）")


def default_output_dir() -> Path:
    """结果目录：$LITE_DYID_OUT 优先，其次 ~/.ros/lite_dyid/runs。"""
    env = os.environ.get("LITE_DYID_OUT")
    if env:
        return Path(env).expanduser()
    return Path.home() / ".ros" / "lite_dyid" / "runs"

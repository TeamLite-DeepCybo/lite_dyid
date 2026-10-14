# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""与 lite_bridge 的唯一出口：状态查询、模式切换、CSP 位置命令。

信息通路（本包**只当数据源**，不改动任何底层包）
================================================
    本包 ──MotionCommand(mode=CSP)──▶ /lite_bridge/motion_command
         ──桥：归一化 + I5 全覆盖校验 + 仲裁──▶ JointCommand
         ──▶ /csp_tracking_controller/command ──▶ CspTrackingController
         ──▶ 驱动（写 loc_ref）──▶ 电机

三条硬约束（读桥与控制器源码得到）
==================================
1. **必须给全部 14 个关节**。桥的安全不变量 I5 会拒绝"只覆盖部分关节"的命令
   （CSP 下 0 是绝对目标 = 命令电机奔向零点）。
2. **``mode`` 必须与桥当前协议一致**（CSP=0），否则整帧被拒。
3. ``JointCommand.position`` 长度必须等于控制器的关节数、且顺序完全一致。

模式（CSP 组）
=============
    5 CSP_MONITOR   只读，**不使能**（电机自由）
    6 CSP_HOLD      使能，锁在进档姿态
    7 CSP_TRACKING  使能，跟随命令
跨协议切换会被桥**拒绝**并返回所需 launch 参数 —— 本包据此给出明确提示。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence

import numpy as np

from .common import (
    ARM_JOINTS,
    CSP_HOLD,
    CSP_MONITOR,
    CSP_TRACKING,
    MODE_CSP,
    PROTOCOL_CSP,
    PROTOCOL_KEEP,
    SAFETY_FAULT,
    SRV_CLEAR_FAULT,
    SRV_GET_STATUS,
    SRV_SET_MODE,
    SRV_STANDBY,
    TOPIC_JOINT_STATES,
    TOPIC_MOTION_COMMAND,
    TOPIC_ROBOT_DESCRIPTION,
    TOPIC_SAFETY_STATUS,
    DyidError,
)


@dataclass
class JointSnapshot:
    stamp: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.full(len(ARM_JOINTS), np.nan))
    velocity: np.ndarray = field(default_factory=lambda: np.full(len(ARM_JOINTS), np.nan))
    effort: np.ndarray = field(default_factory=lambda: np.full(len(ARM_JOINTS), np.nan))

    def age(self) -> float:
        return time.monotonic() - self.stamp


class BridgeClient:
    """一个节点 + 一个执行器线程；主线程只做控制循环。"""

    def __init__(
        self,
        node_name: str = "lite_dyid",
        *,
        source: str = "dyid",
        command_topic: str = TOPIC_MOTION_COMMAND,
        command_timeout_ms: float = 300.0,
    ) -> None:
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from robstride_msgs.msg import MotionCommand, SafetyStatus
        from robstride_msgs.srv import GetBridgeStatus, SetMode
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        self.source = source
        self.command_timeout_ms = float(command_timeout_ms)

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self._lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        self._snapshot = JointSnapshot()
        self._have_all = False
        self._safety_level = 0
        self._safety_flags = 0
        self._safety_source = ""
        self._urdf: Optional[str] = None

        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        self._MotionCommand = MotionCommand
        self.node.create_subscription(JointState, TOPIC_JOINT_STATES, self._on_js, rel,
                                      callback_group=self._cb_group)
        self.node.create_subscription(SafetyStatus, TOPIC_SAFETY_STATUS, self._on_safety,
                                      latched, callback_group=self._cb_group)
        self.node.create_subscription(String, TOPIC_ROBOT_DESCRIPTION, self._on_urdf,
                                      latched, callback_group=self._cb_group)

        self.pub = self.node.create_publisher(MotionCommand, command_topic, rel)

        self._set_mode_cli = self.node.create_client(SetMode, SRV_SET_MODE,
                                                     callback_group=self._cb_group)
        self._get_status_cli = self.node.create_client(GetBridgeStatus, SRV_GET_STATUS,
                                                       callback_group=self._cb_group)
        self._standby_cli = self.node.create_client(Trigger, SRV_STANDBY,
                                                    callback_group=self._cb_group)
        self._clear_fault_cli = self.node.create_client(Trigger, SRV_CLEAR_FAULT,
                                                        callback_group=self._cb_group)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True,
                                             name="lite_dyid-exec")
        self._spin_thread.start()

    # -- 生命周期 ---------------------------------------------------------

    def shutdown(self) -> None:
        # ⚠️ 顺序很重要：先让执行器停下并**等线程真正退出**，再销毁节点。
        #    否则线程还在 spin 时节点被销毁 —— 实测会
        #    `terminate called without an active exception` + core dump（退出码 134）。
        try:
            self._executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=3.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

    def log(self, text: str) -> None:
        self.node.get_logger().info(text)

    def warn(self, text: str) -> None:
        self.node.get_logger().warn(text)

    # -- 回调 -------------------------------------------------------------

    def _on_js(self, msg) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        pos = np.full(len(ARM_JOINTS), np.nan)
        vel = np.full(len(ARM_JOINTS), np.nan)
        eff = np.full(len(ARM_JOINTS), np.nan)
        found = 0
        for k, j in enumerate(ARM_JOINTS):
            i = idx.get(j)
            if i is None:
                continue
            found += 1
            if i < len(msg.position):
                pos[k] = msg.position[i]
            if i < len(msg.velocity):
                vel[k] = msg.velocity[i]
            if i < len(msg.effort):
                eff[k] = msg.effort[i]
        with self._lock:
            self._snapshot = JointSnapshot(stamp=time.monotonic(), position=pos,
                                           velocity=vel, effort=eff)
            self._have_all = (found == len(ARM_JOINTS))

    def _on_safety(self, msg) -> None:
        with self._lock:
            self._safety_level = int(msg.level)
            self._safety_flags = int(msg.flags)
            self._safety_source = str(msg.source)

    def _on_urdf(self, msg) -> None:
        if msg.data:
            with self._lock:
                self._urdf = msg.data

    # -- 状态 -------------------------------------------------------------

    def snapshot(self) -> JointSnapshot:
        with self._lock:
            s = self._snapshot
            return JointSnapshot(stamp=s.stamp, position=s.position.copy(),
                                 velocity=s.velocity.copy(), effort=s.effort.copy())

    def pose_dict(self) -> Dict[str, float]:
        s = self.snapshot()
        if not np.all(np.isfinite(s.position)):
            raise DyidError("/joint_states 里 14 个关节尚未齐备（含 NaN）")
        return dict(zip(ARM_JOINTS, s.position))

    def safety(self):
        with self._lock:
            return self._safety_level, self._safety_flags, self._safety_source

    def wait_joint_states(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._have_all and self.snapshot().age() < 1.0:
                return
            time.sleep(0.05)
        raise DyidError(f"等待 {TOPIC_JOINT_STATES}（14 关节齐备）超时 {timeout:.0f}s")

    def wait_urdf(self, timeout: float = 15.0) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._urdf:
                    return self._urdf
            time.sleep(0.05)
        raise DyidError(f"等待 {TOPIC_ROBOT_DESCRIPTION} 超时（robot_state_publisher 没起？）")

    # -- 服务 -------------------------------------------------------------

    def _call(self, client, request, timeout: float, what: str):
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            raise DyidError(f"服务不可用：{what}（lite_bringup 起了吗？）")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise DyidError(f"{what} 调用超时（{timeout:.1f}s）")
            time.sleep(0.01)
        result = future.result()
        if result is None:
            raise DyidError(f"{what} 未返回结果")
        return result

    def get_status(self, timeout: float = 5.0):
        from robstride_msgs.srv import GetBridgeStatus
        return self._call(self._get_status_cli, GetBridgeStatus.Request(), timeout, SRV_GET_STATUS)

    def set_mode(self, protocol: int, mode: int, *, force: bool = False, timeout: float = 30.0):
        from robstride_msgs.srv import SetMode
        req = SetMode.Request()
        req.protocol = int(protocol)
        req.mode = int(mode)
        req.force = bool(force)
        return self._call(self._set_mode_cli, req, timeout, SRV_SET_MODE)

    def standby(self, timeout: float = 10.0):
        from std_srvs.srv import Trigger
        return self._call(self._standby_cli, Trigger.Request(), timeout, SRV_STANDBY)

    def clear_fault(self, timeout: float = 10.0):
        from std_srvs.srv import Trigger
        return self._call(self._clear_fault_cli, Trigger.Request(), timeout, SRV_CLEAR_FAULT)

    # -- 模式：确保 CSP + TRACKING ---------------------------------------

    def ensure_csp_ready(self, *, force_enter_tracking: bool = True) -> str:
        """检查协议 → 必要时切入 CSP → 6(CSP_HOLD) → 7(CSP_TRACKING)。

        返回一段人类可读的说明（用于日志）。
        跨协议切换不可在运行期完成 —— 桥会返回 restart_required，这里据此报错。
        """
        st = self.get_status()
        notes = [f"当前：protocol={st.protocol_name} mode={st.mode_name}({st.mode}) "
                 f"controller={st.active_controller}[{'active' if st.controller_active else 'inactive'}]"]

        if st.protocol != PROTOCOL_CSP:
            notes.append(f"协议是 {st.protocol_name}，尝试切到 CSP")
            res = self.set_mode(PROTOCOL_CSP, CSP_MONITOR, force=False)
            if res.restart_required or not res.ok:
                raise DyidError(
                    "当前 bringup 不是 CSP 组，运行期**不能**跨协议切换。请用下面的参数重启：\n"
                    f"    {res.required_launch_args or 'controller:=csp control_mode:=csp'}\n"
                    f"    桥的回复：{res.message}"
                )
            notes.append(f"已切到 {res.actual_protocol_name} / {res.actual_mode_name}")
            st = self.get_status()

        if not st.csp_monitor_loaded or not st.csp_tracking_loaded:
            raise DyidError("CSP 控制器未全部加载（需要 csp_monitor / csp_hold / csp_tracking）")

        if st.mode == CSP_MONITOR:
            res = self.set_mode(PROTOCOL_KEEP, CSP_HOLD)
            if not res.ok:
                raise DyidError(f"切 CSP_HOLD 失败：{res.message}")
            notes.append("已进 CSP_HOLD（使能，锁在进档姿态）")

        if force_enter_tracking:
            res = self.set_mode(PROTOCOL_KEEP, CSP_TRACKING)
            if not res.ok:
                raise DyidError(f"切 CSP_TRACKING 失败：{res.message}")
            if str(res.actual_mode_name) != "CSP_TRACKING":
                raise DyidError(f"模式回读不是 CSP_TRACKING，而是 {res.actual_mode_name}")
            notes.append("已进 CSP_TRACKING（跟随命令）")
        return "；".join(notes)

    def assert_healthy(self) -> None:
        level, flags, src = self.safety()
        if level >= SAFETY_FAULT:
            raise DyidError(
                f"/safety_status level={level} flags={flags}（{src}）—— 先 clear_fault / 排障"
            )

    # -- 命令 -------------------------------------------------------------

    def publish_position(self, q: Sequence[float]) -> None:
        """发布一帧 CSP 位置命令（必须覆盖全部 14 个关节）。"""
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != len(ARM_JOINTS):
            raise DyidError(f"position 长度 {q.size} != {len(ARM_JOINTS)}（桥的 I5 会拒帧）")
        if not np.all(np.isfinite(q)):
            raise DyidError("position 含 NaN/Inf")
        msg = self._MotionCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = self.source
        msg.mode = MODE_CSP
        msg.timeout_ms = self.command_timeout_ms
        msg.joint_names = list(ARM_JOINTS)
        msg.position = [float(x) for x in q]
        self.pub.publish(msg)

    def publish_hold(self, positions: Sequence[float]) -> None:
        self.publish_position(positions)

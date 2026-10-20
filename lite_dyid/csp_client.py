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
from typing import Dict, List, Optional, Sequence

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
    SRV_SET_SOURCE,
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
        from rclpy.executors import SingleThreadedExecutor
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
        # 释放数据源干预（采集收尾用：把控制权还回去）
        from robstride_msgs.srv import SetSource
        self._SetSource = SetSource
        self._set_source_cli = self.node.create_client(SetSource, SRV_SET_SOURCE,
                                                       callback_group=self._cb_group)

        # 一个后台执行器线程已经足够：订阅回调只复制最新状态，服务请求由
        # 主线程发出后等待 future，响应仍由本线程处理。此前这里使用
        # MultiThreadedExecutor(2)，真机实测其两个 Python worker 与 200 Hz
        # 控制主线程争用 GIL：请求 200 Hz 实际仅约 115 Hz，最长断流 334 ms，
        # CSP 控制器因此反复判命令陈旧。单线程执行器避免这类调度争用。
        self._executor = SingleThreadedExecutor()
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

    def lock_source(self, timeout: float = 10.0):
        """**硬锁**本源（``SetSource{mode: LOCK, source: 自己}``）。

        ⚠️ 为什么采集期必须锁：本包在采集时是一条**长达几分钟**的连续激励轨迹。
        若不锁，桥的 ``SourceArbiter`` 只按"优先级 + 独占"裁定 —— 一旦有更高
        优先级且 ``exclusive`` 的源（例如 ``llm``）突然发命令，控制权会被抢走：
          · 本包仍在按轨迹下发，但命令被桥**丢弃**；
          · 机械臂停在原地不动，而 ``dynamics.csv`` 里 ``cmd`` 还在变、``q`` 不动
            → 数据里出现一段"命令与实测完全脱节"的样本，**静默污染辨识**。
        锁源之后本源"只要在发命令就一直是它"，上述抢占不可能发生。
        """
        req = self._SetSource.Request()
        req.mode = self._SetSource.Request.MODE_LOCK
        req.source = str(self.source)
        req.hold_ms = 0.0
        return self._call(self._set_source_cli, req, timeout, SRV_SET_SOURCE)

    def set_source_auto(self, timeout: float = 10.0):
        """解除数据源干预，回到自动仲裁（``SetSource{mode: AUTO}``）。

        ⚠️ 为什么采集收尾必须调它：本包通过 ``MotionCommand.source`` 声明自己是
        ``dyid``；一旦停发，桥的 ``SourceArbiter`` 会因超时进 ``SAFE_HOLD``
        并**保持**最后目标。把干预解除后控制权就干净地还回去，
        操作员 / 其它源可以立刻接管，而不是卡在"dyid 还占着"的状态。
        """
        req = self._SetSource.Request()
        req.mode = self._SetSource.Request.MODE_AUTO
        req.source = ""
        req.hold_ms = 0.0
        return self._call(self._set_source_cli, req, timeout, SRV_SET_SOURCE)

    # -- 收尾：先停发 → 回安全档 → 再交权 ---------------------------------

    def finish_session(self, *, entered_tracking: bool,
                       held_position: Optional[Sequence[float]] = None,
                       settle_sec: float = 1.0) -> str:
        """采集结束的**收尾序列**，返回人类可读的总结（供日志）。

        顺序（每一步的理由都写在下面，**不要调换**）：

          ① **停发** MotionCommand —— 先停发再动模式，避免"还在发旧目标、
             同时又在切控制器"这种目标来源不唯一的状态。
          ② 等 ``settle_sec``（默认 1.0 s）—— 让桥发现本源已失活并进 SAFE_HOLD。
             桥的 ``source_timeouts_ms`` 默认 500 ms，命令自身的 ``timeout_ms``
             是 300 ms，1.0 s 对两者都有余量。
          ③ ``set_mode{CSP_HOLD}``（只在**真的进过 TRACKING** 时）——
             把机械臂保持在当前目标。⚠️ 此时**不能**切 ``CSP_MONITOR(5)``：
             CSP 没有"松劲"，切 5 会失能 → 机械臂自由落体。
          ④ ``set_source{AUTO}`` —— 解除数据源干预，把控制权干净地还回去。
             放在最后：它只影响仲裁，不影响"谁在写 loc_ref"。

        ``held_position``：想把机械臂钉在某个位置时传入（默认不额外发帧）。
        """
        notes: List[str] = []

        # ① 停发（销毁 publisher 会立刻停止出流；再显式打一条日志便于对时间线）
        try:
            if held_position is not None:
                # 最后一帧：把目标钉在我们希望保持的位置上，再停发
                self.publish_position(held_position)
            self.node.destroy_publisher(self.pub)
            notes.append("① 已停止下发 MotionCommand")
        except Exception as exc:                       # 停发失败不致命
            notes.append(f"① 停发时异常（忽略）：{exc}")

        # ② 等桥侧超时
        time.sleep(max(0.0, float(settle_sec)))
        notes.append(f"② 已等待 {settle_sec:.1f}s（桥侧判本源失活 → SAFE_HOLD）")

        # ③ 回安全档
        if entered_tracking:
            try:
                res = self.set_mode(PROTOCOL_KEEP, CSP_HOLD, timeout=15.0)
                ok = getattr(res, "ok", None)
                notes.append(
                    f"③ set_mode(CSP_HOLD) ok={ok} "
                    f"actual_mode={getattr(res, 'actual_mode', '?')}"
                    f"（{getattr(res, 'actual_mode_name', '')}）")
            except Exception as exc:
                notes.append(f"③ 切 CSP_HOLD 失败：{exc}（机械臂仍由桥 SAFE_HOLD 顶着）")
        else:
            notes.append("③ 未进入过 TRACKING（电机未使能）→ 不切模式，保持原状")

        # ④ 交权
        try:
            res = self.set_source_auto(timeout=10.0)
            ok = getattr(res, "ok", None)
            notes.append(f"④ set_source(AUTO) ok={ok} "
                         f"active_source='{getattr(res, 'active_source', '')}' "
                         f"state={getattr(res, 'arbitration_state_name', '')}")
        except Exception as exc:
            notes.append(f"④ 解除数据源干预失败：{exc}（可手动 "
                         f"ros2 service call {SRV_SET_SOURCE} "
                         f"robstride_msgs/srv/SetSource \"{{mode: 0}}\"）")

        return "；".join(notes)

    # -- 模式：确保 CSP + TRACKING ---------------------------------------

    def ensure_csp_ready(self, *, force_enter_tracking: bool = True) -> str:
        """在任何位置命令发出前确认 CSP，并进入 HOLD → TRACKING。

        返回一段人类可读的说明（用于日志）。
        MIT/CSP 是驱动启动期协议，运行期不能跨协议切换；检测到 MIT 时本方法
        直接失败并给出重启参数，绝不会继续发布 CSP 位置命令。
        """
        st = self.get_status()
        if not getattr(st, "ok", False):
            raise DyidError(f"读取桥状态失败：{getattr(st, 'message', '')}")
        notes = [f"当前：protocol={st.protocol_name} mode={st.mode_name}({st.mode}) "
                 f"controller={st.active_controller}[{'active' if st.controller_active else 'inactive'}]"]

        if st.protocol != PROTOCOL_CSP:
            raise DyidError(
                f"启动检查检测到 protocol={st.protocol_name}、mode={st.mode_name}。"
                "当前底层不是 CSP；MIT/CSP 由驱动启动参数决定，运行期不能切换，"
                "本次不会下发任何 CSP 位置命令。请停止当前 bringup 后用以下参数重启：\n"
                f"    controller:=csp control_mode:=csp source_names:={self.source}\n"
                "重启后先确认 /lite_bridge/get_status 返回 protocol=csp，再重新运行 dyid_collect。"
            )

        if not (st.csp_monitor_loaded and st.csp_hold_loaded and st.csp_tracking_loaded):
            raise DyidError("CSP 控制器未全部加载（需要 csp_monitor / csp_hold / csp_tracking）")

        # 无论启动时停在 MONITOR、HOLD 还是旧任务留下的 TRACKING，都先落到
        # CSP_HOLD。这样新任务接管时有一个已回读的静止中间态，不会沿用旧源目标。
        if st.mode != CSP_HOLD or not st.controller_active:
            res = self.set_mode(PROTOCOL_KEEP, CSP_HOLD)
            if (not getattr(res, "ok", False)
                    or getattr(res, "actual_protocol", None) != PROTOCOL_CSP
                    or getattr(res, "actual_mode", None) != CSP_HOLD
                    or not getattr(res, "controller_active", False)):
                raise DyidError(
                    "切 CSP_HOLD 后回读不一致："
                    f"ok={getattr(res, 'ok', False)} "
                    f"protocol={getattr(res, 'actual_protocol_name', '?')} "
                    f"mode={getattr(res, 'actual_mode_name', '?')} "
                    f"controller_active={getattr(res, 'controller_active', False)}；"
                    f"桥的回复：{getattr(res, 'message', '')}")
            notes.append("已切 CSP_HOLD 并回读控制器 active（锁在当前姿态）")
        else:
            notes.append("启动时已在 CSP_HOLD，控制器 active")

        # 在 HOLD 中先用"当前实测姿态"给本源保鲜。桥的第一帧只负责把源记成
        # online，仲裁 tick 后后续帧才会被授权；HOLD 控制器不会跟随这些帧，
        # 所以机械臂仍静止。必须先保鲜再 LOCK：若对一个从未发过帧的源直接
        # LOCK，桥会按设计进入 SAFE_HOLD；timeout_action=disable 时甚至会退到
        # CSP_MONITOR，使紧接着的 TRACKING 迁移守卫失败。
        q_hold = np.asarray(list(self.pose_dict().values()), dtype=float)
        for _ in range(6):
            self.publish_position(q_hold)
            time.sleep(0.03)

        # ★ 锁源：必须在进 TRACKING **之前**完成。
        #   理由见 `lock_source`：不锁的话，采集途中被更高优先级源抢占会
        #   **静默**污染 dynamics.csv（cmd 在变、q 不动）。
        try:
            lk = self.lock_source(timeout=10.0)
            if not getattr(lk, "ok", False):
                raise DyidError(
                    f"锁源失败：{getattr(lk, 'message', '')}；"
                    f"可用源：{list(getattr(lk, 'available_sources', []))}。"
                    f"请确认 bringup 传了 source_names:=<本包 source>")
            if not getattr(lk, "source_online", False):
                raise DyidError(
                    f"锁源后 '{self.source}' 仍不是 online；"
                    "桥没有收到 HOLD 保鲜帧，拒绝进入 TRACKING")
            if getattr(lk, "active_source", "") != self.source:
                raise DyidError(
                    f"锁源回读 active_source='{getattr(lk, 'active_source', '')}'，"
                    f"期望 '{self.source}'")
            notes.append(f"已锁定数据源 '{self.source}'（MODE_LOCK）")
        except DyidError:
            raise
        except Exception as exc:
            raise DyidError(f"锁源调用异常：{exc}")

        if force_enter_tracking:
            res = self.set_mode(PROTOCOL_KEEP, CSP_TRACKING)
            if (not getattr(res, "ok", False)
                    or getattr(res, "actual_protocol", None) != PROTOCOL_CSP
                    or getattr(res, "actual_mode", None) != CSP_TRACKING
                    or not getattr(res, "controller_active", False)):
                raise DyidError(
                    "切 CSP_TRACKING 后回读不一致："
                    f"ok={getattr(res, 'ok', False)} "
                    f"protocol={getattr(res, 'actual_protocol_name', '?')} "
                    f"mode={getattr(res, 'actual_mode_name', '?')} "
                    f"controller_active={getattr(res, 'controller_active', False)}；"
                    f"桥的回复：{getattr(res, 'message', '')}")

            # 再独立查询一次，避免只相信 set_mode 的单次响应。这里通过后主流程
            # 才会把 entered_tracking 置真、才允许小幅测试发布第一帧。
            verify = self.get_status()
            if (not getattr(verify, "ok", False)
                    or verify.protocol != PROTOCOL_CSP
                    or verify.mode != CSP_TRACKING
                    or not verify.controller_active):
                raise DyidError(
                    "CSP_TRACKING 二次状态确认失败："
                    f"protocol={getattr(verify, 'protocol_name', '?')} "
                    f"mode={getattr(verify, 'mode_name', '?')} "
                    f"controller={getattr(verify, 'active_controller', '?')} "
                    f"active={getattr(verify, 'controller_active', False)}")
            notes.append("已进 CSP_TRACKING，二次回读确认控制器 active")
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

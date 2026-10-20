# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""运动原语：限速平滑移动、保持、实时节拍循环。

为什么不用 JointTrajectoryController
====================================
本项目 CSP 组只有 ``csp_monitor / csp_hold / csp_tracking`` 三个控制器，
``csp_tracking`` 只接受**位置目标**（锁存并持续写 loc_ref），没有轨迹 action。
所以轨迹由本包在**上位机**按节拍展开成位置序列，逐帧下发。

实时性
======
上位机默认 200 Hz 下发。命令**陈旧也不松劲**
（CspTrackingController 保持锁存目标），所以偶发的调度抖动不会让机械臂塌下来
—— 这是选 CSP 而不是 MIT 的重要原因之一。
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

import numpy as np

from .common import ARM_JOINTS, DyidError

#: smoothstep ``x²(3−2x)`` 的峰值斜率
SMOOTHSTEP_PEAK = 1.5


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, float(x)))
    return x * x * (3.0 - 2.0 * x)


def smoothstep_dot(x: float, duration: float) -> float:
    x = min(1.0, max(0.0, float(x)))
    if duration <= 0.0 or x <= 0.0 or x >= 1.0:
        return 0.0
    return 6.0 * x * (1.0 - x) / duration


class RateLoop:
    """固定频率的墙钟节拍器（落后就重锚，不追帧）。"""

    def __init__(self, rate_hz: float) -> None:
        self.period = 1.0 / max(1e-3, float(rate_hz))
        self._next = 0.0
        self.frames = 0
        self.lag_max = 0.0

    def start(self) -> None:
        self._next = time.monotonic()
        self.frames = 0
        self.lag_max = 0.0

    def tick(self) -> None:
        self.frames += 1
        self._next += self.period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.lag_max = max(self.lag_max, -delay)
            self._next = time.monotonic()

    @property
    def elapsed(self) -> float:
        return self.frames * self.period


def move_joints(
    client,
    target: Sequence[float],
    *,
    rate_hz: float = 200.0,
    speed_cap: float = 0.10,
    start: Optional[Sequence[float]] = None,
    cancel: Optional[Callable[[], bool]] = None,
    on_sample: Optional[Callable[[np.ndarray, np.ndarray], None]] = None,
    label: str = "",
) -> float:
    """从**实测/给定**起始姿态按 smoothstep 移到 ``target``。

    时长由位移与 ``speed_cap`` 反算：``T = 1.5·max|Δ| / speed_cap``，
    因此无论位移多大，峰值速度都不超过 ``speed_cap``（宁可慢）。
    返回实际耗时（秒）。
    """
    tgt = np.asarray(target, dtype=float).reshape(-1)
    if tgt.size != len(ARM_JOINTS):
        raise DyidError(f"target 长度 {tgt.size} != {len(ARM_JOINTS)}")
    src = (np.asarray(start, dtype=float).reshape(-1) if start is not None
           else np.asarray(list(client.pose_dict().values()), dtype=float))
    delta = tgt - src
    travel = float(np.max(np.abs(delta)))
    if travel < 1e-6:
        client.publish_position(tgt)
        return 0.0
    duration = SMOOTHSTEP_PEAK * travel / max(1e-3, speed_cap)

    loop = RateLoop(rate_hz)
    loop.start()
    t0 = time.monotonic()
    while True:
        if cancel is not None and cancel():
            raise DyidError(f"移动被中断：{label}")
        elapsed = time.monotonic() - t0
        x = min(1.0, elapsed / duration)
        q = src + delta * smoothstep(x)
        client.publish_position(q)
        if on_sample is not None:
            snap = client.snapshot()
            on_sample(snap.position.copy(), snap.effort.copy())
        if x >= 1.0:
            break
        loop.tick()
    return time.monotonic() - t0


def hold_position(
    client,
    position: Sequence[float],
    *,
    duration_sec: float,
    rate_hz: float = 200.0,
    cancel: Optional[Callable[[], bool]] = None,
    on_sample: Optional[Callable[[np.ndarray, np.ndarray, float], None]] = None,
) -> None:
    """在 ``duration_sec`` 内持续下发同一个目标（保持锁存）。"""
    q = np.asarray(position, dtype=float).reshape(-1)
    loop = RateLoop(rate_hz)
    loop.start()
    t0 = time.monotonic()
    while True:
        if cancel is not None and cancel():
            raise DyidError("保持被中断")
        elapsed = time.monotonic() - t0
        if elapsed >= duration_sec:
            break
        client.publish_position(q)
        if on_sample is not None:
            snap = client.snapshot()
            on_sample(snap.position.copy(), snap.effort.copy(), elapsed)
        loop.tick()

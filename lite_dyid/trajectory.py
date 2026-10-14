# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""傅里叶激励轨迹：定义、加载、运行期中心/斜坡计算与安全校核。

轨迹形式（与 tmp/bar_dynamics_identify 的定义一致，便于复用其经验）
================================================================
    shape_j(t)  = Σ_k w_k · sin(2π f k t + φ_{j,k}) / Σ_k w_k
    q_j(t)      = center_j + amp_j · shape_j(t)
    v_j(t)      = amp_j · dshape_j/dt

进入段（``t < ramp``）用 smoothstep 从**实测起始姿态** ``entry`` 平滑过渡到
``center + amp·shape``：``q = entry + s(t)·(q_fourier − entry)``。
于是"从当前姿态出发"不需要额外的规划，也不会在 t=0 产生阶跃。

运行期才决定 ``center``
=======================
本包把 YAML 里的 ``bias`` 当作**相对起始姿态的偏置**，运行期：

    center_j = clip(entry_j + bias_j,  lo_j + |amp_j| + margin,  hi_j − |amp_j| − margin)

三个理由：
1. 每次上电的起始姿态不同（例如左肩 roll 本次停在 −89.3°，离下限只有 0.7°），
   写死 center 会越限或浪费行程；
2. ``|amp_j| = 0`` 的关节（不激励的那条臂）**严格**保持 entry，
   绝不能因为限位裁剪被动起来；
3. 裁剪范围不可行时立刻报错，而不是让驱动去静默钳位。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

from .common import ARM_JOINTS, DyidError

#: 稳态下每个关节至少离限位留出的余量（rad）
LIMIT_MARGIN = 0.10
#: smoothstep 的峰值斜率 = 1.5/ΔT，用它把过渡速度压在 cap 以内
SMOOTHSTEP_PEAK = 1.5


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


@dataclass
class TrajectorySpec:
    name: str
    role: str
    duration_sec: float
    ramp_sec: float
    frequency_hz: float
    bias: np.ndarray
    amplitudes: np.ndarray
    phases: np.ndarray
    harmonic_weights: np.ndarray
    harmonic_indices: np.ndarray
    normalization: str = "l1"
    #: 静止段：在 t = k·period（k=1..count）处插入 hold_sec 的静止
    static_insert_count: int = 20
    static_period_sec: float = 10.0
    static_hold_sec: float = 10.0
    #: 静止段末尾用于平均的时长（前面的时间留给过渡）
    static_settle_sec: float = 4.0

    def joint_static_times(self) -> List[float]:
        return [k * self.static_period_sec for k in range(1, self.static_insert_count + 1)]


def load_specs(path: Path) -> Dict[str, TrajectorySpec]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    defaults = payload.get("defaults", {}) or {}
    out: Dict[str, TrajectorySpec] = {}
    for raw in payload.get("trajectories", []):
        merged = {**defaults, **raw}
        name = str(merged["name"])
        n = len(ARM_JOINTS)

        def arr(key: str, size: int) -> np.ndarray:
            v = np.asarray(merged[key], dtype=float)
            if v.shape != (size,):
                raise DyidError(f"{name}.{key} 形状 {v.shape} != ({size},)")
            return v

        bias = arr("bias", n)
        amp = arr("amplitudes", n)
        phases = np.asarray(merged["phases"], dtype=float)
        if phases.shape != (n, 4):
            raise DyidError(f"{name}.phases 形状 {phases.shape} != ({n}, 4)")
        out[name] = TrajectorySpec(
            name=name,
            role=str(merged.get("role", "train")),
            duration_sec=float(merged.get("duration_sec", 200.0)),
            ramp_sec=float(merged.get("ramp_sec", 25.0)),
            frequency_hz=float(merged["frequency_hz"]),
            bias=bias,
            amplitudes=amp,
            phases=phases,
            harmonic_weights=np.asarray(merged.get("harmonic_weights", [1.0, 0.62, 0.42, 0.27]), float),
            harmonic_indices=np.asarray(merged.get("harmonic_indices", [1, 2, 3, 4]), float),
            normalization=str(merged.get("normalization", "l1")),
            static_insert_count=int(merged.get("static_insert_count", defaults.get("static_insert_count", 0))),
            static_period_sec=float(merged.get("static_period_sec", defaults.get("static_period_sec", 10.0))),
            static_hold_sec=float(merged.get("static_hold_sec", defaults.get("static_hold_sec", 10.0))),
            static_settle_sec=float(merged.get("static_settle_sec", defaults.get("static_settle_sec", 4.0))),
        )
    if not out:
        raise DyidError(f"{path} 里没有 trajectories")
    return out


@dataclass
class TrajectoryCheck:
    name: str
    center: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray
    v_peak: np.ndarray
    a_peak: np.ndarray
    limit_margin_deg: np.ndarray
    travel_deg: np.ndarray
    #: 只统计**稳态**（t >= ramp）的限位余量 —— 起始姿态本身可能贴着限位，
    #: 那是"上电就在那儿"，不是轨迹要求的；两者混在一起会让人误判。
    limit_margin_steady_deg: np.ndarray
    ok: bool
    problems: List[str] = field(default_factory=list)


class FourierTrajectory:
    """一次运行用的具体轨迹（起始姿态 + 限位已代入）。"""

    def __init__(
        self,
        spec: TrajectorySpec,
        entry_position: np.ndarray,
        limits: Tuple[np.ndarray, np.ndarray],
        *,
        speed_cap: float = 0.15,
        limit_margin: float = LIMIT_MARGIN,
    ) -> None:
        self.spec = spec
        entry = np.asarray(entry_position, dtype=float)
        if entry.shape != (len(ARM_JOINTS),):
            raise DyidError(f"entry_position 形状 {entry.shape} 不对")
        self.entry = entry
        lo, hi = (np.asarray(a, dtype=float) for a in limits)
        amp = spec.amplitudes
        bias = spec.bias

        center = entry.copy()
        for k in range(len(ARM_JOINTS)):
            if abs(amp[k]) < 1e-12:
                center[k] = entry[k]          # 不激励 → 严格不动
                continue
            inset = abs(amp[k]) + limit_margin
            if lo[k] + inset > hi[k] - inset:
                raise DyidError(
                    f"{spec.name}: {ARM_JOINTS[k]} 的行程不足以容纳 "
                    f"振幅 {math.degrees(abs(amp[k])):.1f}° + 余量 "
                    f"{math.degrees(limit_margin):.1f}°"
                )
            center[k] = float(np.clip(entry[k] + bias[k], lo[k] + inset, hi[k] - inset))
        self.center = center
        self.lo, self.hi = lo, hi

        # 过渡时长：保证进入段速度 ≤ speed_cap
        travel = float(np.max(np.abs(center - entry)))
        needed = SMOOTHSTEP_PEAK * travel / max(1e-6, speed_cap)
        self.ramp = max(float(spec.ramp_sec), needed)

        w = np.asarray(spec.harmonic_weights, dtype=float)
        if spec.normalization != "l1":
            raise DyidError(f"暂只支持 normalization=l1，得到 {spec.normalization}")
        self.w = w
        self.norm = float(np.sum(np.abs(w)))
        self.omega = 2.0 * math.pi * spec.frequency_hz * np.asarray(spec.harmonic_indices, float)

    # -- 计算 -------------------------------------------------------------

    def _shape(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        ang = self.omega[None, :] * t + self.spec.phases
        s = (np.sin(ang) * self.w).sum(axis=1) / self.norm
        sd = (np.cos(ang) * self.w * self.omega).sum(axis=1) / self.norm
        sdd = (-np.sin(ang) * self.w * self.omega ** 2).sum(axis=1) / self.norm
        return s, sd, sdd

    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (q, v)，单位 rad / (rad/s)。"""
        t = max(0.0, float(t))
        amp = self.spec.amplitudes
        s, sd, _ = self._shape(t)
        q_f, v_f = self.center + amp * s, amp * sd
        if t < self.ramp and self.ramp > 0.0:
            x = t / self.ramp
            al = smoothstep(x)
            dal = (6.0 * x * (1.0 - x) / self.ramp) if 0.0 < x < 1.0 else 0.0
            return self.entry + al * (q_f - self.entry), dal * (q_f - self.entry) + al * v_f
        return q_f, v_f

    @property
    def duration(self) -> float:
        return float(self.spec.duration_sec)

    # -- 校核 -------------------------------------------------------------

    def check(self, samples: int = 4000) -> TrajectoryCheck:
        ts = np.linspace(0.0, self.duration, samples)
        Q = np.zeros((samples, len(ARM_JOINTS)))
        V = np.zeros_like(Q)
        for i, t in enumerate(ts):
            Q[i], V[i] = self.evaluate(t)
        A = np.gradient(V, ts, axis=0)
        problems: List[str] = []
        for k, jn in enumerate(ARM_JOINTS):
            if Q[:, k].min() < self.lo[k] - 1e-9 or Q[:, k].max() > self.hi[k] + 1e-9:
                problems.append(f"{jn} 越限")
            if abs(self.spec.amplitudes[k]) > 1e-12 and "shoulder" in jn:
                if math.degrees(Q[:, k].max() - Q[:, k].min()) > 50.0:
                    problems.append(f"{jn} 变动 >50°")
        v_peak = np.abs(V).max()
        if v_peak > 0.20:
            problems.append(f"峰值速度 {v_peak:.3f} rad/s 过大")
        margin = np.array([
            math.degrees(min(Q[:, k].min() - self.lo[k], self.hi[k] - Q[:, k].max()))
            for k in range(len(ARM_JOINTS))
        ])
        steady = ts >= self.ramp
        margin_ss = np.array([
            math.degrees(min(Q[steady, k].min() - self.lo[k],
                             self.hi[k] - Q[steady, k].max()))
            for k in range(len(ARM_JOINTS))
        ])
        return TrajectoryCheck(
            name=self.spec.name,
            center=self.center,
            q_min=Q.min(axis=0),
            q_max=Q.max(axis=0),
            v_peak=np.abs(V).max(axis=0),
            a_peak=np.abs(A).max(axis=0),
            limit_margin_deg=margin,
            travel_deg=np.degrees(Q.max(axis=0) - Q.min(axis=0)),
            limit_margin_steady_deg=margin_ss,
            ok=not problems,
            problems=problems,
        )

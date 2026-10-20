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

from .common import ARM_JOINTS, DyidError, velocity_cap_of

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
    #: 工作点选择方式（交付轨迹用 midpoint）：
    #:   "entry_bias"（默认，向后兼容）—— center 由实测起始姿态 + bias 决定；
    #:   "midpoint"  —— center = 限位区间中点，保证左右行程对称。
    #:                 本机左肩 roll 常停在 −89.3°（下限 −90°），用 entry_bias
    #:                 只能单向摆 ≈54% 行程；midpoint 可覆盖 ≈93%。
    #: ⚠️ midpoint 不依赖实测起始姿态，所以**离线**设计即精确；
    #:    代价是进入段要先把关节从起始姿态平移到中点（见 approach_sec）。
    center_mode: str = "entry_bias"
    #: 进入段（起始姿态 → 工作点）的速度上限 rad/s。<=0 时用 speed_cap。
    approach_speed_cap: float = 0.0
    #: 逐关节**峰值归一化**：把 shape 除以其在名义周期内的峰值，
    #: 使 ``max|shape_j| == 1``，于是
    #:     实际行程 = 2·amp_j·1  （而不是 2·amp_j·(Σw 的部分和)）
    #: 为什么需要：四次谐波叠加的峰值取决于相位对齐，各关节相位不同，
    #: 只按名义 ``2·A·Σw`` 不能保证实际覆盖率。开启后 A 就是实际半行程，
    #: 覆盖率可以精确设计。
    #: 默认 False 是为了让外部自定义旧配置仍可读取；交付配置显式为 True。
    peak_normalize: bool = False
    #: 被激励关节的最小有效行程覆盖率（``(q_max-q_min)/(upper-lower)``）。
    #: 0 表示不检查；交付轨迹写 0.80，使运行期在真机 URDF 上再次硬校核。
    min_coverage: float = 0.0
    #: 被激励关节的最低峰值速度（rad/s）。用于避免大幅轨迹慢到近似静态、
    #: 摩擦项缺少可辨识信号；0 表示不检查。
    min_peak_speed: float = 0.0

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
            center_mode=str(merged.get("center_mode", defaults.get("center_mode", "entry_bias"))),
            approach_speed_cap=float(merged.get("approach_speed_cap",
                                                 defaults.get("approach_speed_cap", 0.0))),
            peak_normalize=bool(merged.get("peak_normalize",
                                           defaults.get("peak_normalize", False))),
            min_coverage=float(merged.get("min_coverage",
                                          defaults.get("min_coverage", 0.0))),
            min_peak_speed=float(merged.get("min_peak_speed",
                                            defaults.get("min_peak_speed", 0.0))),
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
    #: 逐关节限位行程（度）与速度上限（rad/s）
    span_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v_cap: np.ndarray = field(default_factory=lambda: np.zeros(0))
    #: 逐关节"行程覆盖率"= travel / span（0~1）与其目标值（0 = 不检查）
    coverage: np.ndarray = field(default_factory=lambda: np.zeros(0))
    coverage_target: np.ndarray = field(default_factory=lambda: np.zeros(0))
    ok: bool = True
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
        v_cap: Optional[np.ndarray] = None,
        coverage_target: Optional[np.ndarray] = None,
    ) -> None:
        self.spec = spec
        #: 被 _clamp() 改写的次数（非零 = 轨迹设计与限位不符，应重做设计）。
        #: ⚠️ 必须在**任何 evaluate() 之前**初始化：__init__ 里的峰值归一化
        #: 校正循环本身就会调用 evaluate() → _clamp()（实测踩过 AttributeError）。
        self.clamp_count = 0
        entry = np.asarray(entry_position, dtype=float)
        if entry.shape != (len(ARM_JOINTS),):
            raise DyidError(f"entry_position 形状 {entry.shape} 不对")
        self.entry = entry
        lo, hi = (np.asarray(a, dtype=float) for a in limits)
        amp = spec.amplitudes
        bias = spec.bias

        # --- 谐波系数（必须在限位裁剪之前：裁剪要用到 shape 包络）--------------
        w = np.asarray(spec.harmonic_weights, dtype=float)
        if spec.normalization != "l1":
            raise DyidError(f"暂只支持 normalization=l1，得到 {spec.normalization}")
        self.w = w
        self.norm = float(np.sum(np.abs(w)))
        self.omega = 2.0 * math.pi * spec.frequency_hz * np.asarray(spec.harmonic_indices, float)

        if spec.center_mode not in ("entry_bias", "midpoint"):
            raise DyidError(
                f"{spec.name}: center_mode='{spec.center_mode}' 未知"
                "（可选 entry_bias / midpoint）")

        center = entry.copy()

        # ``peak_normalize`` 的定义是把每个关节一个完整基频周期内的波形
        # 精确映射到 [-1, +1]。这样 YAML 里的 ``amplitudes`` 就是实际半行程，
        # ``amplitude = 0.4 * (upper-lower)`` 会得到可复核的 80% 覆盖率。
        #
        # 旧实现只除以 max(abs(shape))，没有消除波形的直流偏心；随后又在
        # evaluate() 里减 ramp 时刻的 shape。结果是名义 80% 可能只走 50%，
        # 还会频繁触发运行期钳位。这里用周期内 min/max 的中点和半峰峰值
        # 一次完成归一化，位置、速度和加速度共用同一尺度。
        if spec.peak_normalize:
            raw_min, raw_max = self._shape_range()
            self._shape_mid = 0.5 * (raw_min + raw_max)
            self._shape_half = 0.5 * (raw_max - raw_min)
            if np.any((np.abs(amp) > 1e-12) & (self._shape_half <= 1e-9)):
                bad = [ARM_JOINTS[k] for k in range(len(ARM_JOINTS))
                       if abs(amp[k]) > 1e-12 and self._shape_half[k] <= 1e-9]
                raise DyidError(f"{spec.name}: 谐波波形退化，无法峰峰值归一化：{bad}")
            self._shape_half = np.where(self._shape_half > 1e-9,
                                        self._shape_half, 1.0)
        else:
            self._shape_mid = np.zeros(len(ARM_JOINTS))
            self._shape_half = np.ones(len(ARM_JOINTS))

        amp_eff = np.abs(amp) if spec.peak_normalize else np.abs(amp) * self._shape_envelope()
        self.amp_eff = amp_eff
        for k in range(len(ARM_JOINTS)):
            if abs(amp_eff[k]) < 1e-12:
                center[k] = entry[k]          # 不激励 → 严格不动
                continue
            inset = float(amp_eff[k]) + limit_margin
            if lo[k] + inset > hi[k] - inset:
                raise DyidError(
                    f"{spec.name}: {ARM_JOINTS[k]} 的行程不足以容纳实际振幅 "
                    f"{math.degrees(abs(amp_eff[k])):.1f}° + 余量 "
                    f"{math.degrees(limit_margin):.1f}°"
                )
            if spec.center_mode == "midpoint":
                # 工作点 = 限位中点（与实测起始姿态无关，离线即精确）
                center[k] = 0.5 * (lo[k] + hi[k])
            else:
                center[k] = float(
                    np.clip(entry[k] + bias[k], lo[k] + inset, hi[k] - inset))
        self.center = center
        self.lo, self.hi = lo, hi

        # 过渡时长：进入段要把**所有**关节从 entry 移到工作点，速度 ≤ cap
        approach_cap = float(spec.approach_speed_cap) or float(speed_cap)
        travel = float(np.max(np.abs(self.center_steady() - entry)))
        needed = SMOOTHSTEP_PEAK * travel / max(1e-6, approach_cap)
        self.ramp = max(float(spec.ramp_sec), needed)

        # --- 逐关节速度上限与覆盖率目标 ---------------------------------------
        if v_cap is None:
            self.v_cap = np.array([max(velocity_cap_of(j), float(speed_cap))
                                   for j in ARM_JOINTS], dtype=float)
        else:
            self.v_cap = np.asarray(v_cap, dtype=float).reshape(-1)
            if self.v_cap.shape != (len(ARM_JOINTS),):
                raise DyidError(f"v_cap 形状 {self.v_cap.shape} 不对")
        if coverage_target is None:
            self.coverage_target = np.where(
                np.abs(spec.amplitudes) > 1e-12, float(spec.min_coverage), 0.0)
        else:
            self.coverage_target = np.asarray(coverage_target, dtype=float).reshape(-1)
        span = hi - lo
        self._span = span
        self._coverage_target = self.coverage_target

    def center_steady(self) -> np.ndarray:
        """进入段结束（= 稳态傅里叶起点）时的目标位置。"""
        center = self.center.copy()
        for k in range(len(ARM_JOINTS)):
            if abs(self.spec.amplitudes[k]) < 1e-12:
                center[k] = self.entry[k]
        return center

    # -- 计算 -------------------------------------------------------------

    def _shape_envelope(self) -> np.ndarray:
        """``max_t |Σ w_k sin(ω_k t + φ_k)| / Σw``（逐关节）。

        ⚠️ 这是**频率无关**的：把时间归一化成周期数后，t 的缩放被 ω 里的 f 抵消，
        所以包络只由相位与谐波权重决定。因此可以"先用包络定限位裁剪、再定频率"。
        """
        ts = np.linspace(0.0, 1.0, 2001)[:, None, None]
        k = np.asarray(self.spec.harmonic_indices, float)[None, None, :]
        ph = np.asarray(self.spec.phases, float)[None, :, :]
        ang = ts * k * 2.0 * math.pi + ph
        s = (np.sin(ang) * self.w[None, None, :]).sum(axis=2) / self.norm
        return np.max(np.abs(s), axis=0)

    def _shape_range(self) -> Tuple[np.ndarray, np.ndarray]:
        """一个完整基频周期内逐关节波形的 ``(min, max)``。"""
        # 8193 点能把四次谐波的峰值位置误差压到远低于 0.1%，同时只在
        # 构造轨迹时计算一次，不进入 200 Hz 控制循环。
        phase = np.linspace(0.0, 2.0 * math.pi, 8193)[:, None, None]
        k = np.asarray(self.spec.harmonic_indices, float)[None, None, :]
        ph = np.asarray(self.spec.phases, float)[None, :, :]
        raw = (np.sin(phase * k + ph) * self.w[None, None, :]).sum(axis=2) / self.norm
        return raw.min(axis=0), raw.max(axis=0)

    def _shape_raw(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        ang = self.omega[None, :] * t + self.spec.phases
        w = self.w
        s = (np.sin(ang) * w).sum(axis=1) / self.norm
        sd = (np.cos(ang) * w * self.omega).sum(axis=1) / self.norm
        sdd = (-np.sin(ang) * w * self.omega ** 2).sum(axis=1) / self.norm
        return s, sd, sdd

    def _shape(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        s, sd, sdd = self._shape_raw(t)
        if not self.spec.peak_normalize:
            return s, sd, sdd
        return ((s - self._shape_mid) / self._shape_half,
                sd / self._shape_half,
                sdd / self._shape_half)

    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (q, v)，单位 rad / (rad/s)。

        t=0 严格等于 ``entry``；t>=ramp 后为 ``center + amp·shape(t)``。
        """
        t = max(0.0, float(t))
        amp = self.spec.amplitudes
        s, sd, _ = self._shape(t)
        q_target = self.center + amp * s
        # |amp|=0 的关节：center 已等于 entry（见 __init__），这里再显式钉一次，
        # 保证严格不动（浮点意义上也不可能被 shape 带跑）。
        quiet = np.abs(amp) < 1e-12
        if quiet.any():
            q_target = np.where(quiet, self.entry, q_target)
        v_target = amp * sd
        if quiet.any():
            v_target = np.where(quiet, 0.0, v_target)
        if 0.0 < t < self.ramp and self.ramp > 0.0:
            x = t / self.ramp
            al = smoothstep(x)
            dal = 6.0 * x * (1.0 - x) / self.ramp
            q = self.entry + al * (q_target - self.entry)
            v = dal * (q_target - self.entry) + al * v_target
            return self._clamp(q), v
        if t <= 0.0 and self.ramp > 0.0:
            return self.entry.copy(), np.zeros(len(ARM_JOINTS))
        return self._clamp(q_target), v_target

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        """发布前的**最后一道硬保险**：把位置钳进 [lo, hi]。

        ⚠️ 为什么必须有这一道（而不是只靠构造期校核）：
        `peak_normalize` 的放大倍率是"包络"的函数，而包络依赖相位与谐波权重；
        一旦有人在 YAML 里手工改了 ``amplitudes`` / ``phases`` 而没重跑
        ``--check``，构造期的限位校核就可能与实际曲线不符。
        真实机械臂上"越限"是硬件损坏级别的事故，所以这里逐拍钳位 ——
        代价可忽略（14 个 min/max），收益是"越限不可能发生在真机上"。

        钳位会被 ``self.clamp_count`` 计数，调用方（采集/控制）可据此告警：
        非零就说明轨迹设计与限位不符，应当停下来重做设计。
        """
        out = np.minimum(np.maximum(q, self.lo), self.hi)
        if not np.array_equal(out, q):
            self.clamp_count += 1
        return out

    @property
    def duration(self) -> float:
        return float(self.spec.duration_sec)

    # -- 校核 -------------------------------------------------------------

    def check(self, samples: int = 4000) -> TrajectoryCheck:
        # ⚠️ 速度必须用 evaluate() 的**解析速度**（``_shape`` 的第二项），
        #    不能用 ``np.gradient`` 差分位置 —— 低频大幅档只有 2~3 个周期，
        #    4000 点的差分会把速度峰值低估 30~40%，于是"检测器自己报超速"
        #    （实测踩过：解析 0.194 rad/s 被差分报成 0.223）。
        #    这里按频率自适应加密：每个周期至少 400 点。
        f = max(1e-9, float(self.spec.frequency_hz))
        per_period = 400
        n_fast = int(min(400000, max(4000, self.duration * f * per_period)))
        ts_fast = np.linspace(0.0, self.duration, n_fast)
        Q_fast = np.zeros((n_fast, len(ARM_JOINTS)))
        V_fast = np.zeros_like(Q_fast)
        for i, t in enumerate(ts_fast):
            Q_fast[i], V_fast[i] = self.evaluate(float(t))
        ts = np.linspace(0.0, self.duration, samples)
        Q = np.zeros((samples, len(ARM_JOINTS)))
        V = np.zeros_like(Q)
        for i, t in enumerate(ts):
            Q[i], V[i] = self.evaluate(float(t))
        A = np.gradient(V, ts, axis=0)     # 加速度：对解析速度再差分（够用）
        # 位置/速度的极值取自密采网格
        Q_use, V_use = Q_fast, V_fast
        problems: List[str] = []
        for k, jn in enumerate(ARM_JOINTS):
            if Q_use[:, k].min() < self.lo[k] - 1e-9 or Q_use[:, k].max() > self.hi[k] + 1e-9:
                problems.append(f"{jn} 越限")
            if abs(self.spec.amplitudes[k]) < 1e-12:
                continue
            # 逐关节速度上限（肩/肘 0.2，腕 0.4）—— 用解析速度的密采峰值
            joint_vmax = float(np.abs(V_use[:, k]).max())
            if joint_vmax > self.v_cap[k] + 1e-6:
                problems.append(
                    f"{jn} 峰值速度 {joint_vmax:.4f} > 上限 "
                    f"{self.v_cap[k]:.3f} rad/s")
            if (self.spec.min_peak_speed > 0.0
                    and joint_vmax < self.spec.min_peak_speed - 1e-6):
                problems.append(
                    f"{jn} 峰值速度 {joint_vmax:.4f} < 下限 "
                    f"{self.spec.min_peak_speed:.3f} rad/s")
            # 覆盖率目标（交付轨迹为 0.80；目标为 0 表示不检查）
            if self.coverage_target[k] > 0.0:
                cov = ((Q_use[:, k].max() - Q_use[:, k].min())
                       / max(1e-12, self._span[k]))
                if cov < self.coverage_target[k] - 2e-3:
                    problems.append(
                        f"{jn} 行程覆盖率 {cov * 100:.1f}% < 目标 "
                        f"{self.coverage_target[k] * 100:.1f}%")
        # 全局兜底：任何关节都不应超过 0.4 rad/s（腕部上限为全机最大值）
        if np.abs(V_use).max() > 0.4 + 1e-6:
            problems.append(f"峰值速度 {np.abs(V_use).max():.3f} rad/s 过大（>0.4）")
        margin = np.array([
            math.degrees(min(Q_use[:, k].min() - self.lo[k],
                             self.hi[k] - Q_use[:, k].max()))
            for k in range(len(ARM_JOINTS))
        ])
        steady = ts_fast >= self.ramp
        margin_ss = np.array([
            math.degrees(min(Q_use[steady, k].min() - self.lo[k],
                             self.hi[k] - Q_use[steady, k].max()))
            for k in range(len(ARM_JOINTS))
        ])
        travel = Q_use.max(axis=0) - Q_use.min(axis=0)
        coverage = travel / np.maximum(1e-12, self._span)
        # 报告里的 v_peak/a_peak 用解析量（a_peak 仍来自差分，只作数量级参考）
        return TrajectoryCheck(
            name=self.spec.name,
            center=self.center,
            q_min=Q_use.min(axis=0),
            q_max=Q_use.max(axis=0),
            v_peak=np.abs(V_use).max(axis=0),
            a_peak=np.abs(A).max(axis=0),
            limit_margin_deg=margin,
            travel_deg=np.degrees(travel),
            limit_margin_steady_deg=margin_ss,
            span_deg=np.degrees(self._span),
            v_cap=self.v_cap.copy(),
            coverage=coverage,
            coverage_target=self.coverage_target.copy(),
            ok=not problems,
            problems=problems,
        )

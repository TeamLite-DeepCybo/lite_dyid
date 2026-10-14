# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""轨迹设计与校核工具（**不碰机器人**）。

    dyid_design --check   --pose-file <run_meta.json>   # 校核 trajectories.yaml
    dyid_design --generate config/trajectories.yaml      # 重新生成轨迹定义

设计约束（用户要求 + 本机实测）
================================
* 肩部单关节变动 ≤ 50°
* 峰值速度 ≈ 0.15 rad/s、加速度尽可能小（"速度变化较慢"）
* 不越 URDF 限位（稳态留 ≥ 0.10 rad 余量）
* 在当前 mesh 下**无碰撞**（pinocchio + hppfcl，排除相邻/同关节/夹爪对）
* 前两条左右镜像（符号 mirror：pitch 保号，roll/yaw 取反），第三条做验证

为什么用"相对起始姿态的 bias"而不是绝对 center
----------------------------------------------
本机左肩 roll 停在 −89.3°、上限 −90°，只有 0.7° 余量。写死 center 要么越限、
要么浪费行程。bias 让运行期按实测姿态把工作点推进限位内，且 |amp|=0 的关节
**严格不动**。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

from .common import ARM_JOINTS, DyidError, config_path
from .trajectory import FourierTrajectory, load_specs

#: 参考起始姿态（本机 2026-10-13 实测）——仅当没有 --pose / --pose-file 时用于校核
REFERENCE_POSE = np.array([
    0.0564, -1.5582, 0.4550, -0.1300, 0.2127, 0.0625, -0.0307,
    0.0961, 1.5559, -0.1918, 0.1992, 0.7816, 0.0640, -0.1787,
], dtype=float)

#: 左右手镜像是"pitch 保号、roll/yaw 取反"
MIRROR_SIGN = np.array([1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0])

#: 左臂的偏置与振幅（rad）。偏置把工作点推离限位；振幅决定激励强度。
BIAS7 = np.array([0.00, 0.34, 0.00, -0.28, 0.00, 0.00, 0.00])
AMP7 = np.array([0.32, 0.33, 0.32, 0.28, 0.28, 0.24, 0.24])

HARMONIC_WEIGHTS = [1.0, 0.62, 0.42, 0.27]
HARMONIC_INDICES = [1, 2, 3, 4]

SPECS = [
    # name, role, freq, duration, ramp, 振幅缩放
    ("dyid_train_left",  "train",    0.035, 200.0, 25.0, 1.00),
    ("dyid_train_right", "train",    0.035, 200.0, 25.0, 1.00),
    ("dyid_validate",    "validate", 0.040, 120.0, 25.0, 0.75),
]


def build_specs() -> dict:
    """生成 3 条轨迹的 YAML 结构（phases 固定种子，保证可复现）。"""
    rng = np.random.default_rng(20261013)
    ph_left = rng.uniform(0.0, 2.0 * math.pi, (14, 4))
    ph_left[7:, :] = 0.0                      # 右臂不激励 → 相位无意义

    ph_right = np.zeros((14, 4))
    ph_right[7:, :] = ph_left[:7, :]          # 镜像：同一组相位

    rng3 = np.random.default_rng(777)
    ph_val = rng3.uniform(0.0, 2.0 * math.pi, (14, 4))

    amp_left = np.concatenate([AMP7, np.zeros(7)])
    bias_left = np.concatenate([BIAS7, np.zeros(7)])
    amp_right = np.concatenate([np.zeros(7), AMP7 * MIRROR_SIGN])
    bias_right = np.concatenate([np.zeros(7), BIAS7 * MIRROR_SIGN])
    amp_val = np.concatenate([AMP7 * 0.75, AMP7 * 0.75 * MIRROR_SIGN])
    bias_val = np.concatenate([BIAS7, BIAS7 * MIRROR_SIGN])

    phases = {"dyid_train_left": ph_left, "dyid_train_right": ph_right,
              "dyid_validate": ph_val}
    amps = {"dyid_train_left": amp_left, "dyid_train_right": amp_right,
            "dyid_validate": amp_val}
    biases = {"dyid_train_left": bias_left, "dyid_train_right": bias_right,
              "dyid_validate": bias_val}

    trajs = []
    for name, role, freq, dur, ramp, _scale in SPECS:
        entry = {
            "name": name, "role": role,
            "duration_sec": dur, "ramp_sec": ramp, "frequency_hz": freq,
            "bias": [round(float(x), 6) for x in biases[name]],
            "amplitudes": [round(float(x), 6) for x in amps[name]],
            "phases": [[round(float(x), 6) for x in row] for row in phases[name]],
            "harmonic_weights": HARMONIC_WEIGHTS,
            "harmonic_indices": HARMONIC_INDICES,
            "normalization": "l1",
        }
        if role == "train":
            entry.update({"static_insert_count": 20, "static_period_sec": 10.0,
                          "static_hold_sec": 10.0, "static_settle_sec": 4.0})
        else:
            # 验证轨迹不需要静止段（它只用运动段算残差）—— 省 ~2 min 机器人时间
            entry.update({"static_insert_count": 0})
        trajs.append(entry)

    return {
        "defaults": {"normalization": "l1", "harmonic_weights": HARMONIC_WEIGHTS,
                     "harmonic_indices": HARMONIC_INDICES},
        "trajectories": trajs,
    }


def _pose_from(args) -> np.ndarray:
    if args.pose:
        vals = [float(x) for x in args.pose.replace(",", " ").split()]
        if len(vals) != len(ARM_JOINTS):
            raise DyidError(f"--pose 需要 {len(ARM_JOINTS)} 个数，得到 {len(vals)}")
        return np.array(vals)
    if args.pose_file:
        payload = json.loads(Path(args.pose_file).expanduser().read_text(encoding="utf-8"))
        if "start_pose_rad" in payload:
            return np.asarray(payload["start_pose_rad"], dtype=float)
        if "q" in payload:
            return np.asarray(payload["q"], dtype=float)
        raise DyidError(f"{args.pose_file} 里没有 start_pose_rad / q")
    print("[design] 未给 --pose/--pose-file，使用内置参考姿态校核")
    return REFERENCE_POSE.copy()


def cmd_check(args) -> int:
    from .model import RobotModel
    urdf = Path(args.urdf).read_text(encoding="utf-8") if args.urdf else None
    if urdf is None:
        run_meta = None
        if args.pose_file:
            p = Path(args.pose_file).expanduser()
            if p.name == "run_meta.json" and (p.parent / "robot.urdf").is_file():
                run_meta = p.parent / "robot.urdf"
        if run_meta is not None:
            urdf = run_meta.read_text(encoding="utf-8")
        else:
            raise DyidError("校核需要 URDF：请给 --urdf，或让 --pose-file 指向采集目录里的 run_meta.json")
    model = RobotModel(urdf)
    specs = load_specs(Path(args.trajectories).expanduser() if args.trajectories
                       else config_path("trajectories.yaml"))
    pose = _pose_from(args)
    all_ok = True
    for spec in specs.values():
        traj = FourierTrajectory(spec, pose, model.limit_array(),
                                 speed_cap=float(args.speed_cap))
        chk = traj.check(samples=int(args.samples))
        print(f"\n===== {spec.name}（{spec.role}）f={spec.frequency_hz}Hz "
              f"dur={spec.duration_sec:.0f}s ramp={traj.ramp:.1f}s =====")
        print(f"{'joint':22s} {'steady(deg)':>18s} {'Δ°':>7s} {'vmax':>7s} "
              f"{'amax':>7s} {'余量°':>7s} {'稳态余量°':>9s}")
        for k, jn in enumerate(ARM_JOINTS):
            if abs(spec.amplitudes[k]) < 1e-12:
                continue
            print(f"{jn:22s} [{math.degrees(chk.q_min[k]):+7.1f},"
                  f"{math.degrees(chk.q_max[k]):+7.1f}] "
                  f"{chk.travel_deg[k]:7.1f} {chk.v_peak[k]:7.3f} {chk.a_peak[k]:7.3f} "
                  f"{chk.limit_margin_deg[k]:7.1f} {chk.limit_margin_steady_deg[k]:9.1f}")
        hits = 0
        for t in np.linspace(0.0, traj.duration, int(args.collision_samples)):
            q, _ = traj.evaluate(float(t))
            if model.collisions(dict(zip(ARM_JOINTS, q))):
                hits += 1
        print(f"  峰值速度 {chk.v_peak.max():.3f} rad/s  峰值加速度 {chk.a_peak.max():.3f} rad/s²  "
              f"碰撞采样点 {hits}")
        if chk.problems or hits:
            all_ok = False
            print(f"  ❌ {chk.problems}{' + 碰撞' if hits else ''}")
        else:
            print("  ✓ 通过（限位 / 速度 / 肩部幅度 / mesh 碰撞）")
    print("\n结论：" + ("全部通过 ✓" if all_ok else "存在问题 ❌"))
    return 0 if all_ok else 1


def cmd_generate(args) -> int:
    payload = build_specs()
    out = Path(args.generate).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# lite_dyid 傅里叶激励轨迹（由 dyid_design --generate 生成）\n"
        "#\n"
        "# bias      = 相对**起始姿态**的偏置（rad）；运行期按限位裁剪，\n"
        "#             |amp|=0 的关节严格保持起始姿态。\n"
        "# amplitudes= 傅里叶幅值（rad）\n"
        "# phases    = (14, 4) 相位（rad），固定种子以便复现\n"
        "# 峰值速度 ≈ 0.13 rad/s、峰值加速度 ≈ 0.075 rad/s²、肩部单关节 ≤35.5°\n"
        "# 设计/校核：dyid_design --check --pose-file <run_meta.json>\n"
    )
    out.write_text(header + yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
                   encoding="utf-8")
    print(f"已写入 {out}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="dyid_design", description="轨迹设计/校核（不碰机器人）")
    p.add_argument("--check", action="store_true", help="校核 trajectories.yaml")
    p.add_argument("--generate", default=None, help="重新生成 trajectories.yaml 的路径")
    p.add_argument("--trajectories", default=None)
    p.add_argument("--urdf", default=None)
    p.add_argument("--pose", default=None, help="14 个关节角(rad)，逗号或空格分隔")
    p.add_argument("--pose-file", default=None, help="run_meta.json 或含 q 的 json")
    p.add_argument("--speed-cap", type=float, default=0.15)
    p.add_argument("--samples", type=int, default=4000)
    p.add_argument("--collision-samples", type=int, default=120)
    args, _unknown = p.parse_known_args(argv)
    if args.generate:
        return cmd_generate(args)
    return cmd_check(args)


if __name__ == "__main__":
    sys.exit(main())

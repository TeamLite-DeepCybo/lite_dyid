# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""交付轨迹的生成与离线校核。

默认配置只保留一条双臂同步训练轨迹：

* 左右臂同时运动，运动时间 300 s；
* 每个关节覆盖 URDF 有效区间的 80%；
* 1～4 次谐波幅度依次减小，低频负责大范围构型覆盖，高频只补充动态信息；
* 肩/肘峰值速度不超过 0.2 rad/s，腕部不超过 0.4 rad/s；
* 运行前用现场 URDF 再做限位、速度、覆盖率与 mesh 碰撞检查。

``dyid_design`` 只做离线计算，不发布机器人命令。
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

from .common import (ARM_JOINTS, MIRROR_SIGN, DyidError, config_path,
                     velocity_caps)
from .limits_table import limits_array, midpoint
from .trajectory import FourierTrajectory, load_specs


TRAJECTORY_NAME = "dyid_train_bimanual"
DURATION_SEC = 300.0
RAMP_SEC = 60.0
BASE_FREQUENCY_HZ = 0.011
MIN_COVERAGE = 0.80
MIN_PEAK_SPEED = 0.05
APPROACH_SPEED_CAP = 0.10

# 低频大幅、高频小幅。归一化后整条曲线仍精确覆盖 80%，这些权重只决定
# 频谱能量如何分配：一阶占绝对主导，四阶只保留少量动态激励。
HARMONIC_WEIGHTS = [1.0, 0.35, 0.15, 0.05]
HARMONIC_INDICES = [1, 2, 3, 4]

# 固定相位来自 80% 覆盖率下的离线相位搜索。搜索目标不是缩幅，而是在保持每个
# 关节完整 80% 行程的前提下错开多关节极值。展开后的 arms URDF 上，一个周期
# 6001 点 mesh 检查无碰撞；运行期仍会结合真实起始姿态重新检查整条 300 s 轨迹。
PHASES_LEFT = np.array([
    [0.766925413, 4.574369257, 2.745677666, 4.078077080],
    [3.993557108, 5.318963076, 3.917644155, 4.053431310],
    [6.251277049, 3.691402811, 1.910702988, 5.031623590],
    [5.446989931, 2.586383867, 2.486554870, 4.041143292],
    [3.503903245, 3.254639750, 2.047977393, 3.234328940],
    [0.860482940, 1.537667005, 1.947057961, 5.959546881],
    [0.918325294, 3.852925657, 5.938531865, 1.355114821],
], dtype=float)


def _delivery_trajectory() -> dict:
    lo, hi = limits_array()
    half_travel = 0.5 * MIN_COVERAGE * (hi - lo)
    # 右臂使用同一相位，并用镜像符号反转 roll/yaw。两个手臂因此在同一 300 s
    # 窗口里同步采样，原来左右分开的两条训练轨迹不再串行执行。
    amplitudes = np.concatenate([
        half_travel[:7],
        half_travel[7:] * np.asarray(MIRROR_SIGN, dtype=float),
    ])
    phases = np.vstack([PHASES_LEFT, PHASES_LEFT])
    return {
        "name": TRAJECTORY_NAME,
        "role": "train",
        "duration_sec": DURATION_SEC,
        "ramp_sec": RAMP_SEC,
        "frequency_hz": BASE_FREQUENCY_HZ,
        "center_mode": "midpoint",
        "approach_speed_cap": APPROACH_SPEED_CAP,
        "bias": [0.0] * len(ARM_JOINTS),
        "amplitudes": [round(float(x), 9) for x in amplitudes],
        "phases": [[round(float(x), 9) for x in row] for row in phases],
        "harmonic_weights": HARMONIC_WEIGHTS,
        "harmonic_indices": HARMONIC_INDICES,
        "normalization": "l1",
        "peak_normalize": True,
        "min_coverage": MIN_COVERAGE,
        "min_peak_speed": MIN_PEAK_SPEED,
        # 静止段插在运动时间轴上；运动时间仍是完整 300 s。15×6 s 的保持用于
        # 重力回归，段内最后 3 s 取稳态平均。
        "static_insert_count": 15,
        "static_period_sec": 20.0,
        "static_hold_sec": 6.0,
        "static_settle_sec": 3.0,
    }


def build_specs() -> dict:
    """返回唯一的交付轨迹配置，结果固定且可复现。"""
    return {
        "defaults": {
            "normalization": "l1",
            "harmonic_weights": HARMONIC_WEIGHTS,
            "harmonic_indices": HARMONIC_INDICES,
            "center_mode": "midpoint",
            "approach_speed_cap": APPROACH_SPEED_CAP,
            "peak_normalize": True,
            "min_coverage": MIN_COVERAGE,
            "min_peak_speed": MIN_PEAK_SPEED,
        },
        "trajectories": [_delivery_trajectory()],
    }


def build_large_specs(collide=None) -> dict:
    """兼容旧调用名；大幅双臂轨迹现在就是唯一交付档。"""
    payload = build_specs()
    if collide is not None:
        spec = _spec_from_payload(payload)
        lo, hi = limits_array()
        traj = FourierTrajectory(spec, midpoint(), (lo, hi),
                                 v_cap=np.asarray(velocity_caps(), dtype=float))
        for t in np.linspace(0.0, traj.duration, 2401):
            q, _ = traj.evaluate(float(t))
            if collide(q):
                raise DyidError(
                    f"{TRAJECTORY_NAME} 在 t={t:.3f}s 碰撞；80% 幅度不会自动缩小，"
                    "请重新设计相位")
    return payload


def _spec_from_payload(payload: dict):
    """通过与运行期相同的 YAML 加载器构造 TrajectorySpec。"""
    import tempfile
    # 该帮助函数只供兼容的 build_large_specs(collision_cb) 使用。使用命名临时
    # 文件可以复用 load_specs 的全部字段校验，关闭后立即删除。
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        f.flush()
        return load_specs(Path(f.name))[TRAJECTORY_NAME]


def _pose_from(args) -> np.ndarray:
    if args.pose:
        vals = [float(x) for x in args.pose.replace(",", " ").split()]
        if len(vals) != len(ARM_JOINTS):
            raise DyidError(f"--pose 需要 {len(ARM_JOINTS)} 个数，得到 {len(vals)}")
        return np.asarray(vals, dtype=float)
    if args.pose_file:
        payload = json.loads(Path(args.pose_file).expanduser().read_text(encoding="utf-8"))
        for key in ("start_pose_rad", "q"):
            if key in payload:
                q = np.asarray(payload[key], dtype=float)
                if q.shape != (len(ARM_JOINTS),):
                    raise DyidError(f"{args.pose_file} 的 {key} 形状不是 14")
                return q
        raise DyidError(f"{args.pose_file} 里没有 start_pose_rad / q")
    return midpoint()


def _urdf_from_args(args) -> str:
    if args.urdf:
        return Path(args.urdf).expanduser().read_text(encoding="utf-8")
    if args.pose_file:
        pose_path = Path(args.pose_file).expanduser()
        candidate = pose_path.parent / "robot.urdf"
        if pose_path.name == "run_meta.json" and candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    raise DyidError("校核需要 --urdf，或让 --pose-file 指向采集目录里的 run_meta.json")


def cmd_check(args) -> int:
    from .model import RobotModel

    model = RobotModel(_urdf_from_args(args))
    path = Path(args.trajectories).expanduser() if args.trajectories else config_path("trajectories.yaml")
    specs = load_specs(path)
    pose = _pose_from(args)
    caps = np.asarray(velocity_caps(), dtype=float)
    all_ok = True

    for spec in specs.values():
        traj = FourierTrajectory(spec, pose, model.limit_array(),
                                 speed_cap=float(args.speed_cap), v_cap=caps)
        check = traj.check(samples=int(args.samples))
        print(f"\n===== {spec.name} ({spec.role}) =====")
        print(f"运动 {traj.duration:.0f}s，基频 {spec.frequency_hz:.4f}Hz，"
              f"1~4 次谐波 {list(spec.harmonic_weights)}，进入段 {traj.ramp:.1f}s")
        print(f"{'joint':22s} {'coverage':>9s} {'v_peak':>9s} {'v_cap':>8s} "
              f"{'q_min(deg)':>11s} {'q_max(deg)':>11s}")
        for k, joint in enumerate(ARM_JOINTS):
            if abs(spec.amplitudes[k]) < 1e-12:
                continue
            print(f"{joint:22s} {check.coverage[k] * 100:8.2f}% "
                  f"{check.v_peak[k]:9.4f} {caps[k]:8.3f} "
                  f"{math.degrees(check.q_min[k]):11.2f} "
                  f"{math.degrees(check.q_max[k]):11.2f}")

        hits = []
        for t in np.linspace(0.0, traj.duration, int(args.collision_samples)):
            q, _ = traj.evaluate(float(t))
            collisions = model.collisions(dict(zip(ARM_JOINTS, q)))
            if collisions:
                hits.append((float(t), collisions[0]))
                break
        if traj.clamp_count:
            check.problems.append(f"运行期限位钳位 {traj.clamp_count} 次")
        if hits:
            t, hit = hits[0]
            check.problems.append(
                f"t={t:.3f}s mesh 碰撞 {hit.first} ↔ {hit.second} depth={hit.depth:.6f}m")
        if check.problems:
            all_ok = False
            print("  FAIL: " + "；".join(check.problems))
        else:
            print("  PASS: 80% 覆盖、速度上下限、关节限位和 mesh 碰撞检查全部通过")

    return 0 if all_ok else 1


def cmd_generate(args) -> int:
    out = Path(args.generate).expanduser()
    payload = build_specs()
    header = (
        "# lite_dyid 交付轨迹：双臂同步、300 s、逐关节至少 80% 行程\n"
        "# 低频大幅、高频小幅；默认配置即正式大幅采集版本。\n"
        "# 生成：dyid_design --generate <path>\n"
        "# 校核：dyid_design --check --urdf <expanded.urdf>\n"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(header + yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
                   encoding="utf-8")
    print(f"已写入 {out}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="dyid_design", description="交付轨迹生成/校核（不碰机器人）")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--generate", default=None)
    parser.add_argument("--profile", default="delivery",
                        choices=["delivery", "standard", "large"],
                        help="兼容参数；三个名称都指向同一交付轨迹")
    parser.add_argument("--trajectories", default=None)
    parser.add_argument("--urdf", default=None)
    parser.add_argument("--pose", default=None)
    parser.add_argument("--pose-file", default=None)
    parser.add_argument("--speed-cap", type=float, default=0.15)
    parser.add_argument("--samples", type=int, default=12000)
    parser.add_argument("--collision-samples", type=int, default=6001)
    # 保留旧命令行的参数名，避免脚本升级后直接解析失败；覆盖率由 YAML 中的
    # min_coverage=0.80 固定，不允许从命令行悄悄降低。
    parser.add_argument("--min-coverage", type=float, default=None,
                        help="已弃用；交付轨迹固定为 0.80")
    args, _unknown = parser.parse_known_args(argv)
    if args.min_coverage is not None and abs(args.min_coverage - MIN_COVERAGE) > 1e-9:
        raise DyidError("交付轨迹覆盖率固定为 0.80，不能用 --min-coverage 降低")
    if args.generate:
        return cmd_generate(args)
    return cmd_check(args)


if __name__ == "__main__":
    sys.exit(main())

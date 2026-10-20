# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""MIT 中频跟踪验证：相同两频轨迹下比较无/有辨识前馈。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .common import (ARM_JOINTS, MIRROR_SIGN, PROTOCOL_MIT, DyidError,
                     default_output_dir)
from .csp_client import BridgeClient, ProtocolRestartRequired
from .dataset import save_csv
from .identify import load_params
from .motion import RateLoop, smoothstep, smoothstep_dot
from .report import (RATED_TORQUE, plot_tracking_comparison, plot_tracking_trial,
                     tracking_error_stats, write_stats_csv)
from .trajectory import FourierTrajectory, TrajectorySpec

KP = 15.0
KD = 3.0
RATE_HZ = 200.0
BASE_FREQUENCY_HZ = 0.025
DURATION_SEC = 120.0
RAMP_SEC = 30.0
AMPLITUDE_RATIO = 0.15  # 半行程；峰峰值为整个关节行程的 30%
RETURN_SPEED = 0.10
SETTLE_SEC = 2.0
REQUIRED_ARTIFACTS = (
    "validation_meta.json",
    "tracking_without_identified_ff.csv",
    "tracking_without_identified_ff.png",
    "tracking_with_identified_ff.csv",
    "tracking_with_identified_ff.png",
    "tracking_error_stats.csv",
    "tracking_error_comparison.png",
    "summary.json",
)

# 两个频率的固定相位。右臂复用左臂相位，振幅按镜像符号翻转。
_LEFT_PHASES = np.asarray([
    [0.766925413, 4.574369257],
    [3.993557108, 5.318963076],
    [6.251277049, 3.691402811],
    [5.446989931, 2.586383867],
    [3.503903245, 3.254639750],
    [0.860482940, 1.537667005],
    [0.918325294, 3.852925657],
], dtype=float)


def find_latest_run(run_dir: Optional[Path] = None) -> Path:
    """定位最新含 ``params.json`` 与 ``robot.urdf`` 的辨识结果目录。"""
    if run_dir is not None:
        candidate = Path(run_dir).expanduser().resolve()
        if not (candidate / "params.json").is_file():
            raise DyidError(f"辨识结果缺少 params.json：{candidate}")
        if not (candidate / "robot.urdf").is_file():
            raise DyidError(f"辨识结果缺少 robot.urdf：{candidate}")
        return candidate

    roots = [default_output_dir(), Path("/tmp/dyid_runs")]
    found: List[Path] = []
    seen = set()
    for root in roots:
        root = root.expanduser()
        if root in seen or not root.is_dir():
            continue
        seen.add(root)
        for params_path in root.glob("run_*/params.json"):
            candidate = params_path.parent
            if (candidate / "robot.urdf").is_file():
                found.append(candidate)
    if not found:
        roots_text = "、".join(str(p.expanduser()) for p in roots)
        raise DyidError(f"找不到最新辨识结果（搜索过 {roots_text}）；请传 --run-dir")
    return max(found, key=lambda p: (p / "params.json").stat().st_mtime)


def choose_output_dir(run_dir: Path, requested: Optional[Path] = None,
                      *, timestamp: Optional[str] = None) -> Path:
    """选择持久输出目录；不再默认把验证结果跟随 ``/tmp`` 辨识目录存放。"""
    if requested is not None:
        return Path(requested).expanduser().resolve()
    stamp = timestamp or datetime.now().strftime("tracking_validation_%Y%m%d_%H%M%S")
    root = Path.home() / ".ros" / "lite_dyid" / "tracking_validations"
    return (root / Path(run_dir).name / stamp).resolve()


def verify_output_artifacts(out_dir: Path) -> dict:
    """完成前逐项确认产物非空，并写 ``artifacts.json`` 清单。"""
    out_dir = Path(out_dir).resolve()
    missing = []
    files = []
    for name in REQUIRED_ARTIFACTS:
        path = out_dir / name
        if not path.is_file() or path.stat().st_size <= 0:
            missing.append(name)
            continue
        files.append({"name": name, "bytes": path.stat().st_size,
                      "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if missing:
        raise DyidError(f"输出完整性检查失败，缺少或为空：{missing}；目录：{out_dir}")
    manifest = {
        "verified": datetime.now().isoformat(timespec="seconds"),
        "output_dir": str(out_dir),
        "file_count": len(files),
        "total_bytes": int(sum(item["bytes"] for item in files)),
        "files": files,
    }
    temp = out_dir / ".artifacts.json.tmp"
    temp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    temp.replace(out_dir / "artifacts.json")
    return manifest


def _sha256(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _urdf_semantic_digest(urdf_text: str, *, ignore_control_mode: bool) -> str:
    """计算 URDF XML 的语义摘要，可只忽略驱动启动协议字段。

    辨识在 CSP 下采集，而跟踪验证必须在 MIT 下运行；xacro 因此必然把两个
    ``ros2_control/hardware/param[@name='control_mode']`` 从 ``csp`` 改为
    ``mit``。该字段只选择底层通信协议，不改变关节、限位、传动、惯性或 mesh，
    不能作为拒绝使用同一辨识模型的理由。
    """
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError as exc:
        raise DyidError(f"robot_description 不是合法 XML：{exc}") from exc

    def encode(element: ET.Element, path: Tuple[str, ...]):
        text = " ".join((element.text or "").split())
        here = path + (element.tag,)
        if (ignore_control_mode and element.tag == "param"
                and element.attrib.get("name") == "control_mode"
                and len(path) >= 2 and path[-2:] == ("ros2_control", "hardware")):
            text = "<runtime-protocol>"
        return (element.tag, tuple(sorted(element.attrib.items())), text,
                tuple(encode(child, here) for child in list(element)))

    payload = repr(encode(root, tuple())).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def check_live_urdf_compatible(reference: str, live: str) -> str:
    """验证 live URDF 是否只含格式差异或预期的 CSP→MIT 协议差异。"""
    if _urdf_semantic_digest(reference, ignore_control_mode=False) == \
            _urdf_semantic_digest(live, ignore_control_mode=False):
        return "当前 URDF 与辨识 URDF 语义一致"
    if _urdf_semantic_digest(reference, ignore_control_mode=True) == \
            _urdf_semantic_digest(live, ignore_control_mode=True):
        return "当前 URDF 仅 control_mode 由 CSP 变为 MIT；机械模型一致"
    raise DyidError(
        "当前 /robot_description 与辨识时 URDF 存在机械模型差异"
        "（关节/限位/传动/惯性/几何等），拒绝套用旧参数；"
        "允许的差异仅为 XML 格式和 ros2_control.hardware.control_mode 的 CSP→MIT 切换")


def validate_params(params: dict, urdf_text: str) -> None:
    joints = params.get("robot", {}).get("joints", params.get("joints"))
    if list(joints or []) != list(ARM_JOINTS):
        raise DyidError("params.json 的关节顺序与当前 14 关节规范不一致")
    expected = params.get("robot", {}).get("urdf_sha256")
    if expected and expected != _sha256(urdf_text):
        raise DyidError("辨识目录内 robot.urdf 与 params.json 的 SHA256 不一致")
    for section, key, size in (("gravity", "params", None),
                               ("friction", "c1_pos", 14),
                               ("friction", "c1_neg", 14),
                               ("friction", "c2", 14)):
        values = np.asarray(params.get(section, {}).get(key, []), dtype=float)
        if values.ndim != 1 or not values.size or not np.all(np.isfinite(values)):
            raise DyidError(f"params.json 的 {section}.{key} 无效")
        if size is not None and values.size != size:
            raise DyidError(f"params.json 的 {section}.{key} 长度 {values.size} != {size}")


def build_validation_trajectory(model, entry: np.ndarray, *, duration: float = DURATION_SEC,
                                base_frequency: float = BASE_FREQUENCY_HZ,
                                amplitude_ratio: float = AMPLITUDE_RATIO) -> FourierTrajectory:
    """生成 14 关节同步、频率为 ``f`` 与 ``2f`` 的中频轨迹。"""
    if duration <= RAMP_SEC:
        raise DyidError(f"验证时长 {duration}s 必须大于进入段 {RAMP_SEC}s")
    if not (0.0 < amplitude_ratio < 0.4):
        raise DyidError("amplitude_ratio 必须在 (0, 0.4) 内")
    lo, hi = model.limit_array()
    left_amp = amplitude_ratio * (hi[:7] - lo[:7])
    amplitudes = np.r_[left_amp, left_amp * np.asarray(MIRROR_SIGN)]
    # 两侧 URDF 行程若不完全相同，以各自实际行程为准，同时保留镜像方向。
    amplitudes[7:] = amplitude_ratio * (hi[7:] - lo[7:]) * np.asarray(MIRROR_SIGN)
    spec = TrajectorySpec(
        name="mit_tracking_two_frequency",
        role="validate",
        duration_sec=float(duration),
        ramp_sec=RAMP_SEC,
        frequency_hz=float(base_frequency),
        bias=np.zeros(14),
        amplitudes=amplitudes,
        phases=np.vstack([_LEFT_PHASES, _LEFT_PHASES]),
        harmonic_weights=np.asarray([1.0, 0.35]),
        harmonic_indices=np.asarray([1.0, 2.0]),
        normalization="l1",
        static_insert_count=0,
        center_mode="midpoint",
        approach_speed_cap=RETURN_SPEED,
        peak_normalize=True,
        min_coverage=0.0,
        min_peak_speed=0.0,
    )
    return FourierTrajectory(spec, np.asarray(entry, dtype=float), (lo, hi),
                             speed_cap=RETURN_SPEED)


def sample_reference(traj: FourierTrajectory, rate_hz: float = RATE_HZ
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    count = int(math.floor(traj.duration * rate_hz)) + 1
    t = np.arange(count, dtype=float) / rate_hz
    t[-1] = min(t[-1], traj.duration)
    q = np.zeros((count, len(ARM_JOINTS)), dtype=float)
    v = np.zeros_like(q)
    for i, stamp in enumerate(t):
        q[i], v[i] = traj.evaluate(float(stamp))
    return t, q, v


class IdentifiedFeedforward:
    """最新辨识结果的重力 + 方向相关摩擦前馈。"""

    def __init__(self, model, params: dict) -> None:
        self.gravity = model.fast_gravity_evaluator(
            np.asarray(params["gravity"]["params"], dtype=float))
        self.c1_pos = np.asarray(params["friction"]["c1_pos"], dtype=float)
        self.c1_neg = np.asarray(params["friction"]["c1_neg"], dtype=float)
        self.c2 = np.asarray(params["friction"]["c2"], dtype=float)
        self.model = model

    def __call__(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        gravity = self.gravity(self.model.q_from_list(q))
        coulomb = np.where(v > 0.0, self.c1_pos, np.where(v < 0.0, self.c1_neg, 0.0))
        return gravity + coulomb + self.c2 * v


def precompute_feedforward(predictor: IdentifiedFeedforward,
                           q: np.ndarray, v: np.ndarray) -> np.ndarray:
    tau = np.vstack([predictor(qi, vi) for qi, vi in zip(q, v)])
    limits = np.asarray([RATED_TORQUE[j] for j in ARM_JOINTS], dtype=float)
    peak = np.max(np.abs(tau), axis=0)
    bad = np.flatnonzero(peak > limits + 1e-9)
    if bad.size:
        detail = ", ".join(f"{ARM_JOINTS[i]}={peak[i]:.2f}>{limits[i]:.2f}N·m"
                           for i in bad)
        raise DyidError(f"辨识前馈超过关节额定力矩，拒绝执行：{detail}")
    return tau


def collision_check(model, q: np.ndarray, *, samples: int = 2401) -> None:
    indices = np.linspace(0, q.shape[0] - 1, min(samples, q.shape[0]), dtype=int)
    for idx in np.unique(indices):
        hits = model.collisions(dict(zip(ARM_JOINTS, q[idx])))
        if hits:
            hit = hits[0]
            raise DyidError(
                f"验证轨迹在 t 索引 {idx} 发生 mesh 碰撞："
                f"{hit.first} ↔ {hit.second}（{hit.depth:.4f}m）")


def _trial_header() -> List[str]:
    fields = ["sample_t", "wall_t"]
    for prefix in ("q_des", "q_actual", "error", "v_des", "v_actual", "tau_ff"):
        fields.extend(f"{prefix}/{joint}" for joint in ARM_JOINTS)
    return fields


def run_trial(client: BridgeClient, condition: str, t_ref: np.ndarray,
              q_ref: np.ndarray, v_ref: np.ndarray, tau_ff: np.ndarray,
              *, rate_hz: float = RATE_HZ) -> Dict[str, np.ndarray]:
    """实时执行一组条件并返回同拍期望/实测数据。"""
    n = t_ref.size
    wall = np.zeros(n)
    actual_q = np.zeros_like(q_ref)
    actual_v = np.zeros_like(v_ref)
    loop = RateLoop(rate_hz)
    loop.start()
    start = time.monotonic()
    for i in range(n):
        if i % max(1, int(rate_hz / 5.0)) == 0:
            client.assert_healthy()
        client.publish_mit(q_ref[i], v_ref[i], tau_ff[i], kp=KP, kd=KD)
        snap = client.snapshot()
        if snap.age() > 0.25 or not np.all(np.isfinite(snap.position)):
            raise DyidError(f"{condition} 第 {i} 拍关节状态陈旧/无效（age={snap.age():.3f}s）")
        wall[i] = time.monotonic() - start
        actual_q[i] = snap.position
        actual_v[i] = snap.velocity
        if i + 1 < n:
            loop.tick()
    error = actual_q - q_ref
    return {"sample_t": t_ref.copy(), "wall_t": wall, "q_des": q_ref.copy(),
            "q_actual": actual_q, "error": error, "v_des": v_ref.copy(),
            "v_actual": actual_v, "tau_ff": tau_ff.copy(),
            "rate_actual": np.asarray([(n - 1) / max(wall[-1], 1e-9)]),
            "lag_max": np.asarray([loop.lag_max])}


def save_trial(out_dir: Path, condition: str, trial: Dict[str, np.ndarray]) -> List[dict]:
    matrix = np.column_stack([
        trial["sample_t"], trial["wall_t"], trial["q_des"], trial["q_actual"],
        trial["error"], trial["v_des"], trial["v_actual"], trial["tau_ff"],
    ])
    save_csv(out_dir / f"tracking_{condition}.csv", _trial_header(), matrix,
             traj_name=condition, fmt="%.9g")
    rows = tracking_error_stats(trial["error"], condition=condition)
    plot_tracking_trial(out_dir, condition, trial["sample_t"],
                        trial["q_des"], trial["q_actual"])
    return rows


def settle_at_entry(client: BridgeClient, entry: np.ndarray, *, rate_hz: float = RATE_HZ,
                    duration: float = SETTLE_SEC) -> None:
    zeros = np.zeros(len(ARM_JOINTS))
    loop = RateLoop(rate_hz)
    loop.start()
    count = max(1, int(duration * rate_hz))
    for _ in range(count):
        client.publish_mit(entry, zeros, zeros, kp=KP, kd=KD)
        loop.tick()


def return_to_entry(client: BridgeClient, entry: np.ndarray,
                    predictor: IdentifiedFeedforward, model, *,
                    rate_hz: float = RATE_HZ) -> None:
    """以限速 smoothstep 从当前实测位置返回两组共同的起点。"""
    start = client.snapshot().position
    delta = np.asarray(entry) - start
    duration = max(1.0, 1.5 * float(np.max(np.abs(delta))) / RETURN_SPEED)
    count = int(math.ceil(duration * rate_hz)) + 1
    elapsed = np.minimum(duration, np.arange(count, dtype=float) / rate_hz)
    x = elapsed / duration
    q_return = np.vstack([start + delta * smoothstep(value) for value in x])
    v_return = np.vstack([delta * smoothstep_dot(value, duration) for value in x])
    # 在开始回位前校核整条路径和前馈；实时循环只查表，保持 200Hz。
    for idx in np.linspace(0, count - 1, min(301, count), dtype=int):
        hits = model.collisions(dict(zip(ARM_JOINTS, q_return[idx])))
        if hits:
            raise DyidError(f"回起点路径碰撞：{hits[0].first} ↔ {hits[0].second}")
    tau_return = precompute_feedforward(predictor, q_return, v_return)
    loop = RateLoop(rate_hz)
    loop.start()
    for i in range(count):
        client.publish_mit(q_return[i], v_return[i], tau_return[i], kp=KP, kd=KD)
        if i + 1 < count:
            loop.tick()
    settle_at_entry(client, entry, rate_hz=rate_hz)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="dyid_tracking_validate",
        description="最新辨识参数的 MIT 两频中频轨迹跟踪误差 A/B 验证",
    )
    parser.add_argument("--run-dir", default=None,
                        help="辨识结果目录；默认自动选择最新 params.json")
    parser.add_argument("--output-dir", default=None,
                        help="输出目录；默认写到 ~/.ros/lite_dyid/tracking_validations/")
    parser.add_argument("--source", default="dyid", help="lite_bridge 已注册的数据源名")
    parser.add_argument("--duration", type=float, default=DURATION_SEC,
                        help=f"每组轨迹时长，默认 {DURATION_SEC:g}s")
    parser.add_argument("--rate-hz", type=float, default=RATE_HZ,
                        help=f"下发与采样频率，默认 {RATE_HZ:g}Hz，上限 200Hz")
    parser.add_argument("--base-frequency", type=float, default=BASE_FREQUENCY_HZ,
                        help="基频 Hz；另一频率固定为其 2 倍")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args, _unknown = build_parser().parse_known_args(argv)
    if not (0.0 < args.rate_hz <= 200.0):
        raise DyidError(f"--rate-hz 必须在 (0,200]，得到 {args.rate_hz}")
    if args.base_frequency <= 0.0:
        raise DyidError("--base-frequency 必须为正")

    run_dir = find_latest_run(Path(args.run_dir) if args.run_dir else None)
    params = load_params(run_dir / "params.json")
    run_urdf = (run_dir / "robot.urdf").read_text(encoding="utf-8")
    validate_params(params, run_urdf)
    print(f"[验证] 使用最新辨识结果：{run_dir}")
    print(f"[验证] 模型：{params.get('source', {}).get('model_desc', params.get('source', {}).get('model', '?'))}")
    print(f"[验证] Kp={KP:g}, Kd={KD:g}；频率 {args.base_frequency:g} / "
          f"{2.0 * args.base_frequency:g} Hz；每组 {args.duration:g}s @ {args.rate_hz:g}Hz")

    client = BridgeClient(node_name="lite_dyid_tracking_validate", source=args.source,
                          command_timeout_ms=300.0)
    entered_tracking = False
    entry: Optional[np.ndarray] = None
    try:
        client.wait_joint_states(timeout=25.0)
        live_urdf = client.wait_urdf(timeout=20.0)
        print(f"[预检] {check_live_urdf_compatible(run_urdf, live_urdf)}")
        client.assert_healthy()

        # 若仍在 CSP，此调用只执行 CSP_MONITOR 回退并要求重启，不会发布 MIT。
        status = client.get_status()
        if not getattr(status, "ok", False):
            raise DyidError(f"读取桥状态失败：{getattr(status, 'message', '')}")
        if getattr(status, "protocol", None) != PROTOCOL_MIT:
            client.ensure_mit_ready(kp=KP, kd=KD)

        from .model import RobotModel
        model = RobotModel(run_urdf)
        entry = client.snapshot().position.copy()
        traj = build_validation_trajectory(
            model, entry, duration=float(args.duration),
            base_frequency=float(args.base_frequency))
        check = traj.check(samples=2401)
        if not check.ok or traj.clamp_count:
            raise DyidError("验证轨迹校核失败：" + "；".join(check.problems))
        t_ref, q_ref, v_ref = sample_reference(traj, float(args.rate_hz))
        print(f"[预检] 轨迹峰值速度 {np.max(np.abs(v_ref)):.3f}rad/s；"
              f"进入段 {traj.ramp:.1f}s；开始 mesh 碰撞检查")
        collision_check(model, q_ref)
        print("[预检] 限位、速度、mesh 碰撞检查通过")

        predictor = IdentifiedFeedforward(model, params)
        tau_identified = precompute_feedforward(predictor, q_ref, v_ref)
        tau_none = np.zeros_like(tau_identified)
        peak = np.max(np.abs(tau_identified), axis=0)
        print(f"[预检] 辨识前馈峰值 {peak.max():.3f}N·m，逐关节均未超过额定力矩")

        out_dir = choose_output_dir(
            run_dir, Path(args.output_dir) if args.output_dir else None)
        out_dir.mkdir(parents=True, exist_ok=False)
        (out_dir / "validation_meta.json").write_text(json.dumps({
            "created": datetime.now().isoformat(timespec="seconds"),
            "identification_run": str(run_dir),
            "params_sha256": hashlib.sha256((run_dir / "params.json").read_bytes()).hexdigest(),
            "model": params.get("source", {}).get("model"),
            "kp": KP, "kd": KD, "rate_hz": float(args.rate_hz),
            "duration_sec_each": float(args.duration),
            "frequencies_hz": [float(args.base_frequency), 2.0 * float(args.base_frequency)],
            "harmonic_weights": [1.0, 0.35], "amplitude_ratio_half_span": AMPLITUDE_RATIO,
            "order": ["without_identified_ff", "with_identified_ff"],
            "joints": list(ARM_JOINTS), "entry_position_rad": entry.tolist(),
            "feedforward_peak_nm": peak.tolist(),
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[验证] 输出目录：{out_dir}")

        notes = client.ensure_mit_ready(kp=KP, kd=KD)
        entered_tracking = True
        print(f"[模式] {notes}")

        all_stats: List[dict] = []
        by_condition: Dict[str, List[dict]] = {}
        for condition, tau in (("without_identified_ff", tau_none),
                               ("with_identified_ff", tau_identified)):
            print(f"\n[验证] 开始 {condition}")
            settle_at_entry(client, entry, rate_hz=float(args.rate_hz))
            trial = run_trial(client, condition, t_ref, q_ref, v_ref, tau,
                              rate_hz=float(args.rate_hz))
            rows = save_trial(out_dir, condition, trial)
            by_condition[condition] = rows
            all_stats.extend(rows)
            mean_rmse = float(np.mean([r["rmse_deg"] for r in rows]))
            print(f"[验证] {condition} 完成：实际 {trial['rate_actual'][0]:.1f}Hz，"
                  f"14 关节平均 RMSE={mean_rmse:.3f}°")
            return_to_entry(client, entry, predictor, model, rate_hz=float(args.rate_hz))

        write_stats_csv(out_dir / "tracking_error_stats.csv", all_stats)
        plot_tracking_comparison(out_dir, by_condition)
        baseline = np.asarray([r["rmse_deg"] for r in by_condition["without_identified_ff"]])
        identified = np.asarray([r["rmse_deg"] for r in by_condition["with_identified_ff"]])
        improvement = 100.0 * (baseline - identified) / np.maximum(baseline, 1e-12)
        (out_dir / "summary.json").write_text(json.dumps({
            "mean_rmse_deg_without_identified_ff": float(np.mean(baseline)),
            "mean_rmse_deg_with_identified_ff": float(np.mean(identified)),
            "mean_rmse_improvement_percent": float(np.mean(improvement)),
            "per_joint_rmse_improvement_percent": dict(zip(ARM_JOINTS, improvement.tolist())),
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        manifest = verify_output_artifacts(out_dir)
        size_mib = manifest["total_bytes"] / (1024.0 * 1024.0)
        print(f"\n[完成] 已验证 {manifest['file_count']} 个必需产物，共 {size_mib:.1f} MiB")
        print(f"[完成] 持久输出目录：{manifest['output_dir']}")
        print(f"[完成] 产物清单：{Path(manifest['output_dir']) / 'artifacts.json'}")
        print(f"[完成] 查看命令：ls -lah '{manifest['output_dir']}'")
        return 0
    except ProtocolRestartRequired as exc:
        print(f"\n[需要重启 MIT bringup]\n{exc}", file=sys.stderr)
        return 3
    except (DyidError, KeyboardInterrupt) as exc:
        print(f"\n[验证中止] {exc}", file=sys.stderr)
        return 2
    finally:
        if entered_tracking:
            held = entry if entry is not None else client.snapshot().position
            print("[收尾] " + client.finish_mit_session(
                entered_tracking=True, held_position=held, kp=KP, kd=KD))
        client.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

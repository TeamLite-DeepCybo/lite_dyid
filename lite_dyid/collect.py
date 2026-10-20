# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""主流程：预检 → 确认 CSP → 小幅运动验证 → 双臂同步 300 s 采集
→ 自动解算（重力 + 摩擦）。

设计取舍
========
* **不做 dry-run**：本包的目的就是在真机上采数；所有"能不能动"的判断都放在
  预检与小幅验证里（每关节 ≤ ``--small-delta``°，默认 8.6°，速度 0.1 rad/s）。
* **中止时不切 mode5**：CSP 没有"松劲"，切到 5 会让机械臂自由落下。
  中止时只停止下发（控制器保持最后目标），并在日志里给出"要怎么松开"。
* **CSV 用预分配缓冲**：300 s @200 Hz ≈ 6 万行，用 dict 累积会吃掉几十倍内存。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

from .common import (
    ARM_JOINTS,
    CSP_HOLD,
    PROTOCOL_KEEP,
    DyidError,
    config_path,
    default_output_dir,
)
from .csp_client import BridgeClient
from .dataset import (
    DYN_NCOL,
    STA_NCOL,
    STARAW_NCOL,
    RowBuffer,
    append_csv,
    dynamics_header,
    save_csv,
    statics_header,
    statics_raw_header,
)
from .motion import RateLoop, hold_position, move_joints
from .trajectory import FourierTrajectory, load_specs

DEFAULTS = {
    "source": "dyid",
    "publish_rate_hz": 200.0,
    "command_timeout_ms": 300.0,
    "speed_cap": 0.15,
    "return_speed": 0.12,
    "small_delta_rad": 0.15,
    "small_speed": 0.10,
    "small_hold_sec": 1.5,
    "small_reach_ratio": 0.5,
    "small_tolerance_rad": 0.25,
    "ridge": 1e-6,
    "collision_samples": 6001,
    "lpf_hz": 0.2,       # 求解前对 cmd/q/v/tau 做零相位低通（0=不滤）
    "workers": 4,        # 并行进程数（fork+pipe；ROS 节点内如遇异常可设 1）
    "accel": "ref",      # 惯性回归用的加速度来源（仅 with_inertia 时用到）
    "with_inertia": False,  # 默认『静态+摩擦』；惯性项作为可选项
}


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

def load_defaults(path: Optional[Path]) -> dict:
    cfg = dict(DEFAULTS)
    try:
        payload = yaml.safe_load((path or config_path("dyid.yaml")).read_text(encoding="utf-8"))
        node = (payload or {}).get("lite_dyid", {})
        params = node.get("ros__parameters", node) or {}
        for k, v in params.items():
            if k in cfg:
                cfg[k] = v
    except Exception as exc:  # 配置缺失不算致命
        print(f"[dyid] 提示：读取默认配置失败（{exc}），使用内置默认值")
    return cfg


# ---------------------------------------------------------------------------
# 采集
# ---------------------------------------------------------------------------

class Collector:
    def __init__(self, client: BridgeClient, model, specs: Dict[str, FourierTrajectory],
                 cfg: dict, out_dir: Path):
        self.client = client
        self.model = model
        self.specs = specs
        self.cfg = cfg
        self.out_dir = Path(out_dir)
        self.rate = float(cfg["publish_rate_hz"])
        self.lo, self.hi = model.limit_array()
        self.start_pose: Optional[np.ndarray] = None
        self._aborted = False

    # -- 小幅验证 ---------------------------------------------------------

    def small_check(self) -> List[dict]:
        """逐关节小幅点动：命令是否真的到达电机（每关节 ≤ small_delta）。"""
        delta = float(self.cfg["small_delta_rad"])
        speed = float(self.cfg["small_speed"])
        hold = float(self.cfg["small_hold_sec"])
        base = np.asarray(list(self.client.pose_dict().values()), dtype=float)
        rows: List[dict] = []
        print(f"\n[小幅验证] 每关节 ±{math.degrees(delta):.1f}°，速度 "
              f"{speed:.2f} rad/s，共 {len(ARM_JOINTS)} 个关节")
        for k, jn in enumerate(ARM_JOINTS):
            # 往"余量更大"的一侧走
            room_up = self.hi[k] - base[k]
            room_dn = base[k] - self.lo[k]
            sign = 1.0 if room_up >= room_dn else -1.0
            step = sign * min(delta, max(0.0, (room_up if sign > 0 else room_dn) - 0.05))
            if abs(step) < 0.02:
                rows.append({"joint": jn, "cmd_delta": 0.0, "actual_delta": 0.0,
                             "err_rad": float("nan"), "ok": False,
                             "note": "两侧余量都不足 0.02 rad，跳过"})
                print(f"  {jn:22s} ⚠️ 余量不足，跳过")
                continue
            target_out = base.copy(); target_out[k] += step
            move_joints(self.client, target_out, rate_hz=self.rate, speed_cap=speed)
            # ⚠️ 保持期间**继续下发**同一个目标，而不是 sleep：虽然 CSP 陈旧时
            #    也会锁存目标，但持续下发才是"真的在保持"，也与正式采集同路径。
            hold_position(self.client, target_out, duration_sec=hold, rate_hz=self.rate)
            snap_out = self.client.snapshot()
            actual = float(snap_out.position[k] - base[k])
            err = float(snap_out.position[k] - target_out[k])
            ok = (abs(actual) >= float(self.cfg["small_reach_ratio"]) * abs(step)
                  and np.sign(actual) == np.sign(step)
                  and abs(err) <= float(self.cfg["small_tolerance_rad"]))
            move_joints(self.client, base, rate_hz=self.rate, speed_cap=speed)
            hold_position(self.client, base, duration_sec=0.6, rate_hz=self.rate)
            rows.append({"joint": jn, "cmd_delta": float(step), "actual_delta": actual,
                         "err_rad": err, "ok": bool(ok), "note": ""})
            print(f"  {jn:22s} 命令 {math.degrees(step):+6.2f}°  "
                  f"实动 {math.degrees(actual):+6.2f}°  稳态误差 "
                  f"{math.degrees(err):+5.2f}°  {'✓' if ok else '❌'}")
        bad = [r["joint"] for r in rows if not r["ok"]]
        if bad:
            raise DyidError(
                f"小幅验证未通过：{bad}\n"
                "  可能原因：该关节被限位钳住 / 力矩不足顶不动 / 电机不在伺服态。\n"
                "  排查：ros2 run lite_bridge motor_monitor --once"
            )
        print("[小幅验证] 全部通过 ✓")
        return rows

    # -- 单条轨迹 ---------------------------------------------------------

    def collect_trajectory(self, traj: FourierTrajectory, home: np.ndarray) -> dict:
        spec = traj.spec
        check = traj.check()
        print(f"\n[轨迹 {spec.name}] 时长 {traj.duration:.0f}s  频率 {spec.frequency_hz} Hz  "
              f"斜坡 {traj.ramp:.1f}s  峰值速度 {check.v_peak.max():.3f} rad/s  "
              f"峰值加速度 {check.a_peak.max():.3f} rad/s²")
        if not check.ok:
            raise DyidError(f"轨迹 {spec.name} 自检未通过：{check.problems}")
        self._collision_check(traj)

        # ⚠️ 用 `<=`：落在轨迹终点的静止段也必须保留。
        static_times = [t for t in spec.joint_static_times() if t <= traj.duration + 1e-9]
        n_static = len(static_times)

        dyn = RowBuffer(DYN_NCOL, int(traj.duration * self.rate * 1.3) + 100)
        raw = RowBuffer(STARAW_NCOL, int(n_static * spec.static_hold_sec * self.rate * 1.3) + 100)
        stats: List[List[float]] = []

        loop = RateLoop(self.rate)
        t_wall0 = time.monotonic()
        t_motion = 0.0
        static_set = {round(t, 9) for t in static_times}
        stops = sorted(set([round(t, 9) for t in static_times] + [round(traj.duration, 9)]))
        static_seen = 0
        try:
            for stop in stops:
                # ---- 运动段 ----
                loop.start()
                t0 = time.monotonic()
                while True:
                    self.client.assert_healthy()
                    elapsed = time.monotonic() - t0
                    tm = t_motion + elapsed
                    if tm >= stop:
                        break
                    q, _ = traj.evaluate(tm)
                    self.client.publish_position(q)
                    snap = self.client.snapshot()
                    dyn.add([tm, time.monotonic() - t_wall0]
                            + list(q) + list(snap.position)
                            + list(snap.velocity) + list(snap.effort))
                    loop.tick()
                t_motion = stop

                # ---- 静止段（注意：最后一段静止的 stop 可能**等于**轨迹时长，
                #      所以这里先判"是不是静止时刻"，再判"是不是结束"）----
                if round(stop, 9) not in static_set:
                    break
                static_seen += 1
                idx = static_seen
                q_hold, _ = traj.evaluate(stop)
                hold = float(spec.static_hold_sec)
                settle = float(spec.static_settle_sec)
                acc_q: List[np.ndarray] = []
                acc_tau: List[np.ndarray] = []
                loop.start()
                th0 = time.monotonic()
                while True:
                    self.client.assert_healthy()
                    el = time.monotonic() - th0
                    if el >= hold:
                        break
                    self.client.publish_position(q_hold)
                    snap = self.client.snapshot()
                    raw.add([idx, el] + list(snap.position)
                            + list(snap.velocity) + list(snap.effort))
                    if el >= max(0.0, hold - settle):
                        acc_q.append(snap.position.copy())
                        acc_tau.append(snap.effort.copy())
                    loop.tick()
                if not acc_q:
                    raise DyidError(f"{spec.name} 静止段 {idx} 没有有效样本")
                Q = np.vstack(acc_q); T = np.vstack(acc_tau)
                qm = Q.mean(axis=0); tm_ = T.mean(axis=0)
                stats.append([float(idx), stop, float(Q.shape[0]), hold]
                             + list(qm) + list(tm_)
                             + list(Q.std(axis=0)) + list(T.std(axis=0)))
                print(f"  [{spec.name}] 静止 {idx:2d}/{n_static}  t={stop:6.1f}s  "
                      f"n={Q.shape[0]:4d}  τ_max|·|={np.abs(tm_).max():.3f} N·m  "
                      f"σ_pos_max={np.degrees(Q.std(axis=0).max()):.3f}°")
                if stop >= traj.duration - 1e-9:
                    break
        finally:
            pass

        print(f"  [{spec.name}] 运动段样本 {dyn.n}  静止样本 {raw.n}  静止段 {len(stats)}")
        return {"dyn": dyn.array(), "raw": raw.array(),
                "statics": np.asarray(stats, dtype=float)}

    def _collision_check(self, traj: FourierTrajectory) -> None:
        n = int(self.cfg["collision_samples"])
        worst = None
        for t in np.linspace(0.0, traj.duration, n):
            q, _ = traj.evaluate(float(t))
            hits = self.model.collisions(dict(zip(ARM_JOINTS, q)))
            if hits:
                worst = (float(t), hits[0])
                break
        if worst is not None:
            raise DyidError(
                f"轨迹 {traj.spec.name} 在 t={worst[0]:.1f}s 发生 mesh 碰撞："
                f"{worst[1].first} ↔ {worst[1].second}（{worst[1].depth:.4f} m）"
            )
        print(f"  [{traj.spec.name}] mesh 碰撞检查：{n} 个位形，无碰撞 ✓")

    # -- 回位 -------------------------------------------------------------

    def return_home(self, home: np.ndarray) -> None:
        cur = np.asarray(list(self.client.pose_dict().values()), dtype=float)
        d = float(np.max(np.abs(cur - home)))
        print(f"\n[回位] 距起始姿态最大偏差 {math.degrees(d):.2f}°，"
              f"以 {self.cfg['return_speed']} rad/s 规划回位")
        took = move_joints(self.client, home, rate_hz=self.rate,
                           speed_cap=float(self.cfg["return_speed"]))
        settle = min(5.0, max(1.0, took * 0.1))
        time.sleep(settle)
        self.client.publish_position(home)
        cur2 = np.asarray(list(self.client.pose_dict().values()), dtype=float)
        print(f"[回位] 完成，残余偏差 {math.degrees(np.max(np.abs(cur2 - home))):.2f}°")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="dyid_collect",
        description="lite_dyid 主流程：确认 CSP → 小幅验证 → 双臂 300 s 采集 → 解算",
    )
    p.add_argument("--output-dir", default=None, help="结果目录（默认 ~/.ros/lite_dyid/runs）")
    p.add_argument("--source", default=None, help="桥的数据源名（默认 dyid，需已授权）")
    p.add_argument("--trajectories", default=None, help="轨迹 YAML（默认包内 config/trajectories.yaml）")
    p.add_argument("--config", default=None, help="默认参数 YAML")
    p.add_argument("--rate-hz", type=float, default=None, help="下发频率")
    p.add_argument("--speed-cap", type=float, default=None, help="轨迹速度上限 rad/s")
    p.add_argument("--ridge", type=float, default=None, help="重力辨识岭系数")
    p.add_argument("--lpf", type=float, default=None,
                   help="求解前的零相位低通截止频率 Hz（默认 0.2，0=不滤）")
    p.add_argument("--workers", type=int, default=None,
                   help="辨识并行进程数（默认 1=串行；>1 会 fork，ROS 节点内有风险）")
    p.add_argument("--accel", default=None, choices=["ref", "meas"])
    p.add_argument("--with-inertia", action="store_true", default=None,
                   help="额外辨识惯性项（模型 C/D）；默认只做静态+摩擦")
    p.add_argument("--small-delta", type=float, default=None, help="小幅验证单关节幅度 rad")
    p.add_argument("--skip-small-check", action="store_true", help="跳过小幅验证（不推荐）")
    p.add_argument("--small-check-only", action="store_true",
                   help="只完成 CSP 启动检查和逐关节小幅验证，不执行 80% 大幅轨迹")
    p.add_argument("--traj", default=None, help="只跑指定轨迹名（可多次用逗号分隔）")
    p.add_argument("--auto-validate", action="store_true",
                   help="若自定义 YAML 含 role=validate，采集后自动运行该验证轨迹")
    p.add_argument("--no-validate-prompt", action="store_true", help="不询问，直接结束")
    return p


def ask_validate(args) -> bool:
    """最终是否运行自定义 YAML 中可选的 ``role=validate`` 轨迹。

    三态：``--auto-validate`` 直接做；``--no-validate-prompt`` 直接不做；
    默认**在命令行问用户**（非交互 stdin 视为"不做"，不会挂住）。
    抽成独立函数是为了能单测 —— 这是流程的最后一道人工闸门，不该只靠肉眼审代码。
    """
    if args.auto_validate:
        return True
    if args.no_validate_prompt:
        print("\n[验证] --no-validate-prompt：不验证")
        return False
    try:
        ans = input("\n是否运行自定义 role=validate 轨迹？[y/N] ").strip().lower()
    except EOFError:
        print("\n[验证] 无交互输入（stdin 关闭）：视为不验证")
        return False
    return ans in ("y", "yes", "是")


def main(argv: Optional[Sequence[str]] = None) -> int:
    # ⚠️ 用 parse_known_args：由 launch 拉起时，launch_ros 会追加
    #    `--ros-args -r __node:=...`，严格解析会直接报错退出。
    args, _unknown = build_parser().parse_known_args(argv)
    cfg = load_defaults(Path(args.config).expanduser() if args.config else None)
    for key, val in (("source", args.source), ("speed_cap", args.speed_cap),
                     ("ridge", args.ridge), ("small_delta_rad", args.small_delta),
                     ("publish_rate_hz", args.rate_hz),
                     ("lpf_hz", args.lpf), ("workers", args.workers),
                     ("accel", args.accel), ("with_inertia", args.with_inertia)):
        if val is None or (isinstance(val, str) and not val.strip()):
            continue
        cfg[key] = val
    # ⚠️ 下发频率**硬上限 200 Hz**（用户要求）。
    #    为什么要有这道闸：采样点密度 = 下发频率（本包一个循环里既下发又记录），
    #    频率越高点数越多、CAN 带宽与磁盘写入压力越大；实测 1 Mbit/s 总线在
    #    14 台电机 × 500 Hz 下余量不足（会出现 TX_QUEUE_OVERRUN）。
    #    200 Hz 是"点数翻倍"与"总线安全"的折中，且**不改变轨迹的时间-状态关系**
    #    （q(t) 一模一样，只是采得更细）。
    rate = float(cfg.get("publish_rate_hz", 200.0))
    if rate > 200.0:
        print(f"[dyid] publish_rate_hz={rate} > 200 → 已钳到 200 Hz（硬上限）")
        cfg["publish_rate_hz"] = 200.0
    if rate <= 0.0:
        raise DyidError(f"publish_rate_hz 必须为正，得到 {rate}")

    out_root = Path(args.output_dir).expanduser() if args.output_dir else default_output_dir()
    out_dir = out_root / datetime.now().strftime("run_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=False)
    print(f"[dyid] 结果目录：{out_dir}")

    client = BridgeClient(source=cfg["source"], command_timeout_ms=cfg["command_timeout_ms"])
    ok = False
    entered_tracking = False
    try:
        # ---- 预检 ----
        client.wait_joint_states(timeout=25.0)
        urdf = client.wait_urdf(timeout=20.0)
        (out_dir / "robot.urdf").write_text(urdf, encoding="utf-8")
        from .model import RobotModel
        model = RobotModel(urdf)
        print(f"[预检] URDF {len(urdf)} 字节；{len(ARM_JOINTS)} 臂关节；"
              f"mesh 碰撞对 {model.n_collision_pairs}")

        level, flags, src = client.safety()
        if level >= 2:
            print(f"[预检] /safety_status level={level} flags={flags}（{src}）→ 尝试 clear_fault")
            try:
                res = client.clear_fault()
                print(f"[预检] clear_fault: {res.message}")
            except DyidError as exc:
                print(f"[预检] clear_fault 失败：{exc}")
            time.sleep(1.0)
        client.assert_healthy()
        print("[预检] 安全状态 OK")

        notes = client.ensure_csp_ready()
        entered_tracking = True
        print(f"[预检] {notes}")

        start_pose = np.asarray(list(client.pose_dict().values()), dtype=float)
        print("[预检] 起始姿态：")
        for jn, q in zip(ARM_JOINTS, start_pose):
            print(f"    {jn:22s} {math.degrees(q):+8.2f}°")

        specs = load_specs(Path(args.trajectories).expanduser() if args.trajectories
                           else config_path("trajectories.yaml"))
        if args.traj:
            wanted = [s.strip() for s in args.traj.split(",")]
            unknown = [n for n in wanted if n not in specs]
            if unknown:
                raise DyidError(f"--traj 指定的轨迹不存在：{unknown}；可选 {list(specs)}")
        else:
            wanted = [s.name for s in specs.values() if s.role == "train"]
        train_specs = [specs[n] for n in wanted]
        val_specs = [s for s in specs.values() if s.role == "validate"]

        (out_dir / "run_meta.json").write_text(json.dumps({
            "started": datetime.now().isoformat(timespec="seconds"),
            "config": cfg, "train_specs": [s.name for s in train_specs],
            "validate_specs": [s.name for s in val_specs],
            "start_pose_rad": start_pose.tolist(),
            "joints": list(ARM_JOINTS),
        }, indent=2, ensure_ascii=False), encoding="utf-8")

        collector = Collector(client, model, specs, cfg, out_dir)

        # ---- 小幅验证 ----
        if args.skip_small_check:
            print("[小幅验证] 已跳过（--skip-small-check）")
            small_rows: List[dict] = []
        else:
            small_rows = collector.small_check()
            from .report import write_stats_csv
            write_stats_csv(out_dir / "small_check.csv", small_rows)

        if args.small_check_only:
            print("[dyid] --small-check-only：小幅验证完成，结束前保持当前位置并切回 CSP_HOLD")
            ok = True
            return 0

        # ---- 采集训练轨迹（默认仅一条双臂同步 300 s 交付轨迹）----
        home = start_pose
        statics_all: List[np.ndarray] = []
        first = True
        for spec in train_specs:
            entry = np.asarray(list(client.pose_dict().values()), dtype=float)
            traj = FourierTrajectory(spec, entry, model.limit_array(),
                                     speed_cap=float(cfg["speed_cap"]))
            res = collector.collect_trajectory(traj, home)
            if first:
                save_csv(out_dir / "dynamics.csv", dynamics_header(), res["dyn"],
                         traj_name=spec.name)
                first = False
            else:
                append_csv(out_dir / "dynamics.csv", dynamics_header(), res["dyn"],
                           traj_name=spec.name)
            save_csv(out_dir / f"dynamics_{spec.name}.csv", dynamics_header(), res["dyn"],
                     traj_name=spec.name)
            save_csv(out_dir / f"statics_{spec.name}.csv", statics_header(), res["statics"],
                     traj_name=spec.name)
            save_csv(out_dir / f"statics_raw_{spec.name}.csv", statics_raw_header(),
                     res["raw"], traj_name=spec.name)
            statics_all.append(res["statics"])
            collector.return_home(home)

        # 合并 CSV
        all_st = np.vstack(statics_all) if statics_all else np.zeros((0, STA_NCOL))
        save_csv(out_dir / "statics.csv", statics_header(), all_st, traj_name="train")
        # dynamics.csv 已逐条写入；这里保证文件存在
        if not (out_dir / "dynamics.csv").is_file():
            raise DyidError("dynamics.csv 未生成")

        # ---- 自动解算 ----
        print("\n" + "=" * 72)
        print("自动解算：重力（质心/质量）→ 冻结 → 摩擦 c1±/c2")
        print("=" * 72)
        from .dataset import read_table
        from .identify import identify_fast, save_params, _summary
        statics_tbl = read_table(out_dir / "statics.csv")
        lpf = float(cfg["lpf_hz"]); nw = int(cfg["workers"])
        print(f"  求解前零相位低通 {lpf} Hz（0=不滤）；并行 {nw} 进程；"
              f"加速度={cfg['accel']}")
        grav, fric, fit, _tr = identify_fast(
            model, urdf, statics_tbl, out_dir / "dynamics.csv",
            lpf_hz=lpf, workers=nw, ridge=float(cfg["ridge"]), accel=str(cfg["accel"]),
            with_inertia=bool(cfg["with_inertia"]))
        save_params(out_dir / "params.json", grav, fric,
                    meta={"run_dir": str(out_dir), "ridge": float(cfg["ridge"]),
                          "lpf_hz": lpf, "workers": nw, "accel": str(cfg["accel"]),
                          "fast": True,
                          "rate_hz": float(cfg["publish_rate_hz"]),
                          "trajectories": [s.name for s in train_specs]},
                    urdf_text=urdf,
                    model_kind=grav.model_kind,
                    fit=fit,
                    # ⚠️ 只有 `--with-inertia` 的 fit 才含 pi_I/pi_ID/rank_I。
                    #    默认「静态+摩擦」没有这些键 —— 无条件取键会在**采集完成之后**
                    #    崩掉（KeyError: 'pi_I'），连 params.json 都写不出来。
                    extra=None if (fit is None or "pi_I" not in fit) else {
                        "inertia": {"pi_I": np.asarray(fit["pi_I"]).tolist(),
                                    "pi_ID": np.asarray(fit["pi_ID"]).tolist(),
                                    "rank_I": int(fit["rank_I"]),
                                    "n_train": int(fit["n_train"])}})
        print(_summary(grav, fric))
        from . import report
        report.plot_training(out_dir, model, grav, fric)
        print(f"\n训练集拟合残差 RMS = {grav.fit_rms:.4f} N·m（重力，低通 {lpf} Hz）")

        ok = True

        # ---- 询问是否验证 ----
        if not val_specs:
            print("\n[验证] trajectories.yaml 里没有 role=validate 的轨迹，跳过")
            return 0
        do = ask_validate(args)
        if not do:
            print("[验证] 已跳过。随时可离线重解：dyid_identify --run-dir "
                  f"{out_dir}")
            return 0

        # ---- 可选的自定义验证轨迹（默认交付 YAML 不包含）----
        vdir = out_dir / "validation"
        vdir.mkdir(exist_ok=True)
        spec = val_specs[0]
        entry = np.asarray(list(client.pose_dict().values()), dtype=float)
        traj = FourierTrajectory(spec, entry, model.limit_array(),
                                 speed_cap=float(cfg["speed_cap"]))
        res = collector.collect_trajectory(traj, home)
        save_csv(vdir / "dynamics_validate.csv", dynamics_header(), res["dyn"],
                 traj_name=spec.name)
        collector.return_home(home)

        from .validate import validate_run
        validate_run(out_dir, model, params_path=out_dir / "params.json",
                     dynamics_csv=vdir / "dynamics_validate.csv", out_dir=vdir,
                     urdf_text=urdf, lpf_hz=float(cfg["lpf_hz"]),
                     workers=int(cfg["workers"]), accel=str(cfg["accel"]),
                     with_inertia=bool(cfg["with_inertia"]))
        print(f"\n[验证] 结果见 {vdir}")
        return 0

    except KeyboardInterrupt:
        print("\n[dyid] 操作员 Ctrl+C 中止")
        return 130
    except DyidError as exc:
        print(f"\n❌ {exc}")
        return 2
    finally:
        # =====================================================================
        # 收尾序列：**先停发 → 回安全档 → 再交权**
        # =====================================================================
        #
        # ⚠️ 顺序不可调换，理由见 `BridgeClient.finish_session` 的文档：
        #   ① 先停发 MotionCommand（避免"还在发旧目标、同时又在切控制器"）；
        #   ② 等桥侧判本源失活进 SAFE_HOLD（source_timeouts_ms 默认 500 ms）；
        #   ③ set_mode(CSP_HOLD) 把机械臂保持在当前目标
        #      ⚠️ **不能**切 CSP_MONITOR(5)：CSP 没有"松劲"，切 5 会失能 → 自由落体；
        #   ④ set_source(AUTO) 把控制权干净地还回去。
        #
        # ⚠️ ③ 只在**真的进过 CSP_TRACKING**（= 电机已使能）时才做：
        #    若预检阶段就失败（还停在 mode5 未使能），切 6 会**把电机使能起来** ——
        #    那是在"什么都没做"的情况下凭空上电，绝不能发生。
        try:
            held = None
            if entered_tracking:
                cur = client.pose_dict()
                if cur:
                    held = np.asarray(list(cur.values()), dtype=float)
            summary = client.finish_session(entered_tracking=entered_tracking,
                                            held_position=held)
            print(f"[dyid] 收尾：{summary}")
            if entered_tracking:
                print("       要松开：ros2 service call /lite_bridge/set_mode "
                      "robstride_msgs/srv/SetMode \"{protocol: 1, mode: 5, force: true}\""
                      "（注意支撑，电机将自由）")
            else:
                print("[dyid] 未进入 CSP_TRACKING，未使能电机（保持当前模式不变）")
        except Exception as exc:                       # 收尾失败不影响退出码
            print(f"[dyid] 收尾序列异常（已尽力）：{exc}")
            if entered_tracking:
                try:
                    client.set_mode(PROTOCOL_KEEP, CSP_HOLD, timeout=15.0)
                    print("[dyid] 兜底：已切 CSP_HOLD")
                except Exception as exc2:
                    print(f"[dyid] 兜底切 CSP_HOLD 也失败：{exc2}")
        client.shutdown()
        _ = ok


if __name__ == "__main__":
    sys.exit(main())

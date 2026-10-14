# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""第三条（验证）轨迹：把辨识出的重力+摩擦代进去，算残差、出统计与可视化。

残差定义
========
    resid(t) = τ_实测(t) − [ τ_grav(q(t)) + c1(sign v) + c2·v ]

残差里如果出现明显的**惯性特征**（与 v̇ 同相），说明准静态假设在本轨迹上
不成立 —— 图里能直接看出来，而不是被一个总数掩盖。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from .common import ARM_JOINTS, DyidError
from .dataset import joint_matrix, read_table
from .identify import load_params, predict_torque


_MODEL_DESC = {"A": "纯静力学", "B": "静力学+摩擦", "C": "静力学+惯性项",
               "D": "静力学+摩擦+惯性"}


def validate_run(run_dir: Path, model, *, params_path: Path, dynamics_csv: Path,
                 out_dir: Path, urdf_text: str = "", lpf_hz: float = 0.2,
                 workers: int = 4, accel: str = "ref",
                 with_inertia: bool = False) -> dict:
    """在给定 CSV 上算残差并出图。

    ⚠️ **必须与辨识用同一个滤波带宽**：模型是在滤波后的数据上拟合的，
    如果验证时拿未滤波的实测力矩去比，残差里会混进测量噪声与高频未建模效应，
    看起来"模型很差"，其实只是带宽不一致。``lpf_hz=0`` 复现未滤波的口径。
    """
    params = load_params(params_path)
    pi_I = np.asarray(params.get("inertia", {}).get("pi_I", []), dtype=float)
    pi_ID = np.asarray(params.get("inertia", {}).get("pi_ID", []), dtype=float)
    c = params["friction"]
    c_B = np.zeros(3 * len(ARM_JOINTS))
    c_B[0::3] = c["c1_pos"]; c_B[1::3] = c["c1_neg"]; c_B[2::3] = c["c2"]
    if urdf_text:
        from . import fastsolve
        pr = fastsolve.predict(model, urdf_text, Path(dynamics_csv),
                               pi_grav=np.asarray(params["gravity"]["params"], dtype=float),
                               pi_I=pi_I if pi_I.size else None, c_B=c_B,
                               pi_ID=pi_ID if pi_ID.size else None,
                               c_D=c_B, lpf_hz=lpf_hz, workers=workers, accel=accel,
                               with_inertia=with_inertia and pi_I.size > 0)
        t, tau = pr["t"], pr["tau"]
        # 主模型 = B（静态+摩擦）；--with-inertia 时把 C/D 也一并报出来
        idx_b = pr["models"].index("B")
        pred = pr["pred"][:, idx_b, :]
        resid = tau - pred
        if lpf_hz > 0:
            print(f"[验证] 与辨识同一带宽：零相位低通 {lpf_hz} Hz（并行 {workers}）")
        if pr["with_inertia"]:
            print("\n各模型全局残差 RMS（N·m）：")
            for k, nm in enumerate(pr["models"]):
                r = tau - pr["pred"][:, k, :]
                flag = "  ← 主模型" if nm == "B" else ""
                print(f"  {nm}  {_MODEL_DESC[nm]:22s} RMS={np.sqrt(np.mean(r ** 2)):.4f}  "
                      f"max|·|={np.max(np.abs(r)):.4f}{flag}")
        return _finish(run_dir, out_dir, t, tau, pred, resid, dynamics_csv)

    # 回退：无 URDF / 无惯性参数时按老口径（未滤波）算
    tbl = read_table(dynamics_csv)
    if tbl.n == 0:
        raise DyidError(f"{dynamics_csv} 没有数据行")
    q = joint_matrix(tbl, "q"); v = joint_matrix(tbl, "v"); tau = joint_matrix(tbl, "tau")
    t = tbl.col("motion_t")
    ok = np.all(np.isfinite(q), axis=1) & np.all(np.isfinite(v), axis=1) & \
        np.all(np.isfinite(tau), axis=1)
    if int(np.sum(~ok)):
        print(f"[验证] 丢弃 {int(np.sum(~ok))} 行含 NaN 的样本")
    q, v, tau, t = q[ok], v[ok], tau[ok], t[ok]
    pred = np.vstack([predict_torque(model, params, q[i], v[i]) for i in range(q.shape[0])])
    resid = tau - pred
    return _finish(run_dir, out_dir, t, tau, pred, resid, dynamics_csv)


def _finish(run_dir: Path, out_dir: Path, t, tau, pred, resid, dynamics_csv: Path) -> dict:

    from . import report
    report.plot_validation(Path(out_dir), t, tau, pred, resid,
                           title=f"Validation ({Path(run_dir).name})")
    rows = report.residual_stats(resid, tau)

    print("\n" + "=" * 74)
    print("验证残差统计（实测 − 计算）")
    print("=" * 74)
    print(f"{'joint':22s} {'rms':>8s} {'std':>8s} {'max|·|':>8s} {'p95':>8s} "
          f"{'rms%额定':>9s}")
    for r in rows:
        print(f"{r['joint']:22s} {r['rms']:8.4f} {r['std']:8.4f} {r['max_abs']:8.4f} "
              f"{r['p95_abs']:8.4f} {r['rms_pct_rated']:9.1f}")
    all_r = resid[np.isfinite(resid)]
    print(f"\n全局：RMS={np.sqrt(np.mean(all_r**2)):.4f} N·m  "
          f"max|·|={np.max(np.abs(all_r)):.4f} N·m  n={all_r.size}")
    print(f"可视化与统计已写入 {out_dir}")
    _ = run_dir
    return {"rows": rows, "rms": float(np.sqrt(np.mean(all_r ** 2)))}


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="dyid_validate",
                                description="用已辨识参数在给定动力学 CSV 上算残差并可视化")
    p.add_argument("--run-dir", required=True, help="采集输出目录（含 robot.urdf / params.json）")
    p.add_argument("--dynamics", default=None, help="要验证的 dynamics CSV（默认 validation/dynamics_validate.csv）")
    p.add_argument("--out", default=None, help="输出目录（默认 run-dir/validation）")
    p.add_argument("--urdf", default=None)
    p.add_argument("--params", default=None)
    p.add_argument("--lpf", type=float, default=0.2,
                   help="验证前同样做零相位低通（0=不滤；应与辨识时一致）")
    p.add_argument("--workers", type=int, default=4, help="并行进程数（默认 4）")
    p.add_argument("--with-inertia", action="store_true",
                   help="连模型 C/D 一起算（需要 params.json 里有 inertia；否则忽略）")
    p.add_argument("--accel", default="ref", choices=["ref", "meas"])
    args, _unknown = p.parse_known_args(argv)

    from .model import RobotModel
    run = Path(args.run_dir).expanduser()
    urdf_path = Path(args.urdf) if args.urdf else run / "robot.urdf"
    if not urdf_path.is_file():
        raise DyidError(f"找不到 URDF：{urdf_path}")
    model = RobotModel(urdf_path.read_text(encoding="utf-8"))
    params = Path(args.params) if args.params else run / "params.json"
    dyn = Path(args.dynamics) if args.dynamics else run / "validation" / "dynamics_validate.csv"
    out = Path(args.out) if args.out else run / "validation"
    validate_run(run, model, params_path=params, dynamics_csv=dyn, out_dir=out,
                 urdf_text=urdf_path.read_text(encoding="utf-8"),
                 lpf_hz=args.lpf, workers=args.workers, accel=args.accel,
                 with_inertia=args.with_inertia)
    return 0


if __name__ == "__main__":
    sys.exit(main())

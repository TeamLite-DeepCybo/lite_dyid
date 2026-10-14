# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""可视化与统计输出（matplotlib，Agg 后端，全部落 PNG）。"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .common import ARM_JOINTS

#: 每关节的额定力矩（N·m）—— 只用于把残差折算成"占额 %"
RATED_TORQUE = {
    "left_shoulder_pitch": 7.0, "left_shoulder_roll": 5.0, "left_shoulder_yaw": 5.0,
    "left_elbow_pitch": 5.0, "left_wrist_yaw": 1.8, "left_wrist_roll": 1.8,
    "left_wrist_pitch": 1.8,
    "right_shoulder_pitch": 7.0, "right_shoulder_roll": 5.0, "right_shoulder_yaw": 5.0,
    "right_elbow_pitch": 5.0, "right_wrist_yaw": 1.8, "right_wrist_roll": 1.8,
    "right_wrist_pitch": 1.8,
}


def _ensure_mpl_config_dir() -> None:
    """确保 matplotlib 的配置/字体缓存目录可写。

    为什么需要：matplotlib 首次导入会写 ``~/.cache/matplotlib/fontlist-*.json``，
    在容器 / 受限权限 / 只读 HOME 的环境下会直接
    ``PermissionError`` 让整个辨识流程在**出图这一步**失败
    （数据都在，只是图没出来 —— 很迷惑人）。
    这里先探测默认目录；不可写就退到临时目录，并显式设置 MPLCONFIGDIR。
    """
    import os
    import tempfile
    from pathlib import Path as _P

    if os.environ.get("MPLCONFIGDIR"):
        return
    cand = _P.home() / ".cache" / "matplotlib"
    try:
        cand.mkdir(parents=True, exist_ok=True)
        probe = cand / ".lite_dyid_probe"
        probe.write_text("", encoding="utf-8")
        probe.unlink()
    except Exception:
        fallback = _P(tempfile.gettempdir()) / "lite_dyid_mplconfig"
        fallback.mkdir(parents=True, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = str(fallback)


def _plt():
    _ensure_mpl_config_dir()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # ⚠️ 字体：**图上的文字一律用 ASCII**。原因：matplotlib 遇到缺字形只会打
    #    一条 UserWarning、然后把汉字画成方框 —— 图看着"有标题"其实读不出来，
    #    而不同机器上有没有 CJK 字体完全不可控（本机的 Noto CJK 是 .ttc，
    #    matplotlib 3.6 解析不到）。要中文请自行装字体并在下面加入候选名。
    plt.rcParams["axes.unicode_minus"] = False
    try:
        from matplotlib import font_manager
        have = {f.name for f in font_manager.fontManager.ttflist}
        # ⚠️ 名字要按**实际可见的字体名**列：本机是 "Noto Sans CJK JP"
        #    （不是 SC！）与 "AR PL UMing CN"。写错名字 matplotlib 不报错，
        #    只是找不到字体、把汉字画成方框。
        for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Serif CJK SC",
                     "Noto Serif CJK JP", "WenQuanYi Zen Hei",
                     "AR PL UMing CN", "AR PL UMing TW"):
            if cand in have:
                plt.rcParams["font.sans-serif"] = [cand, "DejaVu Sans"]
                break
    except Exception:
        pass
    return plt


def _grid(n: int = 14, ncols: int = 2):
    plt = _plt()
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 1.9 * nrows), sharex=False)
    return plt, fig, np.atleast_1d(axes).ravel()


def residual_stats(resid: np.ndarray, tau_meas: Optional[np.ndarray] = None) -> List[Dict[str, float]]:
    """逐关节统计：均值/标准差/RMS/最大绝对残差/占额 %。"""
    out = []
    for j, jn in enumerate(ARM_JOINTS):
        r = resid[:, j]
        r = r[np.isfinite(r)]
        if r.size == 0:
            out.append({"joint": jn})
            continue
        rated = RATED_TORQUE.get(jn, 1.0)
        row = {
            "joint": jn,
            "n": float(r.size),
            "mean": float(np.mean(r)),
            "std": float(np.std(r)),
            "rms": float(np.sqrt(np.mean(r ** 2))),
            "max_abs": float(np.max(np.abs(r))),
            "p95_abs": float(np.percentile(np.abs(r), 95)),
            "rms_pct_rated": 100.0 * float(np.sqrt(np.mean(r ** 2))) / rated,
            "max_pct_rated": 100.0 * float(np.max(np.abs(r))) / rated,
        }
        if tau_meas is not None:
            t = tau_meas[:, j]
            t = t[np.isfinite(t)]
            if t.size:
                row["tau_meas_rms"] = float(np.sqrt(np.mean(t ** 2)))
                row["tau_meas_max_abs"] = float(np.max(np.abs(t)))
        out.append(row)
    return out


def write_stats_csv(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys: List[str] = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def plot_training(run_dir: Path, model, grav, fric) -> Path:
    """训练集自检：静止样本 τ_meas vs τ_grav，以及摩擦拟合残差。"""
    from .dataset import joint_matrix, read_table

    plt = _plt()
    run_dir = Path(run_dir)
    statics = read_table(run_dir / "statics.csv")
    q = joint_matrix(statics, "q")
    tau = joint_matrix(statics, "tau")
    pred = np.vstack([model.gravity_torque(model.q_from_list(q[i]), grav.params)
                      for i in range(q.shape[0])])

    plt2, fig, axes = _grid(14, 2)
    for j, jn in enumerate(ARM_JOINTS):
        ax = axes[j]
        ax.plot(tau[:, j], pred[:, j], ".", ms=3, label="static poses")
        lo = float(min(tau[:, j].min(), pred[:, j].min()))
        hi = float(max(tau[:, j].max(), pred[:, j].max()))
        ax.plot([lo, hi], [lo, hi], "k--", lw=0.8)
        ax.set_title(f"{jn}  rms={grav.resid_rms_joint[j]:.3f} N·m", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    fig.suptitle("Gravity fit check: measured vs computed (static poses)", fontsize=11)
    fig.tight_layout()
    p1 = run_dir / "train_gravity_fit.png"
    fig.savefig(p1, dpi=130)
    plt.close(fig)

    _, fig, axes = _grid(14, 2)
    for j, jn in enumerate(ARM_JOINTS):
        ax = axes[j]
        ax.bar([0], [fric.c1_pos[j]], color="tab:blue", label="c1+")
        ax.bar([1], [fric.c1_neg[j]], color="tab:red", label="c1-")
        ax.bar([2], [fric.c2[j]], color="tab:green", label="c2")
        ax.set_xticks([0, 1, 2]); ax.set_xticklabels(["c1+", "c1-", "c2"], fontsize=7)
        ax.set_title(f"{jn}  rms={fric.resid_rms_joint[j]:.3f}", fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3, axis="y")
    fig.suptitle("Friction coefficients c1+/c1-/c2 (training dynamics)", fontsize=11)
    fig.tight_layout()
    p2 = run_dir / "train_friction.png"
    fig.savefig(p2, dpi=130)
    plt.close(fig)
    return p1


def plot_validation(out_dir: Path, time_sec: np.ndarray, tau_meas: np.ndarray,
                    tau_pred: np.ndarray, resid: np.ndarray,
                    *, title: str = "Validation trajectory") -> List[Path]:
    """残差的可视化：时序、实测vs计算、直方图、汇总条。"""
    plt = _plt()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []

    _, fig, axes = _grid(14, 2)
    for j, jn in enumerate(ARM_JOINTS):
        ax = axes[j]
        ax.plot(time_sec, resid[:, j], lw=0.7, color="tab:purple")
        ax.axhline(0.0, color="k", lw=0.6)
        r = resid[:, j]
        ax.set_title(f"{jn}  rms={np.sqrt(np.mean(r**2)):.3f}  "
                     f"max={np.max(np.abs(r)):.3f} N·m", fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
    fig.suptitle(f"{title}: torque residual (measured - computed) over time", fontsize=11)
    fig.tight_layout()
    p = out_dir / "residual_timeseries.png"; fig.savefig(p, dpi=130); plt.close(fig)
    written.append(p)

    _, fig, axes = _grid(14, 2)
    for j, jn in enumerate(ARM_JOINTS):
        ax = axes[j]
        ax.plot(time_sec, tau_meas[:, j], lw=0.8, label="measured", color="tab:blue")
        ax.plot(time_sec, tau_pred[:, j], lw=0.8, label="computed", color="tab:orange", alpha=0.8)
        ax.set_title(jn, fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(fontsize=7)
    fig.suptitle(f"{title}: measured vs computed torque", fontsize=11)
    fig.tight_layout()
    p = out_dir / "torque_compare.png"; fig.savefig(p, dpi=130); plt.close(fig)
    written.append(p)

    _, fig, axes = _grid(14, 2)
    for j, jn in enumerate(ARM_JOINTS):
        ax = axes[j]
        ax.hist(resid[:, j], bins=40, color="tab:gray")
        ax.set_title(jn, fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
    fig.suptitle(f"{title}: residual distribution", fontsize=11)
    fig.tight_layout()
    p = out_dir / "residual_hist.png"; fig.savefig(p, dpi=130); plt.close(fig)
    written.append(p)

    rows = residual_stats(resid, tau_meas)
    rms = [r.get("rms", 0.0) for r in rows]
    mx = [r.get("max_abs", 0.0) for r in rows]
    pct = [r.get("rms_pct_rated", 0.0) for r in rows]
    x = np.arange(len(ARM_JOINTS))
    fig, axes = plt.subplots(2, 1, figsize=(13, 7))
    axes[0].bar(x - 0.2, rms, width=0.4, label="RMS", color="tab:blue")
    axes[0].bar(x + 0.2, mx, width=0.4, label="max|·|", color="tab:red")
    axes[0].set_xticks(x); axes[0].set_xticklabels(ARM_JOINTS, rotation=60, ha="right", fontsize=7)
    axes[0].set_ylabel("N·m"); axes[0].legend(); axes[0].grid(alpha=0.3, axis="y")
    axes[0].set_title(f"{title}: per-joint residual")
    axes[1].bar(x, pct, color="tab:green")
    axes[1].set_xticks(x); axes[1].set_xticklabels(ARM_JOINTS, rotation=60, ha="right", fontsize=7)
    axes[1].set_ylabel("RMS as % of rated"); axes[1].grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = out_dir / "residual_summary.png"; fig.savefig(p, dpi=130); plt.close(fig)
    written.append(p)

    write_stats_csv(out_dir / "residual_stats.csv", rows)
    written.append(out_dir / "residual_stats.csv")
    return written

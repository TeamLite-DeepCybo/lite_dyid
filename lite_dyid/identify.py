# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""两段式辨识：

第一段（静力学 CSV → 重力模型）
================================
冻结摩擦、只留重力。对每个静止姿态：

    τ_meas = Y(q) · π ,   π = [m_i, (m·c)_i]  （每连杆 4 个参数）

``Y`` 由 :mod:`lite_dyid.model` 用 pinocchio 的"单位惯性基"构造。
解出 ``π`` 后即得每连杆 **质量 m** 与 **质心 c = (m·c)/m**，随后**冻结**。

不可辨识性
----------
纯重力数据只能定出 ``(m, m·c)`` 的某些组合（例如同关节刚接的几节连杆、
以及某些轴向上的分量）。本实现用**列归一化 + 向 URDF 标称值收缩的岭回归**
求稳定解，并输出奇异值谱与条件数 —— 让人一眼看出"哪些方向是真数据定的、
哪些方向是靠先验定的"，而不是报一组看起来精确、其实任意的数字。

第二段（动力学 CSV → 摩擦）
============================
认为 ``τ_meas = τ_grav(q) + τ_fric(v)``，摩擦建模为

    τ_fric(v) = c1⁺ (v>0)  /  c1⁻ (v<0)  +  c2 · v

即 **库仑项分速度符号、粘滞项共用一个 c2**。逐关节线性最小二乘（3 个参数）。
默认模型不引入惯性项；需要惯量张量时显式使用 ``--with-inertia``。
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .common import ARM_JOINTS, DyidError
from .dataset import Table, read_table


@dataclass
class GravityResult:
    link_names: List[str]
    params: np.ndarray
    masses: np.ndarray
    coms: np.ndarray                 # (n_links, 3)
    nominal_masses: np.ndarray
    nominal_coms: np.ndarray
    singular_values: np.ndarray
    rank: int
    cond: float
    resid: np.ndarray                # 每个静止样本（14 关节展开）的残差
    resid_rms_joint: np.ndarray      # (14,)
    scale_check: float               # 最优整体比例（应≈1）
    corr_joint: np.ndarray           # (n_joints,) 实测/预测相关系数
    prior_source: str = ""
    joints: List[str] = field(default_factory=lambda: list(ARM_JOINTS))
    #: 这条结果来自哪个辨识模型（写进 schema2 的 inertia.mode）。
    #: 取值见 MODEL_KINDS；默认"静态+摩擦"（= 惯量张量置 0）。
    model_kind: str = "static_friction_only"

    @property
    def fit_rms(self) -> float:
        return float(np.sqrt(np.mean(self.resid ** 2)))

    def arm_mass(self, first: int, last: int) -> float:
        return float(np.sum(self.masses[first:last]))

    def arm_com(self, first: int, last: int) -> np.ndarray:
        mu = self.params.reshape(-1, 4)[first:last, 1:4]
        tot = float(np.sum(self.masses[first:last]))
        return (np.sum(mu, axis=0) / tot) if abs(tot) > 1e-9 else np.zeros(3)


@dataclass
class FrictionResult:
    c1_pos: np.ndarray
    c1_neg: np.ndarray
    c2: np.ndarray
    resid_rms_joint: np.ndarray
    n_samples: int
    joints: List[str] = field(default_factory=lambda: list(ARM_JOINTS))


def model_joints(model) -> List[str]:
    """模型实际参与的臂关节（真机 = ARM_JOINTS；小 URDF 单测时可以更少）。"""
    joints = getattr(model, "joints", None)
    return list(joints) if joints else list(ARM_JOINTS)


def _stack_gravity(model, table: Table) -> Tuple[np.ndarray, np.ndarray]:
    """把静止样本堆成 (n, n_params) 与 (n,)。"""
    joints = model_joints(model)
    q = table.cols([f"q/{j}" for j in joints])
    tau = table.cols([f"tau/{j}" for j in joints])
    if q.shape[0] == 0:
        raise DyidError("静力学 CSV 没有数据行")
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(tau)):
        raise DyidError("静力学 CSV 里有 NaN（采集或写盘有问题）")
    A = np.vstack([model.arm_regressor(model.q_from_list(q[i])) for i in range(q.shape[0])])
    return A, tau.reshape(-1)


def identify_gravity(
    model,
    statics: Table,
    *,
    method: str = "bounded",
    ridge: float = 1e-6,
    use_nominal: bool = True,
) -> GravityResult:
    """静力学 → 重力参数。

    ``method``：
      * ``bounded``（默认）—— 带约束最小二乘：**质量 ≥ 0**（``scipy.optimize.lsq_linear``），
        其余参数自由。这样即使数据欠定，解也是物理上可接受的。
      * ``ridge``  —— 岭回归（无约束），用于对照。

    ``ridge``：一个很小的 L2 项，只用来在**不可辨识方向**上给出确定的取值；
    它把解拉向 ``prior``（见下）。

    ⚠️ **先验的来源**：本机 URDF 的 ``<inertial>`` 是**占位值**
    （整机总质量 0.88 g，MJCF 生成脚本还会把 0 质量填成 1 g），不能当先验。
    所以默认逻辑是"URDF 总质量 > 10 g 才拿它当先验，否则用 0"。
    """
    from scipy.optimize import lsq_linear

    A, b = _stack_gravity(model, statics)
    nominal = model.nominal_arm_params()
    nominal_total = float(np.sum(nominal[0::4]))
    if use_nominal and nominal_total > 0.01:
        prior, prior_source = nominal, f"URDF 标称（总质量 {nominal_total:.3f} kg）"
    else:
        prior, prior_source = np.zeros_like(nominal), \
            f"零（URDF 标称总质量仅 {nominal_total * 1000:.2f} g，判为占位值）"

    # 列归一化：不同列量纲差几个数量级，不归一化会让正则项"只压某几列"
    scale = np.linalg.norm(A, axis=0)
    scale[scale < 1e-12] = 1.0
    As = A / scale
    ns = prior * scale

    lam = float(ridge)
    if lam > 0.0:
        aug = np.vstack([As, np.sqrt(lam) * np.eye(As.shape[1])])
        rhs = np.concatenate([b, np.sqrt(lam) * ns])
    else:
        aug, rhs = As, b

    if method == "bounded":
        lb = np.full(aug.shape[1], -np.inf)
        ub = np.full(aug.shape[1], np.inf)
        n_var = As.shape[1]
        lb[0:n_var:4] = 0.0                     # 质量列 ≥ 0
        res = lsq_linear(aug, rhs, bounds=(lb, ub), max_iter=400, tol=1e-12)
        sol = res.x
    elif method == "ridge":
        sol, *_ = np.linalg.lstsq(aug, rhs, rcond=None)
    else:
        raise DyidError(f"未知 method={method}（bounded / ridge）")
    params = sol / scale

    joints = model_joints(model)
    resid = A @ params - b
    resid_j = resid.reshape(-1, len(joints))
    rms_j = np.sqrt(np.mean(resid_j ** 2, axis=0))

    # 奇异值谱用**未加正则**的归一化矩阵，才反映真实可辨识性
    sv_raw = np.linalg.svd(As, compute_uv=False)
    tol = sv_raw[0] * max(As.shape) * np.finfo(float).eps
    real_rank = int(np.sum(sv_raw > tol))
    cond = float(sv_raw[0] / sv_raw[-1]) if sv_raw[-1] > 0 else float("inf")

    # 整体比例自检：若 URDF 的符号/量纲约定不对，这里会明显偏离 1
    pred = A @ params
    k = float((pred @ b) / max(1e-12, pred @ pred))
    corr = np.zeros(len(joints))
    for j in range(len(joints)):
        pj, bj = pred.reshape(-1, len(joints))[:, j], b.reshape(-1, len(joints))[:, j]
        if np.std(pj) > 1e-12 and np.std(bj) > 1e-12:
            corr[j] = float(np.corrcoef(pj, bj)[0, 1])

    masses = params[0::4].copy()
    mu = params.reshape(-1, 4)[:, 1:4]
    coms = np.zeros_like(mu)
    for i, m in enumerate(masses):
        if abs(m) > 1e-6:
            coms[i] = mu[i] / m

    return GravityResult(
        link_names=list(model.link_names),
        params=params,
        masses=masses,
        coms=coms,
        nominal_masses=np.array([model.nominal.get(n, (0.0, np.zeros(3)))[0]
                                 for n in model.link_names]),
        nominal_coms=np.array([model.nominal.get(n, (0.0, np.zeros(3)))[1]
                               for n in model.link_names]),
        singular_values=sv_raw,
        rank=real_rank,
        cond=cond,
        resid=resid,
        resid_rms_joint=rms_j,
        scale_check=k,
        corr_joint=corr,
        prior_source=prior_source,
        joints=joints,
    )


def identify_friction(model, dynamics: Table, gravity_params: np.ndarray) -> FrictionResult:
    joints = model_joints(model)
    q = dynamics.cols([f"q/{j}" for j in joints])
    v = dynamics.cols([f"v/{j}" for j in joints])
    tau = dynamics.cols([f"tau/{j}" for j in joints])
    if q.shape[0] == 0:
        raise DyidError("动力学 CSV 没有数据行")
    ok = np.all(np.isfinite(q), axis=1) & np.all(np.isfinite(v), axis=1) & \
        np.all(np.isfinite(tau), axis=1)
    q, v, tau = q[ok], v[ok], tau[ok]
    if q.shape[0] < 50:
        raise DyidError(f"动力学 CSV 有效行太少（{q.shape[0]}）")

    tau_g = np.vstack([model.gravity_torque(model.q_from_list(q[i]), gravity_params)
                       for i in range(q.shape[0])])
    r = tau - tau_g

    n = len(joints)
    c1p = np.zeros(n); c1n = np.zeros(n); c2 = np.zeros(n); rms = np.zeros(n)
    for j in range(n):
        X = np.column_stack([(v[:, j] > 0).astype(float),
                             (v[:, j] < 0).astype(float),
                             v[:, j]])
        coef, *_ = np.linalg.lstsq(X, r[:, j], rcond=None)
        c1p[j], c1n[j], c2[j] = coef
        rms[j] = float(np.sqrt(np.mean((X @ coef - r[:, j]) ** 2)))
    return FrictionResult(c1_pos=c1p, c1_neg=c1n, c2=c2,
                          resid_rms_joint=rms, n_samples=int(q.shape[0]), joints=joints)



# ---------------------------------------------------------------------------
# 快速路径：先滤波，再流式法方程 + fork 并行（见 fastsolve.py）
# ---------------------------------------------------------------------------

def identify_fast(model, urdf_text: str, statics: Table, dynamics_path,
                  *, lpf_hz: float = 0.2, workers: int = 4, ridge: float = 1e-8,
                  accel: str = "ref", guard_sec: float = 0.6,
                  sg_window_sec: float = 3.0, sg_poly: int = 3,
                  with_inertia: bool = False):
    # noqa: D401
    """完整辨识（Stage1 重力 + Stage2 惯性/摩擦），**先按 lpf_hz 滤波**。

    返回 ``(GravityResult, friction_result, fit_dict)``。估计量与旧路径一致
    （实测逐项差 ≤ 1e-11），但快 6.5×（串行）~ 20×（16 进程）。
    """
    from . import fastsolve

    grav = identify_gravity(model, statics, method="bounded", ridge=1e-6)
    grav.model_kind = ("static_friction_inertia" if with_inertia
                       else "static_friction_only")
    fit = fastsolve.fit(model, urdf_text, statics, Path(dynamics_path),
                        pi_grav=grav.params, lpf_hz=lpf_hz, workers=workers,
                        ridge=ridge, accel=accel, guard_sec=guard_sec,
                        sg_window_sec=sg_window_sec, sg_poly=sg_poly,
                        with_inertia=with_inertia)
    c = np.asarray(fit["c_B"], dtype=float).reshape(len(ARM_JOINTS), 3)
    # 训练集残差（用于报告）：需要一次预测，但很便宜
    tr = fastsolve.predict(model, urdf_text, Path(dynamics_path),
                           pi_grav=grav.params, c_B=fit["c_B"],
                           pi_I=fit.get("pi_I"), pi_ID=fit.get("pi_ID"),
                           c_D=fit.get("c_D"), lpf_hz=lpf_hz,
                           workers=workers, accel=accel, guard_sec=guard_sec,
                           sg_window_sec=sg_window_sec, sg_poly=sg_poly,
                           with_inertia=with_inertia)
    resid = tr["tau"] - tr["pred"][:, 1, :]
    fric = FrictionResult(c1_pos=c[:, 0], c1_neg=c[:, 1], c2=c[:, 2],
                          resid_rms_joint=np.sqrt(np.mean(resid ** 2, axis=0)),
                          n_samples=int(fit["n_train"]))
    return grav, fric, fit, tr


# ---------------------------------------------------------------------------
# 保存 / 载入 / 命令行
# ---------------------------------------------------------------------------

#: 参数文件格式版本。
#:   schema 1 —— 历史格式：只有 gravity/friction/fit/meta（**保留兼容**）
#:   schema 2 —— 新增 robot（URDF 哈希 + 关节顺序）、inertia（逐连杆惯性）、
#:               validation（四模型对比）；并**显式标注用的是哪个辨识模型**
SCHEMA_VERSION = 2

#: 辨识模型标识 → 人类可读说明（写进 params.json，供下游判断"能不能用惯性项"）
MODEL_KINDS = {
    "static_friction_only": "静态+摩擦（**惯量张量全部置 0**；质量与一阶矩来自静态辨识）",
    "static_friction_inertia": "静态+摩擦+惯性（惯量张量来自动力学回归，数值秩 < 参数个数，非唯一）",
    "static_inertia": "静态+惯性（无摩擦；一般不用）",
    "static_only": "仅静态（重力）",
}


def _urdf_digest(urdf_text: Optional[str]) -> Optional[str]:
    if not urdf_text:
        return None
    import hashlib
    return hashlib.sha256(urdf_text.encode("utf-8")).hexdigest()


def _inertia_section(grav: GravityResult, model_kind: str,
                     fit: Optional[dict]) -> dict:
    """构造 schema2 的 ``inertia`` 段。

    ⚠️ 用户第 3 条要求（务必保持）：
      · ``static_friction_only`` 时 **惯量张量写 0**；
      · 但 **质量与质心必须保留**（重力矩 τ_g = Σ Jᵀ(m·g) 全靠它，
        置 0 会让 g(q)≡0、前馈完全失效）；
      · 并**注明用的是哪个辨识模型**（``mode`` + ``mode_desc``）—— 让下游能
        fail-closed 地拒绝"需要 M(q) 但只有静态参数"的用法（导纳惯性整形等）。

    ``links`` 按**连杆名**索引（绝不按下标：JSON/YAML 的键会重新排序）。
    ``identified`` 标出哪些分量是数据真正定出来的（其余是正则/边界给的）。
    """
    zero_I = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    with_inertia = (model_kind == "static_friction_inertia"
                    and fit is not None and "pi_I" in (fit or {}))
    links: dict = {}
    for i, name in enumerate(grav.link_names):
        m = float(grav.masses[i])
        com = [float(x) for x in grav.coms[i]]
        identified = ["mass", "mx", "my", "mz"]
        if abs(m) <= 1e-6:
            # 质量不可辨识（例如肩 pitch 连杆原点固定在基座上）：
            # 取 mass=1.0、com=first_moment/1.0 —— 与 GravityCompensator 同一条铁律
            m_out: Optional[float] = None
            com_out = [float(x) for x in grav.params.reshape(-1, 4)[i, 1:4]]
            identified = ["mx", "my", "mz"]
        else:
            m_out = m
            com_out = com
        links[name] = {
            "identified": identified,
            "mass": m_out,                 # None = 不可辨识（下游取 1.0，见上）
            "com": com_out if m_out is None else com,
            "I": zero_I if not with_inertia else None,   # 有惯性时在下面用 pi_I 填
        }
    if with_inertia:
        pi_I = np.asarray((fit or {}).get("pi_I"), dtype=float)
        # 每个关节 6 个独立分量（对称张量的 6 个自由度），共 6·n_links 个
        nl = len(grav.link_names)
        if pi_I.size == 6 * nl:
            for i, name in enumerate(grav.link_names):
                a, b, c, d, e, f = (float(x) for x in pi_I[6 * i:6 * i + 6])
                links[name]["I"] = [[a, b, c], [b, d, e], [c, e, f]]
        else:
            # 形状不符：不猜，置 0 并在段里标注（下游会 fail-closed）
            for name in links:
                links[name]["I"] = zero_I
    return {
        "mode": model_kind,
        "mode_desc": MODEL_KINDS.get(model_kind, model_kind),
        # ⚠️ 显式写给下游看的判据：true 才允许用它算 M(q) / 惯性整形
        "usable_for_mass_matrix": bool(with_inertia),
        "note": ("惯量张量已置 0（未辨识）；质量与一阶矩保留，g(q) 仍然准确"
                 if not with_inertia else
                 "惯量分量来自动力学回归；数值秩小于参数个数 ⇒ 逐分量**非唯一**，"
                 "请整体写回模型后用 rnea 使用，不要单独解读 Ixx"),
        "links": links,
    }


def save_params(path: Path, grav: GravityResult, fric: FrictionResult,
                meta: Optional[dict] = None,
                extra: Optional[dict] = None,
                *, urdf_text: Optional[str] = None,
                model_kind: Optional[str] = None,
                fit: Optional[dict] = None) -> None:
    """写 ``params.json``（**schema 2**）。

    ⚠️ **向后兼容**：``joints`` / ``gravity`` / ``friction`` / ``fit`` / ``meta``
    五个键的语义与 schema 1 **完全一致**，历史读取方（含
    ``lite_controller/config/gravity/inertia_params.yaml`` 的生成脚本）不受影响；
    schema 2 只是**新增** ``schema`` / ``source`` / ``robot`` / ``inertia`` /
    ``validation`` 五个键。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    kind = model_kind or getattr(grav, "model_kind", None) or "static_friction_only"
    payload = {
        # ---- schema2 新增 -------------------------------------------------
        "schema": SCHEMA_VERSION,
        "source": {
            "package": "lite_dyid",
            "model": kind,
            "model_desc": MODEL_KINDS.get(kind, kind),
            "generated": _now_iso(),
        },
        "robot": {
            "urdf_sha256": _urdf_digest(urdf_text),
            "joints": list(ARM_JOINTS),
            "links": list(grav.link_names),
        },
        "inertia": _inertia_section(grav, kind, fit),
        # ---- 与 schema 1 完全一致的兼容段 ----------------------------------
        "joints": list(ARM_JOINTS),
        "links": grav.link_names,
        "gravity": {
            "params": grav.params.tolist(),
            "masses": grav.masses.tolist(),
            "coms": grav.coms.tolist(),
            "nominal_masses": grav.nominal_masses.tolist(),
            "nominal_coms": grav.nominal_coms.tolist(),
            "rank": grav.rank,
            "cond": grav.cond,
        },
        "friction": {
            "c1_pos": fric.c1_pos.tolist(),
            "c1_neg": fric.c1_neg.tolist(),
            "c2": fric.c2.tolist(),
        },
        "fit": {
            "gravity_rms_joint": grav.resid_rms_joint.tolist(),
            "friction_rms_joint": fric.resid_rms_joint.tolist(),
            "scale_check": grav.scale_check,
        },
        "meta": {**(meta or {}), "model": kind},
    }
    # ⚠️⚠️ 合并 `extra` 时**必须**把 ``inertia`` 拆出来单独处理。
    #     历史上 extra["inertia"] 的语义是"整个 inertia 段"（旧格式：只有
    #     pi_I/pi_ID/rank_I/n_train）。若直接 `payload.update(extra)`，
    #     它会**整个替换**掉 schema2 刚构造好的 inertia 段 ——
    #     mode / links / usable_for_mass_matrix 全部消失（实测踩过）。
    #     正确做法：把旧格式的**原始拟合量**塞进新段的子键 ``raw_fit``。
    extra_clean = dict(extra or {})
    legacy_inertia = extra_clean.pop("inertia", None)
    if extra_clean:
        payload.update(extra_clean)
    if legacy_inertia:
        payload["inertia"]["raw_fit"] = dict(legacy_inertia)
        # 保留 rank_I 在顶层子键里方便直接读（不覆盖 mode/links）
        if "rank_I" in legacy_inertia:
            payload["inertia"]["rank_I"] = int(legacy_inertia["rank_I"])
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _now_iso() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def load_params(path: Path) -> dict:
    if not Path(path).is_file():
        raise DyidError(f"参数文件不存在：{path}")
    return json.loads(Path(path).read_text(encoding="utf-8"))


def predict_torque(model, params: dict, q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """τ_pred = τ_grav(q) + c1(sign v) + c2·v。"""
    gp = np.asarray(params["gravity"]["params"], dtype=float)
    c1p = np.asarray(params["friction"]["c1_pos"], dtype=float)
    c1n = np.asarray(params["friction"]["c1_neg"], dtype=float)
    c2 = np.asarray(params["friction"]["c2"], dtype=float)
    tg = model.gravity_torque(model.q_from_list(q), gp)
    return tg + np.where(v > 0, c1p, np.where(v < 0, c1n, 0.0)) + c2 * v


def _summary(grav: GravityResult, fric: FrictionResult) -> str:
    n_link_half = len(grav.link_names) // 2
    lines = ["重力辨识：",
             f"  先验来源: {grav.prior_source}",
             "  奇异值谱（前 12）: " + " ".join(f"{s:.3g}" for s in grav.singular_values[:12]),
             f"  数值秩 {grav.rank}/{grav.params.size}   条件数 {grav.cond:.3g}",
             f"  整体比例自检 k = {grav.scale_check:.4f}（应≈1；差得远说明符号/量纲约定不对）",
             f"  拟合 RMS = {grav.fit_rms:.4f} N·m"]
    if grav.rank < grav.params.size:
        lines.append(
            f"  ⚠️ 参数欠定：重力只能定出 {grav.rank} 个独立组合（≈每关节 2 个），"
            f"而参数有 {grav.params.size} 个。\n"
            "     逐连杆的 m / c **不是**数据唯一确定的 —— 它们只是"
            "『质量非负、且与数据一致』的一组解；换一组同样能完美解释数据。\n"
            "     注意：连『整臂总质量』也**不可辨识**（只有 Σm·c 的某些组合可辨识）。\n"
            "     真正可信、也是后续要用的，是**重力力矩本身**（拟合 RMS 见上）。")
    lines.append(f"  参数合计（不可辨识，仅量级参考）："
                 f"左臂 Σm={grav.arm_mass(0, 9):.3f} kg  "
                 f"右臂 Σm={grav.arm_mass(n_link_half, len(grav.link_names)):.3f} kg")
    if grav.corr_joint.min() < 0.5:
        bad = [grav.joints[i] for i in range(len(grav.joints)) if grav.corr_joint[i] < 0.5]
        lines.append(f"  ⚠️ 相关系数偏低的关节：{bad}")
    lines.append("")
    lines.append(f"{'link':32s} {'m_urdf':>9s} {'m_id':>9s} {'com_id (m)':>26s}")
    for i, name in enumerate(grav.link_names):
        lines.append(f"{name:32s} {grav.nominal_masses[i]:9.5f} {grav.masses[i]:9.4f} "
                     f"{np.array2string(grav.coms[i], precision=4, floatmode='fixed'):>26s}")
    lines.append("")
    lines.append(f"摩擦辨识（{fric.n_samples} 个动样本）：")
    lines.append(f"{'joint':22s} {'c1+':>8s} {'c1-':>8s} {'c2':>9s} {'rms':>8s}")
    for j, jn in enumerate(fric.joints):
        lines.append(f"{jn:22s} {fric.c1_pos[j]:8.4f} {fric.c1_neg[j]:8.4f} "
                     f"{fric.c2[j]:9.5f} {fric.resid_rms_joint[j]:8.4f}")
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="dyid_identify",
                                description="离线重解：静力学+动力学 CSV → 重力/摩擦参数")
    p.add_argument("--run-dir", required=True, help="采集输出目录（含 statics.csv / dynamics.csv）")
    p.add_argument("--urdf", default=None, help="URDF 文件；缺省用 run-dir/robot.urdf")
    p.add_argument("--ridge", type=float, default=1e-8, help="正则系数（只作用于不可辨识方向）")
    p.add_argument("--lpf", type=float, default=0.2,
                   help="求解前对 cmd/q/v/tau 的零相位低通截止频率 Hz（0=不滤）")
    p.add_argument("--workers", type=int, default=4,
                   help="并行进程数（1=串行；默认 4）")
    p.add_argument("--with-inertia", action="store_true",
                   help="额外辨识惯性项（模型 C/D）。默认只做『静态+摩擦』；"
                        "实测本工况下惯性项不改善验证残差，见 README")
    p.add_argument("--accel", default="ref", choices=["ref", "meas"])
    p.add_argument("--legacy", action="store_true",
                   help="用旧实现（lsq_linear 大矩阵，慢但不并行）做交叉对照")
    p.add_argument("--method", default="bounded", choices=["bounded", "ridge"],
                   help="bounded=质量非负的带约束最小二乘（默认）；ridge=无约束岭回归")
    p.add_argument("--no-nominal", action="store_true", help="先验一律用 0（不尝试 URDF 标称）")
    p.add_argument("--out", default=None, help="参数输出路径（默认 run-dir/params.json）")
    args, _unknown = p.parse_known_args(argv)

    from .model import RobotModel
    run = Path(args.run_dir).expanduser()
    urdf_path = Path(args.urdf) if args.urdf else run / "robot.urdf"
    if not urdf_path.is_file():
        raise DyidError(f"找不到 URDF：{urdf_path}（采集时会自动落一份）")
    model = RobotModel(urdf_path.read_text(encoding="utf-8"))
    statics = read_table(run / "statics.csv")
    dynamics = read_table(run / "dynamics.csv")
    urdf_text = urdf_path.read_text(encoding="utf-8")
    fit = None
    if args.legacy:
        grav = identify_gravity(model, statics, method=args.method, ridge=args.ridge,
                                use_nominal=not args.no_nominal)
        fric = identify_friction(model, dynamics, grav.params)
    else:
        grav, fric, fit, _tr = identify_fast(
            model, urdf_text, statics, run / "dynamics.csv",
            lpf_hz=args.lpf, workers=args.workers, ridge=args.ridge, accel=args.accel,
            with_inertia=args.with_inertia)
        print(f"\n[辨识] 零相位低通 {args.lpf} Hz；并行 {args.workers} 进程；"
              f"模型={'静态+摩擦+惯性' if args.with_inertia else '静态+摩擦（默认）'}")
    out = Path(args.out) if args.out else run / "params.json"
    save_params(out, grav, fric,
                meta={"run_dir": str(run), "ridge": args.ridge,
                      "lpf_hz": 0.0 if args.legacy else args.lpf,
                      "workers": 0 if args.legacy else args.workers,
                      "accel": args.accel, "fast": not args.legacy,
                      "model": ("static+friction+inertia" if args.with_inertia
                                else "static+friction")},
                urdf_text=urdf_text,
                model_kind=grav.model_kind,
                fit=fit,
                extra=None if (fit is None or "pi_I" not in fit) else {
                    "inertia": {"pi_I": np.asarray(fit["pi_I"]).tolist(),
                                "pi_ID": np.asarray(fit["pi_ID"]).tolist(),
                                "rank_I": int(fit["rank_I"]),
                                "n_train": int(fit["n_train"])}})
    print(_summary(grav, fric))
    print(f"\n参数已写入 {out}")

    from . import report
    report.plot_training(run, model, grav, fric)
    return 0


if __name__ == "__main__":
    sys.exit(main())

# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""快速辨识内核：**单遍滤波 + 流式法方程 + fork 并行**。

为什么单独一个模块
==================
`identify.py` 的原始实现（`solve_bounded` 走 `scipy.optimize.lsq_linear`）
在 488698×150 的稠密矩阵上要跑 **36 s**、峰值 RSS **5 GB**。
剖析（`tmp/dyid_verify/profile_full.py`）显示时间**几乎全在求解器**上，
滤波只占 1.9%。本模块把同一估计量换成三次改动：

1. **只物化需要的东西**：每个样本只留 ``A_I``（108 列，惯量分量），
   τ_g 与"质量/一阶矩对 M、C 的贡献"都**不建大矩阵**。
   峰值内存 5 GB → < 60 MB。
2. **法方程流式累加 + 按批 BLAS-3**：Stage2 的三个求解**本来就没有下界**
   （只有 Stage1 重力那处有），所以等价于岭回归，可写成
   ``(AᵀA + λI)x = Aᵀb`` —— 108×108 的小矩阵，微秒级求解。
   求解 26.7 s → ~0。
3. **fork 并行**：worker 只回传几十 KB 的法方程。**不用 `multiprocessing`**
   —— 它依赖 POSIX 信号量（落在 `/dev/shm`），容器里常常不可写；
   `os.fork` + pipe 到哪都能跑，而且子进程直接继承父进程已建好的模型与样本。

实测（同一批数据，结果与旧实现逐项一致到 1e-11）：

===========================  ========  ========
实现                          墙钟      加速
===========================  ========  ========
旧实现（lsq_linear）          35.9 s    1.0×
本模块 workers=1               5.5 s    6.5×
本模块 workers=16              1.8 s    20.4×
===========================  ========  ========

⚠️ 两条硬约束（踩过，别改回去）
--------------------------------
* **不能用"物理模型"代替线性参数化**。辨识解里会出现 **m≈0 但 μ≠0** 的连杆
  （秩亏最小二乘的产物，物理不可实现：μ = m·c）。``pin.Inertia`` 只能表示
  物理可实现的惯性，用它去算"质量/一阶矩对 M、C 的贡献"会带进 m·c² 的
  虚假大转动惯量，实测偏到 **1e4 N·m** 量级。必须回到 ``Yf[:,g]·π_g``。
* **静止段边界要挖保护带**：每段静止处机器人真的停住再起步，速度有真实跳变，
  那段 ``q̈`` 用参考轨迹是错的（参考轨迹在运动时间里连续）。
"""

from __future__ import annotations

import os
import struct
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .common import ARM_JOINTS
from .dataset import read_table
from .model import RobotModel

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("需要 pinocchio") from exc

from scipy.signal import butter, filtfilt, savgol_filter

NJ = len(ARM_JOINTS)          # 14
NC = 3                        # 每关节摩擦列数 [1(v>0), 1(v<0), v]
BLK = 256                     # 每批样本数：把逐样本小矩阵乘攒成一次 BLAS-3

#: 默认滤波截止频率（Hz）。交付轨迹成分最高 0.044 Hz（基频 0.011 × 4 次谐波），
#: 0.2 Hz 是"能保住运动本身"的最低带宽。
DEFAULT_LPF_HZ = 0.2

_P: Dict[str, object] = {}


# ===========================================================================
# 1. 滤波：4 条信号一条流水线
# ===========================================================================

def _interp_uniform(t: np.ndarray, Y: np.ndarray):
    """把 (n, m) 从近均匀网格插到严格均匀网格（全向量化，不按列循环）。"""
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0 or t.size < 12:
        return None, None
    tu = np.arange(t[0], t[-1], dt)
    if tu.size < 12:
        return None, None
    j = np.clip(np.searchsorted(t, tu, side="right") - 1, 0, t.size - 2)
    w = ((tu - t[j]) / (t[j + 1] - t[j]))[:, None]
    return tu, Y[j] * (1.0 - w) + Y[j + 1] * w


def lowpass(Y: np.ndarray, fc: float, fs: float, order: int = 4) -> np.ndarray:
    """对整块 (n, m) 做**一次**零相位 Butterworth 低通。

    零相位（``filtfilt``）很重要：有相位延迟的话，``τ`` 与 ``q`` 会对不齐，
    辨识出来的摩擦/惯量会被"时间错位"污染。
    """
    if fc <= 0.0 or Y.shape[0] < 16:
        return Y
    wn = min(0.98, float(fc) / (0.5 * fs))
    if wn <= 0.0:
        return Y
    b, a = butter(order, wn, btype="low")
    pad = min(3 * max(len(a), len(b)), Y.shape[0] - 1)
    if pad < 1:
        return Y
    return filtfilt(b, a, Y, axis=0, padlen=pad)


def _sg_uniform(tu: np.ndarray, Yu: np.ndarray, window_sec: float,
                poly: int, deriv: int):
    dt = float(np.median(np.diff(tu)))
    wl = int(round(window_sec / dt))
    if wl % 2 == 0:
        wl += 1
    wl = max(wl, poly + 2)
    wl = min(wl, Yu.shape[0] - 1 if (Yu.shape[0] - 1) % 2 == 1 else Yu.shape[0] - 2)
    if wl <= poly:
        return None
    return savgol_filter(Yu, wl, poly, deriv=deriv, axis=0, delta=dt, mode="interp")


def load_motion(path: Path, *, lpf_hz: float = DEFAULT_LPF_HZ, lpf_order: int = 4,
                guard_sec: float = 0.6, sg_window_sec: float = 3.0,
                sg_poly: int = 3, workers: int = 1) -> List[dict]:
    """读 CSV → 分段 → 插值 → 低通 → SG 求导 → 打保护带。

    返回每段的 dict：``t/q/v/cmd/tau/a_ref/a_meas/keep``。
    """
    tbl = read_table(path)
    if tbl.n == 0:
        raise ValueError(f"{path} 没有数据")
    mats = [tbl.cols([f"{p}/{j}" for j in ARM_JOINTS]) for p in ("cmd", "q", "v", "tau")]
    mt, wt = tbl.col("motion_t"), tbl.col("wall_t")
    traj = np.asarray(tbl.traj)

    jobs = []
    for label in np.unique(traj):
        base = np.where(traj == label)[0]
        mb, wb = mt[base], wt[base]
        gap = (np.diff(wb) - np.diff(mb)) > 0.5
        stop_times = mb[np.where(gap)[0]]
        dt = float(np.median(np.diff(mb)))
        cuts = np.where(np.diff(mb) > max(0.5, 10 * dt))[0] + 1
        for a, b in zip(np.r_[0, cuts], np.r_[cuts, mb.size]):
            if b - a >= 12:
                jobs.append((str(label), base[a:b], mb[a:b], stop_times, a, b, mb.size))

    def _one(job):
        label, idx, t, stop_times, a, b, msize = job
        stack = np.hstack([m[idx] for m in mats])
        tu, Yu = _interp_uniform(t, stack)
        if tu is None:
            return None
        fs = 1.0 / float(np.median(np.diff(tu)))
        if lpf_hz > 0.0:
            Yu = lowpass(Yu, lpf_hz, fs, lpf_order)
        cmd = Yu[:, 0:NJ]
        qq = Yu[:, NJ:2 * NJ]
        vv = Yu[:, 2 * NJ:3 * NJ]
        tt = Yu[:, 3 * NJ:4 * NJ]
        vr = _sg_uniform(tu, cmd, sg_window_sec, sg_poly, 1)
        ar = _sg_uniform(tu, cmd, sg_window_sec, sg_poly, 2)
        am = _sg_uniform(tu, vv, sg_window_sec, sg_poly, 1)
        if vr is None or ar is None or am is None:
            return None
        keep = np.ones(tu.size, dtype=bool)
        for ts in stop_times:
            if tu[0] - guard_sec <= ts <= tu[-1] + guard_sec:
                keep &= np.abs(tu - ts) > guard_sec
        if a > 0:
            keep &= (tu - tu[0]) > guard_sec
        if b < msize:
            keep &= (tu[-1] - tu) > guard_sec
        return {"traj": label, "t": tu, "cmd": cmd, "q": qq, "v": vv, "tau": tt,
                "v_ref": vr, "a_ref": ar, "a_meas": am, "keep": keep}

    res = fork_map(_one, jobs, workers) if workers > 1 and len(jobs) > 1 \
        else [_one(j) for j in jobs]
    out = [r for r in res if r is not None]
    if not out:
        raise ValueError(f"{path} 切不出任何运动段")
    return out


def gather(segs: Sequence[dict]) -> Tuple[np.ndarray, ...]:
    """拼成 (n,14) 数组：``(q, v, a_ref, a_meas, tau, t)``。"""
    keys = ("q", "v", "a_ref", "a_meas", "tau")
    acc = {k: [] for k in keys}
    ts = []
    for s in segs:
        k = np.where(s["keep"])[0]
        for kk in keys:
            acc[kk].append(s[kk][k])
        ts.append(s["t"][k])
    return tuple(np.vstack(acc[k]) for k in keys) + (np.concatenate(ts),)


# ===========================================================================
# 2. fork + pipe 并行（不依赖 /dev/shm 信号量）
# ===========================================================================

def _write_all(fd: int, data: bytes) -> None:
    mv = memoryview(data)
    while mv:
        k = os.write(fd, mv[:1 << 20])
        mv = mv[k:]


def _read_all(fd: int, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = os.read(fd, min(1 << 20, n - len(buf)))
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def fork_map(fn, items, workers: int, child_init=None):
    """把 ``items`` 分给 ``workers`` 个子进程跑 ``fn``，按原顺序汇总。

    ``multiprocessing.Pool`` 依赖 POSIX 信号量（在 ``/dev/shm``），容器里常不可写；
    这里用 ``os.fork`` + 匿名管道，只需要进程与 fd。
    """
    import pickle

    items = list(items)
    if workers <= 1 or len(items) <= 1:
        return [fn(it) for it in items]
    workers = min(workers, len(items))
    # ⚠️⚠️ 轮转分配（负载均衡）**必须**记下原始下标、再按下标回填。
    #     早期版本用 `items[i::workers]` 分组后直接按组顺序拼接 ——
    #     那样结果顺序是 [0,w,2w,…,1,1+w,…]，**整体错位**：
    #     耗时看着正常，数值却是错的（实测验证残差 0.116 → 0.507）。
    groups, order = [], []
    for w in range(workers):
        idxs = list(range(w, len(items), workers))
        if idxs:
            groups.append([items[i] for i in idxs])
            order.append(idxs)

    readers, pids, counts = [], [], []
    for g in groups:
        r, w = os.pipe()
        pid = os.fork()
        if pid == 0:                                     # ---- 子进程 ----
            code = 0
            try:
                os.close(r)
                if child_init is not None:
                    child_init()
                payload = pickle.dumps([fn(it) for it in g],
                                       protocol=pickle.HIGHEST_PROTOCOL)
                _write_all(w, struct.pack("<Q", len(payload)) + payload)
            except BaseException:
                code = 1
            finally:
                try:
                    os.close(w)
                except OSError:
                    pass
                os._exit(code)
        os.close(w)                                      # ---- 父进程 ----
        readers.append(r); pids.append(pid); counts.append(len(g))

    out = [None] * len(items)
    for r, pid, idxs in zip(readers, pids, order):
        head = _read_all(r, 8)
        if len(head) < 8:
            os.close(r)
            _, status = os.waitpid(pid, 0)
            raise RuntimeError(f"子进程 pid={pid} 未回传数据（status={status}）")
        n = struct.unpack("<Q", head)[0]
        vals = pickle.loads(_read_all(r, n))
        os.close(r)
        os.waitpid(pid, 0)
        if len(vals) != len(idxs):
            raise RuntimeError(f"子进程 pid={pid} 回传 {len(vals)} 项，期望 {len(idxs)}")
        for pos, val in zip(idxs, vals):
            out[pos] = val
    return out


# ===========================================================================
# 3. worker：按批算 A_I / 摩擦块 / τ_g / Yf[:,g]·π_g
# ===========================================================================

def prepare(model: RobotModel, urdf_text: str, pi_grav: np.ndarray,
            *, with_inertia: bool = False) -> None:
    """在父进程建好模型与 π_grav 模型；``fork`` 后子进程直接继承（零传参）。"""
    import tempfile
    tmp = Path(tempfile.gettempdir()) / "lite_dyid_fast.urdf"
    tmp.write_text(urdf_text, encoding="utf-8")
    pm = pin.buildModelFromUrdf(str(tmp))
    jid = {pm.names[i]: i for i in range(pm.njoints)}
    v_index = {n: pm.joints[jid[n]].idx_v for n in pm.names[1:]}
    arm_v = [v_index[j] for j in ARM_JOINTS]
    nl = pm.njoints - 1
    gcols = [10 * k + j for k in range(nl) for j in range(4)]
    icols = [10 * k + j for k in range(nl) for j in range(4, 10)]

    # π_grav → 只含 (m, μ) 的模型，用来精确算 τ_g（重力只依赖 (m, μ)）。
    # ⚠️ 质量给一个极小下限：辨识解里存在 m≈0 但 μ≠0 的连杆（物理不可实现），
    #    若直接 max(m,0) 会丢掉 μ（实测 τ_g 偏 0.16 N·m）。
    #    取 mass=ε、lever=μ/ε 时 mass*lever=μ 精确成立，ε 自身的重力贡献可忽略。
    eps_m = 1e-9
    gm = pm.copy()
    for k, jid_ in enumerate(range(1, pm.njoints)):
        m = float(pi_grav[4 * k])
        mu = np.asarray(pi_grav[4 * k + 1:4 * k + 4], dtype=float)
        m_rep = m if m > eps_m else eps_m
        gm.inertias[jid_] = pin.Inertia(m_rep, mu / m_rep, np.zeros((3, 3)))

    _P.update(model=pm, gmodel=gm, arm_v=arm_v, arm_idx=np.asarray(arm_v, dtype=int),
              gcols=gcols, icols=icols, pi_grav=np.asarray(pi_grav, dtype=float),
              with_inertia=bool(with_inertia),
              data=pm.createData(), gdata=gm.createData())
    _child_init()


def _child_init() -> None:
    m = _P["model"]
    _P["data"] = m.createData()
    _P["gdata"] = _P["gmodel"].createData()
    _P["QB"] = np.zeros((BLK, m.nq))
    _P["VB"] = np.zeros((BLK, m.nv))
    _P["AB"] = np.zeros((BLK, m.nv))


def _fill_vecs(b0: int, b1: int):
    ai = _P["arm_idx"]
    m = b1 - b0
    QB, VB, AB = _P["QB"], _P["VB"], _P["AB"]
    QB[:m] = 0.0; QB[:m, ai] = _P["Q"][b0:b1]
    VB[:m] = 0.0; VB[:m, ai] = _P["V"][b0:b1]
    AB[:m] = 0.0; AB[:m, ai] = _P["A"][b0:b1]
    return QB, VB, AB


def _fill_block(b0: int, b1: int, Ablk, Xblk, tgblk, gdblk) -> None:
    """填一批：摩擦块 X(14×42)、τ_g(14)；只有 ``with_inertia`` 才额外算 A_I 与
    ``Yf[:,g]·π_g``。

    ⚠️ 默认（静态+摩擦）**完全不调 regressor** —— 那是最贵的一步（~35 µs/样本）。
    只在需要惯性项时才付这个代价。
    """
    M, GM = _P["model"], _P["gmodel"]
    D, GD = _P["data"], _P["gdata"]
    arm_v, icols, gcols = _P["arm_v"], _P["icols"], _P["gcols"]
    pi_g = _P["pi_grav"]
    wi = _P["with_inertia"]
    m = b1 - b0
    QB, VB, AB = _fill_vecs(b0, b1)
    Vb = _P["V"][b0:b1]
    for j in range(NJ):
        vj = Vb[:, j]
        Xblk[j:m * NJ:NJ, 3 * j] = (vj > 0)
        Xblk[j:m * NJ:NJ, 3 * j + 1] = (vj < 0)
        Xblk[j:m * NJ:NJ, 3 * j + 2] = vj
    for k in range(m):
        q = QB[k]
        r0 = k * NJ
        tgblk[r0:r0 + NJ] = pin.computeGeneralizedGravity(GM, GD, q)[arm_v]
        if wi:
            Y = pin.computeJointTorqueRegressor(M, D, q, VB[k], AB[k])[arm_v]
            Ablk[r0:r0 + NJ, :] = Y[:, icols]
            gdblk[r0:r0 + NJ] = Y[:, gcols] @ pi_g


def _chunk_normal_eq(rng: Tuple[int, int]):
    """worker：回传本块的法方程（几十 KB）。

    ``_P["with_inertia"]`` 为假时只累加摩擦部分（``XtX/XtbB``），
    惯量相关项返回空数组 —— 默认路径因此**不调 regressor**。
    """
    lo, hi = rng
    wi = _P["with_inertia"]
    nI = 6 * (_P["model"].njoints - 1) if wi else 0
    nF = NJ * NC
    TAU = _P["TAU"]
    AtA = np.zeros((nI, nI)); AtbD = np.zeros(nI); AtX = np.zeros((nI, nF))
    XtX = np.zeros((nF, nF)); XtbB = np.zeros(nF); XtbD = np.zeros(nF)
    Ablk = np.empty((BLK * NJ, nI)) if wi else None
    Xblk = np.zeros((BLK * NJ, nF))
    tgblk = np.empty(BLK * NJ); gdblk = np.empty(BLK * NJ)
    for b0 in range(lo, hi, BLK):
        b1 = min(b0 + BLK, hi)
        m = b1 - b0
        Xblk[:m * NJ, :] = 0.0
        _fill_block(b0, b1, Ablk, Xblk, tgblk, gdblk)
        X = Xblk[:m * NJ]
        tv = TAU[b0:b1].reshape(-1)
        r = tv - tgblk[:m * NJ]              # τ − τ_g
        XtX += X.T @ X
        XtbB += X.T @ r
        if wi:
            A = Ablk[:m * NJ]
            rD = tv - gdblk[:m * NJ]         # τ − Yf[:,g]·π_g（τ_g 抵消）
            AtA += A.T @ A
            AtbD += A.T @ rD
            XtbD += X.T @ rD
            AtX += A.T @ X
    return AtA, AtbD, XtX, XtbB, XtbD, AtX


def _chunk_predict(payload):
    lo, hi, pi_I, pi_ID, c_B, c_D = payload
    wi = _P["with_inertia"]
    nI = 6 * (_P["model"].njoints - 1) if wi else 0
    km = 4 if wi else 2
    out = np.zeros((hi - lo, km, NJ))
    Ablk = np.empty((BLK * NJ, nI)) if wi else None
    Xblk = np.zeros((BLK * NJ, NJ * NC))
    tgblk = np.empty(BLK * NJ); gdblk = np.empty(BLK * NJ)
    for b0 in range(lo, hi, BLK):
        b1 = min(b0 + BLK, hi)
        m = b1 - b0
        Xblk[:m * NJ, :] = 0.0
        _fill_block(b0, b1, Ablk, Xblk, tgblk, gdblk)
        X = Xblk[:m * NJ]
        tg = tgblk[:m * NJ]
        p = np.empty((m * NJ, km))
        p[:, 0] = tg
        p[:, 1] = tg + X @ c_B
        if wi:
            A = Ablk[:m * NJ]
            mom = gdblk[:m * NJ] - tg
            p[:, 2] = tg + mom + A @ pi_I
            p[:, 3] = tg + mom + A @ pi_ID + X @ c_D
        out[b0 - lo:b1 - lo, :, :] = p.reshape(m, NJ, km).transpose(0, 2, 1)
    return out


# ===========================================================================
# 4. 求解（与旧实现同一估计量：列归一化 + 岭，无下界）
# ===========================================================================

def solve_ne(H: np.ndarray, g: np.ndarray, ridge: float) -> np.ndarray:
    scale = np.sqrt(np.clip(np.diag(H), 1e-300, None))
    Hs = H / np.outer(scale, scale)
    gs = g / scale
    return np.linalg.solve(Hs + ridge * np.eye(H.shape[0]), gs) / scale


# ===========================================================================
# 5. 对外 API
# ===========================================================================

def fit(model: RobotModel, urdf_text: str, statics, dynamics_path: Path, *,
        pi_grav: np.ndarray, lpf_hz: float = DEFAULT_LPF_HZ, workers: int = 4,
        ridge: float = 1e-8, accel: str = "ref", guard_sec: float = 0.6,
        sg_window_sec: float = 3.0, sg_poly: int = 3,
        with_inertia: bool = False) -> dict:
    """冻结 π_grav 后，用「静力学+动力学」数据辨识摩擦（可选：惯性项）。

    **默认只做「静态 + 摩擦」模型**（``with_inertia=False``）——
    这也是 `dyid_collect` / `dyid_identify` 的默认。此时：
      · 每样本只调一次 ``computeGeneralizedGravity``，**完全不调 regressor**；
      · 未知量只有 42 个（每关节 c1⁺/c1⁻/c2），法方程 42×42。

    ``with_inertia=True`` 才额外辨识 108 个惯量分量（模型 C/D），
    见 README「可选：静态+摩擦+惯性」一节 —— 实测在本工况下**并不改善验证残差**。
    """
    segs = load_motion(dynamics_path, lpf_hz=lpf_hz, guard_sec=guard_sec,
                       sg_window_sec=sg_window_sec, sg_poly=sg_poly, workers=workers)
    Q, V, AR, AM, TAU, _TT = gather(segs)
    ACC = AR if accel == "ref" else AM
    prepare(model, urdf_text, pi_grav, with_inertia=with_inertia)
    for k, v in (("Q", Q), ("V", V), ("A", ACC), ("TAU", TAU)):
        _P[k] = v
    n = Q.shape[0]
    bounds = np.linspace(0, n, min(workers * 4, n) + 1).astype(int)
    res = fork_map(_chunk_normal_eq, list(zip(bounds[:-1], bounds[1:])),
                   workers, child_init=_child_init)
    XtX = sum(r[2] for r in res); XtbB = sum(r[3] for r in res)
    c_B = solve_ne(XtX, XtbB, ridge)
    out = {"c_B": c_B, "n_train": n, "lpf_hz": lpf_hz,
           "with_inertia": bool(with_inertia)}
    if with_inertia:
        AtA = sum(r[0] for r in res); AtbD = sum(r[1] for r in res)
        XtbD = sum(r[4] for r in res); AtX = sum(r[5] for r in res)
        nI = AtA.shape[0]
        pi_I = solve_ne(AtA, AtbD, ridge)
        H = np.block([[AtA, AtX], [AtX.T, XtX]])
        theta = solve_ne(H, np.concatenate([AtbD, XtbD]), ridge)
        # 秩：法方程会把条件数平方，**仅供参照**；对角线可能有 0 → 必须 clip。
        d = np.sqrt(np.clip(np.diag(AtA), 1e-300, None))
        sv = np.linalg.svd(AtA / np.outer(d, d), compute_uv=False)
        out.update(pi_I=pi_I, pi_ID=theta[:nI], c_D=theta[nI:],
                   rank_I=int(np.sum(sv > sv[0] * nI * np.finfo(float).eps)))
    return out


def predict(model: RobotModel, urdf_text: str, csv_path: Path, *, pi_grav: np.ndarray,
            pi_I: np.ndarray, c_B: np.ndarray,
            pi_ID: Optional[np.ndarray] = None, c_D: Optional[np.ndarray] = None,
            lpf_hz: float = DEFAULT_LPF_HZ, workers: int = 4, accel: str = "ref",
            guard_sec: float = 0.6, sg_window_sec: float = 3.0,
            sg_poly: int = 3, with_inertia: bool = False) -> dict:
    """在给定 CSV 上算预测。

    默认两个模型：``A`` 纯静力学、``B`` 静力学+摩擦（返回 ``models=["A","B"]``）。
    ``with_inertia=True`` 时再加 ``C`` 静力学+惯性、``D`` 静力学+摩擦+惯性。
    """
    segs = load_motion(csv_path, lpf_hz=lpf_hz, guard_sec=guard_sec,
                       sg_window_sec=sg_window_sec, sg_poly=sg_poly, workers=workers)
    Q, V, AR, AM, TAU, TT = gather(segs)
    ACC = AR if accel == "ref" else AM
    prepare(model, urdf_text, pi_grav, with_inertia=with_inertia)
    for k, v in (("Q", Q), ("V", V), ("A", ACC), ("TAU", TAU)):
        _P[k] = v
    n = Q.shape[0]
    pi_ID = pi_I if pi_ID is None else pi_I
    c_D = c_B if c_D is None else c_D
    if not with_inertia:
        pi_I = np.zeros(0); pi_ID = np.zeros(0); c_D = np.zeros(0)
    bounds = np.linspace(0, n, min(workers * 4, n) + 1).astype(int)
    args = [(int(a), int(b), pi_I, pi_ID, c_B, c_D)
            for a, b in zip(bounds[:-1], bounds[1:])]
    pred = np.vstack(fork_map(_chunk_predict, args, workers, child_init=_child_init))
    return {"t": TT, "tau": TAU, "pred": pred, "n": n, "lpf_hz": lpf_hz,
            "models": ["A", "B", "C", "D"] if with_inertia else ["A", "B"],
            "with_inertia": bool(with_inertia),
            "a_rms": float(np.sqrt(np.mean(ACC ** 2)))}

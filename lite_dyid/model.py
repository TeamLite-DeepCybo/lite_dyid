# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""URDF → pinocchio：模型、限位、碰撞检查、重力回归基。

设计要点
========
* **重力回归的 4 参数基**：重力力矩只对每根连杆的 ``(m, m·c)`` 线性
  （``c`` = 质心在连杆系下的位置）。于是每个连杆 4 个基向量：

      m  = 1, c = 0        →  -J_linkᵀ g
      m  = 1, c = e_a      →   系数矩阵的第 a 列（对 μ = m·c）

  用 pinocchio 直接算这 4 个基下的广义重力即可 —— 不需要手写旋转矩阵的导数，
  也不会因为解析推导出错。回归矩阵 ``Y(q)`` 的形状是 ``(nv, 4·n_links)``。

* **只取臂关节的行**：夹爪关节（本机型 mode:=arms 下不驱动）没有力矩测量，
  其行必须丢掉，否则会把 0 当成"测得 0 力矩"污染最小二乘。
"""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .common import ARM_JOINTS, DyidError

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - 环境问题
    raise DyidError(
        "需要 pinocchio（含 hppfcl）。安装：/usr/bin/python3 -m pip install pin"
    ) from exc


def package_dirs() -> List[str]:
    """URDF 里 ``package://`` 的解析目录（lite_description 的 share）。"""
    from ament_index_python.packages import get_package_share_directory

    dirs = []
    for pkg in ("lite_description", "lite_bringup"):
        try:
            dirs.append(get_package_share_directory(pkg))
        except Exception:  # pragma: no cover
            pass
    if not dirs:
        raise DyidError("找不到 lite_description 的 share 目录（先 source install/setup.bash）")
    return dirs


def _parse_limits(urdf_text: str) -> Dict[str, Tuple[float, float]]:
    limits: Dict[str, Tuple[float, float]] = {}
    root = ET.fromstring(urdf_text)
    for joint in root.findall("joint"):
        lim = joint.find("limit")
        name = joint.get("name")
        if lim is None or name is None:
            continue
        lo, hi = lim.get("lower"), lim.get("upper")
        if lo is None or hi is None:
            continue
        limits[name] = (float(lo), float(hi))
    return limits


def _parse_inertials(urdf_text: str) -> Dict[str, Tuple[float, np.ndarray]]:
    """link → (mass, com)。URDF 里 ``<inertial>`` 的 origin 就是质心位置。"""
    out: Dict[str, Tuple[float, np.ndarray]] = {}
    root = ET.fromstring(urdf_text)
    for link in root.findall("link"):
        ine = link.find("inertial")
        if ine is None:
            continue
        mass_el = ine.find("mass")
        org = ine.find("origin")
        if mass_el is None:
            continue
        mass = float(mass_el.get("value", "0"))
        com = np.zeros(3)
        if org is not None and org.get("xyz"):
            com = np.array([float(v) for v in org.get("xyz").split()])
        out[link.get("name", "")] = (mass, com)
    return out


@dataclass
class CollisionHit:
    first: str
    second: str
    depth: float


class RobotModel:
    """pinocchio 模型 + 限位 + 碰撞对 + 重力回归基。"""

    def __init__(self, urdf_text: str, package_dirs_: Optional[Sequence[str]] = None):
        self.urdf_text = urdf_text
        self.pkg_dirs = list(package_dirs_) if package_dirs_ else package_dirs()
        tmp = Path("/tmp") / "lite_dyid_model.urdf"
        tmp.write_text(urdf_text, encoding="utf-8")
        self.urdf_path = str(tmp)

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.limits = _parse_limits(urdf_text)
        self.nominal = _parse_inertials(urdf_text)

        self.joint_id = {self.model.names[i]: i for i in range(self.model.njoints)}
        self.v_index = {
            n: self.model.joints[self.joint_id[n]].idx_v for n in self.model.names[1:]
        }
        missing = [j for j in ARM_JOINTS if j not in self.v_index]
        if missing:
            raise DyidError(f"URDF 缺少臂关节：{missing}")
        self.arm_v = [self.v_index[j] for j in ARM_JOINTS]

        #: 连杆（关节）序号 1..njoints-1，其惯性参数参与回归
        self.link_joints = list(range(1, self.model.njoints))
        #: ⚠️ pinocchio 的 ``model.names[jid]`` 是**关节名**，而 URDF 的
        #:    ``<link>``/``<inertial>`` 用的是**连杆名**（本机型差一个 ``_link`` 后缀）。
        #:    真正的连杆名在 BODY frame 上，这里按 parentJoint 取第一个。
        #:    另外固定关节会被 pinocchio **合并**进父关节，所以标称参数
        #:    必须从 ``model.inertias[jid]`` 取（已含合并进来的子连杆），
        #:    而不是去 URDF 里逐个查 —— 后者会漏掉合并的部分。
        self.link_names = []
        for jid in self.link_joints:
            name = self.model.names[jid]
            for f in self.model.frames:
                if f.type == pin.FrameType.BODY and f.parentJoint == jid:
                    name = f.name
                    break
            self.link_names.append(name)
        self.n_params = 4 * len(self.link_joints)
        self.nominal = {}
        for jid, link in zip(self.link_joints, self.link_names):
            dyn = self.model.inertias[jid].toDynamicParameters()
            mass = float(dyn[0])
            com = (np.asarray(dyn[1:4], dtype=float) / mass) if mass > 1e-12 else np.zeros(3)
            self.nominal[link] = (mass, com)

        self._basis_models = self._build_basis_models()
        self._setup_collisions()

    # -- 基础 -------------------------------------------------------------

    def q_from_dict(self, values: Dict[str, float]) -> np.ndarray:
        q = pin.neutral(self.model)
        for name, val in values.items():
            idx = self.v_index.get(name)
            if idx is not None:
                q[idx] = float(val)
        return q

    def q_from_list(self, values: Sequence[float]) -> np.ndarray:
        return self.q_from_dict(dict(zip(ARM_JOINTS, values)))

    def limit_array(self) -> Tuple[np.ndarray, np.ndarray]:
        lo = np.array([self.limits[j][0] for j in ARM_JOINTS])
        hi = np.array([self.limits[j][1] for j in ARM_JOINTS])
        return lo, hi

    def nominal_arm_params(self) -> np.ndarray:
        """URDF 标称参数，按回归列顺序 [m, μx, μy, μz] × 连杆。

        取自 ``model.inertias``（已含固定关节合并进来的子连杆）。
        """
        p = np.zeros(self.n_params)
        for k, jid in enumerate(self.link_joints):
            dyn = self.model.inertias[jid].toDynamicParameters()
            p[4 * k] = float(dyn[0])
            p[4 * k + 1:4 * k + 4] = np.asarray(dyn[1:4], dtype=float)
        return p

    def param_labels(self) -> List[str]:
        labels = []
        for jid in self.link_joints:
            link = self.model.names[jid]
            labels += [f"m[{link}]", f"mx[{link}]", f"my[{link}]", f"mz[{link}]"]
        return labels

    # -- 重力回归 ---------------------------------------------------------

    def _build_basis_models(self) -> List["pin.Model"]:
        """每个 (连杆, 4 个基) 一个"只有该连杆有惯性"的模型。"""
        models = []
        zero = pin.Inertia.Zero()
        for jid in self.link_joints:
            for b in range(4):
                m = self.model.copy()
                for k in self.link_joints:
                    m.inertias[k] = zero
                if b == 0:
                    m.inertias[jid] = pin.Inertia(1.0, np.zeros(3), np.zeros((3, 3)))
                else:
                    lever = np.zeros(3)
                    lever[b - 1] = 1.0
                    m.inertias[jid] = pin.Inertia(1.0, lever, np.zeros((3, 3)))
                models.append((m, m.createData()))
        return models

    def gravity_regressor(self, q: np.ndarray) -> np.ndarray:
        """Y(q)：形状 ``(nv, 4·n_links)``，使 ``τ_gravity = Y(q) · π``。"""
        Y = np.zeros((self.model.nv, self.n_params))
        col = 0
        for m, d in self._basis_models:
            Y[:, col] = pin.computeGeneralizedGravity(m, d, q)
            col += 1
        return Y

    def gravity_torque(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        """由参数算重力力矩（只取臂关节行）。"""
        return (self.gravity_regressor(q) @ params)[self.arm_v]

    def fast_gravity_evaluator(self, params: np.ndarray):
        """返回与 :meth:`gravity_torque` 等价、适合实时预计算的函数。

        回归实现每次需要对每个参数基分别调用 Pinocchio。验证轨迹有数万拍，
        因此把辨识得到的 ``[m,mx,my,mz]`` 一次写入模型，再用一次
        ``computeGeneralizedGravity`` 求整机重力。转动惯量不参与重力项，置零不会
        改变结果。
        """
        p = np.asarray(params, dtype=float).reshape(-1)
        if p.shape != (self.n_params,) or not np.all(np.isfinite(p)):
            raise DyidError(f"重力参数形状/数值无效：{p.shape}，期望 ({self.n_params},)")
        gravity_model = self.model.copy()
        for k, jid in enumerate(self.link_joints):
            # gravity_regressor 的三个一阶矩基是 Inertia(1, e_axis, 0)，
            # 因而每列除了单位一阶矩也各自带一个单位质量。把这些列线性合并成
            # 单个模型时，等效质量必须是 p0+p1+p2+p3，一阶矩仍是 p1:p4。
            # 这不是常见的直接 [m,mx,my,mz] 打包；按常见方式写会与已保存参数
            # 的 predict_torque 相差约 0.2 N·m。
            chunk = p[4 * k:4 * k + 4]
            dynamic = np.zeros(10, dtype=float)
            dynamic[0] = float(np.sum(chunk))
            dynamic[1:4] = chunk[1:4]
            gravity_model.inertias[jid] = pin.Inertia.FromDynamicParameters(dynamic)
        gravity_data = gravity_model.createData()

        def evaluate(q: np.ndarray) -> np.ndarray:
            tau = pin.computeGeneralizedGravity(gravity_model, gravity_data, q)
            return np.asarray(tau, dtype=float)[self.arm_v].copy()

        return evaluate

    def arm_regressor(self, q: np.ndarray) -> np.ndarray:
        return self.gravity_regressor(q)[self.arm_v, :]

    # -- 碰撞 -------------------------------------------------------------

    def _setup_collisions(self) -> None:
        self.geom = pin.buildGeomFromUrdf(
            self.model, self.urdf_path, pin.GeometryType.COLLISION, self.pkg_dirs
        )
        parents = list(self.model.parents)
        n = 0
        for i, j in itertools.combinations(range(len(self.geom.geometryObjects)), 2):
            gi, gj = self.geom.geometryObjects[i], self.geom.geometryObjects[j]
            ji, jj = gi.parentJoint, gj.parentJoint
            if ji == jj:
                continue
            if parents[ji] == jj or parents[jj] == ji:
                continue                      # 父子：结构上贴合，必然"接触"
            if parents[ji] == parents[jj]:
                continue                      # 兄弟：例如夹爪两指，设计上接触
            if "gripper" in gi.name or "gripper" in gj.name:
                continue                      # 夹爪自身不参与
            self.geom.addCollisionPair(pin.CollisionPair(i, j))
            n += 1
        self.n_collision_pairs = n
        self._cdata = self.model.createData()
        self._gdata = self.geom.createData()

    def collisions(self, values: Dict[str, float]) -> List[CollisionHit]:
        q = self.q_from_dict(values)
        pin.computeCollisions(self.model, self._cdata, self.geom, self._gdata, q, False)
        hits = []
        for k, res in enumerate(self._gdata.collisionResults):
            if not res.isCollision():
                continue
            pair = self.geom.collisionPairs[k]
            depth = res.getPenetrationDepth() if hasattr(res, "getPenetrationDepth") else 0.0
            hits.append(CollisionHit(
                self.geom.geometryObjects[pair.first].name,
                self.geom.geometryObjects[pair.second].name,
                float(depth),
            ))
        return hits

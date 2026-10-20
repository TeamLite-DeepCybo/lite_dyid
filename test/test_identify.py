# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""辨识层的单测：用自造的小 URDF，**不依赖真机、也不依赖本机型 URDF**。"""

import numpy as np
import pytest

from lite_dyid.common import ARM_JOINTS, DyidError
from lite_dyid.dataset import dynamics_header, read_table, save_csv, statics_header
from lite_dyid.identify import identify_friction, identify_gravity, predict_torque

#: 两连杆 + 一个"臂"（用真实关节名，模型用不到 14 个关节也没关系：
#: RobotModel 只要求 URDF 里出现 ARM_JOINTS 中的**部分**名称即可 —— 见下）
TINY_URDF = """<?xml version="1.0"?>
<robot name="tiny">
  <link name="world_root"/>
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="world_root"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
  </joint>
  <link name="link1">
    <inertial><origin xyz="0.10 0 0"/><mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><origin xyz="0.10 0 0"/><geometry><cylinder radius="0.03" length="0.20"/></geometry></collision>
  </link>
  <joint name="left_elbow_pitch" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.20 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.00" upper="0.00" effort="10" velocity="2"/>
  </joint>
  <link name="link2">
    <inertial><origin xyz="0.08 0 0"/><mass value="0.8"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><origin xyz="0.08 0 0"/><geometry><cylinder radius="0.03" length="0.16"/></geometry></collision>
  </link>
</robot>
"""


class _TinyModel:
    """把 RobotModel 缩到"只有两个关节"的壳，避免测试依赖 14 关节 URDF。"""

    def __init__(self):
        import pinocchio as pin

        from pathlib import Path
        p = Path("/tmp/lite_dyid_tiny.urdf")
        p.write_text(TINY_URDF, encoding="utf-8")
        self.model = pin.buildModelFromUrdf(str(p))
        self.joint_id = {self.model.names[i]: i for i in range(self.model.njoints)}
        self.v_index = {n: self.model.joints[self.joint_id[n]].idx_v
                        for n in self.model.names[1:]}
        self.joints = list(self.v_index)
        self.link_names = list(self.joints)
        self.link_joints = list(range(1, self.model.njoints))
        self.arm_v = [self.v_index[name] for name in self.joints]
        self.n_params = 4 * len(self.joints)

    def q_from_list(self, vals):
        import pinocchio as pin
        q = pin.neutral(self.model)
        for name, val in zip(self.joints, vals):
            q[self.v_index[name]] = float(val)
        return q

    def nominal_arm_params(self):
        p = np.zeros(self.n_params)
        for k, n in enumerate(self.joints):
            dyn = self.model.inertias[self.joint_id[n]].toDynamicParameters()
            p[4 * k] = dyn[0]
            p[4 * k + 1:4 * k + 4] = dyn[1:4]
        return p

    def gravity_regressor(self, q):
        import pinocchio as pin
        Y = np.zeros((self.model.nv, self.n_params))
        col = 0
        zero = pin.Inertia.Zero()
        for jid in range(1, self.model.njoints):
            for b in range(4):
                m = self.model.copy()
                for k in range(1, self.model.njoints):
                    m.inertias[k] = zero
                if b == 0:
                    m.inertias[jid] = pin.Inertia(1.0, np.zeros(3), np.zeros((3, 3)))
                else:
                    lever = np.zeros(3); lever[b - 1] = 1.0
                    m.inertias[jid] = pin.Inertia(1.0, lever, np.zeros((3, 3)))
                Y[:, col] = pin.computeGeneralizedGravity(m, m.createData(), q)
                col += 1
        return Y

    def arm_regressor(self, q):
        # 小模型里"臂关节"就是全部关节，故与完整回归矩阵相同
        return self.gravity_regressor(q)

    def gravity_torque(self, q, params):
        return (self.gravity_regressor(q) @ params)[:len(self.joints)]

    @property
    def nominal(self):
        out = {}
        for name in self.joints:
            dyn = self.model.inertias[self.joint_id[name]].toDynamicParameters()
            mass = float(dyn[0])
            com = (np.asarray(dyn[1:4], float) / mass) if mass > 1e-12 else np.zeros(3)
            out[name] = (mass, com)
        return out


def _make_statics(model, n_poses=30, seed=0):
    rng = np.random.default_rng(seed)
    Q = np.column_stack([rng.uniform(-1.5, 1.5, n_poses), rng.uniform(-1.5, 0.0, n_poses)])
    pi = model.nominal_arm_params()
    A = np.vstack([model.gravity_regressor(model.q_from_list(Q[i])) for i in range(n_poses)])
    tau = (A @ pi).reshape(n_poses, -1)
    rows = np.column_stack([np.arange(n_poses), np.zeros(n_poses),
                            np.full(n_poses, 100), np.full(n_poses, 5.0),
                            Q, tau, np.zeros_like(Q), np.zeros_like(Q)])
    header = ["static_index", "motion_t", "n_samples", "duration_sec"] + \
             [f"q/{j}" for j in model.joints] + [f"tau/{j}" for j in model.joints] + \
             [f"q_std/{j}" for j in model.joints] + [f"tau_std/{j}" for j in model.joints]
    from lite_dyid.dataset import Table
    return Table(header, rows, ["tiny"] * n_poses), Q, tau


def test_gravity_recovers_torque_exactly():
    """重力力矩本身必须被完全复现（这是真正可辨识的量）。"""
    model = _TinyModel()
    table, _, _ = _make_statics(model)
    g = identify_gravity(model, table)
    assert g.fit_rms < 1e-6
    assert g.scale_check == pytest.approx(1.0, abs=1e-6)
    assert np.all(g.masses >= -1e-9), "带约束求解必须给出非负质量"


def test_gravity_bounded_beats_ridge_on_physicality():
    model = _TinyModel()
    table, _, _ = _make_statics(model)
    gb = identify_gravity(model, table, method="bounded")
    assert np.all(gb.masses >= -1e-9)
    with pytest.raises(DyidError):
        identify_gravity(model, table, method="nonsense")


def test_friction_recovers_known_coefficients():
    model = _TinyModel()
    table, Q, tau = _make_statics(model)
    g = identify_gravity(model, table)
    rng = np.random.default_rng(5)
    n = 400
    V = rng.uniform(-0.13, 0.13, (n, 2))
    Qd = np.column_stack([rng.uniform(-1.5, 1.5, n), rng.uniform(-1.5, 0.0, n)])
    c1p = np.array([0.30, 0.10]); c1n = np.array([-0.25, -0.08]); c2 = np.array([0.05, 0.02])
    A = np.vstack([model.gravity_regressor(model.q_from_list(Qd[i])) for i in range(n)])
    taud = (A @ g.params).reshape(n, 2) + np.where(V > 0, c1p, c1n) + V * c2
    header = ["motion_t", "wall_t"] + [f"cmd/{j}" for j in model.joints] + \
             [f"q/{j}" for j in model.joints] + [f"v/{j}" for j in model.joints] + \
             [f"tau/{j}" for j in model.joints]
    rows = np.column_stack([np.arange(n), np.arange(n) * 0.01, Qd, Qd, V, taud])
    from lite_dyid.dataset import Table
    f = identify_friction(model, Table(header, rows, ["tiny"] * n), g.params)
    assert np.allclose(f.c1_pos, c1p, atol=1e-6)
    assert np.allclose(f.c1_neg, c1n, atol=1e-6)
    assert np.allclose(f.c2, c2, atol=1e-6)


def test_predict_torque_matches_generator():
    model = _TinyModel()
    params = {
        "gravity": {"params": model.nominal_arm_params().tolist()},
        "friction": {"c1_pos": [0.1, 0.2], "c1_neg": [-0.1, -0.2], "c2": [0.3, 0.4]},
    }
    q = np.array([0.3, -0.7]); v = np.array([0.05, -0.05])
    pred = predict_torque(model, params, q, v)
    tg = model.gravity_torque(model.q_from_list(q), np.asarray(params["gravity"]["params"]))
    assert np.allclose(pred, tg + np.array([0.1, -0.2]) + np.array([0.3, 0.4]) * v)


def test_fast_gravity_evaluator_exactly_matches_saved_parameter_semantics():
    """实时前馈优化必须逐值复现原回归基，不能按常见参数格式误解旧结果。"""
    from lite_dyid.model import RobotModel

    model = _TinyModel()
    params = model.nominal_arm_params()
    fast = RobotModel.fast_gravity_evaluator(model, params)
    for values in (np.array([0.3, -0.7]), np.array([-1.1, -0.2]), np.zeros(2)):
        q = model.q_from_list(values)
        assert np.allclose(fast(q), model.gravity_torque(q, params), atol=1e-12)


def test_read_table_rejects_ragged_rows(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("traj,a,b\nx,1,2\ny,3\n", encoding="utf-8")
    with pytest.raises(DyidError):
        read_table(p)


def test_csv_roundtrip(tmp_path):
    p = tmp_path / "d.csv"
    data = np.arange(12, dtype=float).reshape(2, 6)
    header = ["motion_t", "wall_t", "cmd/a", "q/a", "v/a", "tau/a"]
    save_csv(p, header, data, traj_name="t1")
    t = read_table(p)
    assert t.header == header
    assert np.allclose(t.data, data)
    assert t.traj == ["t1", "t1"]
    _ = (dynamics_header(), statics_header(), ARM_JOINTS)

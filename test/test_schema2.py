# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""`params.json` schema 2 的单测：钉住"惯量置 0 但质量/质心保留 + 注明模型"。

这两条是**用户明确要求**的语义，且一旦搞错后果很重：
  · 惯量置 0 ⇒ 下游不能用它算 M(q)（必须在文件里标出来，让下游 fail-closed）；
  · 质量/质心**必须保留** ⇒ 否则 g(q)≡0，重力前馈完全失效。

不需要机器人、不需要 pinocchio：直接构造 dataclass 造一个假结果。
"""

import json

import numpy as np
import pytest

from lite_dyid.identify import (MODEL_KINDS, SCHEMA_VERSION, FrictionResult,
                                GravityResult, save_params)


def _fake_gravity(model_kind="static_friction_only", with_null_mass=True):
    n = 4
    link_names = ["linkA", "linkB", "linkC", "linkD"]
    params = np.zeros(4 * n)
    masses = np.array([0.5, 0.0, 1.5, 2.0])          # linkB 质量不可辨识
    coms = np.array([[0.1, 0.2, 0.3], [0.0, 0.0, 0.0],
                     [-0.1, 0.05, 0.2], [0.3, -0.2, 0.1]])
    for i in range(n):
        params[4 * i] = masses[i]
        params[4 * i + 1:4 * i + 4] = coms[i] * masses[i]
    return GravityResult(
        link_names=link_names, params=params, masses=masses, coms=coms,
        nominal_masses=np.zeros(n), nominal_coms=np.zeros((n, 3)),
        singular_values=np.ones(4 * n), rank=2 * n, cond=10.0,
        resid=np.zeros(4), resid_rms_joint=np.zeros(14),
        scale_check=1.0, corr_joint=np.ones(14),
        model_kind=model_kind,
    )


def _fake_friction():
    n = 14
    return FrictionResult(c1_pos=np.zeros(n), c1_neg=np.zeros(n), c2=np.zeros(n),
                          resid_rms_joint=np.zeros(n), n_samples=100)


def _write(tmp_path, **kw):
    out = tmp_path / "params.json"
    save_params(out, _fake_gravity(kw.pop("model_kind", "static_friction_only")),
                _fake_friction(), urdf_text="<robot/>", **kw)
    return json.loads(out.read_text(encoding="utf-8"))


def test_schema_version_is_two(tmp_path):
    d = _write(tmp_path)
    assert d["schema"] == SCHEMA_VERSION == 2


def test_model_kind_is_recorded(tmp_path):
    d = _write(tmp_path)
    assert d["inertia"]["mode"] == "static_friction_only"
    assert "static_friction_only" in MODEL_KINDS
    assert d["inertia"]["mode_desc"] == MODEL_KINDS["static_friction_only"]
    assert d["source"]["model"] == "static_friction_only"
    assert d["meta"]["model"] == "static_friction_only"


def test_static_friction_zeroes_inertia_tensor(tmp_path):
    """⚠️ 用户要求：静态+摩擦时惯量张量**全部写 0**。"""
    d = _write(tmp_path)
    assert d["inertia"]["usable_for_mass_matrix"] is False
    for name, ln in d["inertia"]["links"].items():
        assert ln["I"] == [[0.0] * 3] * 3, name


def test_static_friction_keeps_mass_and_com(tmp_path):
    """⚠️ 用户要求：质量与质心**必须保留**（否则 g(q)≡0，前馈全失效）。"""
    d = _write(tmp_path)
    links = d["inertia"]["links"]
    assert links["linkA"]["mass"] == pytest.approx(0.5)
    assert links["linkA"]["com"] == pytest.approx([0.1, 0.2, 0.3])
    # 兼容段也要在（历史读取方靠它算重力）
    assert d["gravity"]["params"]
    assert d["gravity"]["masses"][0] == pytest.approx(0.5)


def test_unidentifiable_mass_is_null_and_com_from_first_moment(tmp_path):
    """质量不可辨识 ⇒ mass=null，com = 一阶矩/1.0（与 GravityCompensator 同一条铁律）。"""
    d = _write(tmp_path)
    b = d["inertia"]["links"]["linkB"]
    assert b["mass"] is None
    assert b["identified"] == ["mx", "my", "mz"]
    # 一阶矩全 0 ⇒ com 也全 0；关键是它**不参与** mass 的解读
    assert b["com"] == pytest.approx([0.0, 0.0, 0.0])


def test_robot_section_has_urdf_hash_and_joint_order(tmp_path):
    d = _write(tmp_path)
    assert d["robot"]["urdf_sha256"] is not None
    assert len(d["robot"]["urdf_sha256"]) == 64
    assert d["robot"]["joints"] == d["joints"]
    assert d["robot"]["links"] == ["linkA", "linkB", "linkC", "linkD"]


def test_backward_compatible_keys_unchanged(tmp_path):
    """schema 1 的五个键必须原样保留（历史读取方不能坏）。"""
    d = _write(tmp_path)
    for k in ("joints", "links", "gravity", "friction", "fit", "meta"):
        assert k in d, k
    assert set(d["gravity"]) == {"params", "masses", "coms", "nominal_masses",
                                 "nominal_coms", "rank", "cond"}
    assert set(d["friction"]) == {"c1_pos", "c1_neg", "c2"}


def test_with_inertia_marks_usable_and_fills_tensors(tmp_path):
    """开了惯性项：usable_for_mass_matrix=true，且 I 由 pi_I 填成对称矩阵。"""
    grav = _fake_gravity("static_friction_inertia")
    n = len(grav.link_names)
    # 6 分量/连杆：Ixx Ixy Iyy Ixz Iyz Izz
    pi_I = np.arange(1.0, 6.0 * n + 1.0)
    fit = {"pi_I": pi_I, "pi_ID": pi_I, "rank_I": 3, "n_train": 10}
    out = tmp_path / "p2.json"
    # 与真实调用方一致：pi_I 走 fit（用来填 I），rank_I/n_train 走 extra（并入 inertia 段）
    save_params(out, grav, _fake_friction(), urdf_text="<robot/>",
                model_kind="static_friction_inertia", fit=fit,
                extra={"inertia": {"pi_I": pi_I.tolist(), "pi_ID": pi_I.tolist(),
                                   "rank_I": 3, "n_train": 10}})
    d = json.loads(out.read_text(encoding="utf-8"))
    assert d["inertia"]["usable_for_mass_matrix"] is True
    I0 = d["inertia"]["links"]["linkA"]["I"]
    a, b, c, dd, e, f = pi_I[:6]
    assert I0 == [[a, b, c], [b, dd, e], [c, e, f]]
    # 对称
    assert I0[0][1] == I0[1][0]
    assert d["inertia"]["rank_I"] == 3     # extra 并入而非覆盖
    assert d["inertia"]["mode"] == "static_friction_inertia"

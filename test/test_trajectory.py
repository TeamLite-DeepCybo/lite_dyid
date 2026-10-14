# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""轨迹层的单测：不依赖机器人，也不依赖 pinocchio。"""

import numpy as np
import pytest

from lite_dyid.common import ARM_JOINTS, config_path
from lite_dyid.trajectory import FourierTrajectory, load_specs

#: 本机实测参考姿态（rad）
REF_POSE = np.array([
    0.0564, -1.5582, 0.4550, -0.1300, 0.2127, 0.0625, -0.0307,
    0.0961, 1.5559, -0.1918, 0.1992, 0.7816, 0.0640, -0.1787,
])

#: 与 URDF 一致的限位（rad）
LIMITS = {
    "left_shoulder_pitch": (-3.141592653589793, 0.7853981633974483),
    "left_shoulder_roll": (-1.5707963267948966, 1.9198621771937625),
    "left_shoulder_yaw": (-1.5707963267948966, 1.5707963267948966),
    "left_elbow_pitch": (-2.356194490192345, 0.0),
    "left_wrist_yaw": (-1.5707963267948966, 1.5707963267948966),
    "left_wrist_roll": (-0.6981317007977318, 0.6981317007977318),
    "left_wrist_pitch": (-0.7853981633974483, 0.7853981633974483),
    "right_shoulder_pitch": (-3.141592653589793, 0.7853981633974483),
    "right_shoulder_roll": (-1.9198621771937625, 1.5707963267948966),
    "right_shoulder_yaw": (-1.5707963267948966, 1.5707963267948966),
    "right_elbow_pitch": (0.0, 2.356194490192345),
    "right_wrist_yaw": (-1.5707963267948966, 1.5707963267948966),
    "right_wrist_roll": (-0.6981317007977318, 0.6981317007977318),
    "right_wrist_pitch": (-0.7853981633974483, 0.7853981633974483),
}


def _limits() -> tuple:
    return (np.array([LIMITS[j][0] for j in ARM_JOINTS]),
            np.array([LIMITS[j][1] for j in ARM_JOINTS]))


def _specs():
    return load_specs(config_path("trajectories.yaml"))


def test_config_has_three_roles():
    specs = _specs()
    roles = sorted(s.role for s in specs.values())
    assert roles == ["train", "train", "validate"], roles
    assert len(specs) == 3


@pytest.mark.parametrize("name", ["dyid_train_left", "dyid_train_right", "dyid_validate"])
def test_trajectory_respects_limits_speed_and_shoulder(name):
    spec = _specs()[name]
    traj = FourierTrajectory(spec, REF_POSE, _limits(), speed_cap=0.15)
    chk = traj.check(samples=3000)
    assert chk.ok, chk.problems
    lo, hi = _limits()
    # 稳态（t >= ramp）必须留在限位内且有余量
    for k in range(len(ARM_JOINTS)):
        if abs(spec.amplitudes[k]) < 1e-12:
            continue
        assert chk.q_min[k] >= lo[k] - 1e-9
        assert chk.q_max[k] <= hi[k] + 1e-9
    # 速度 ≈ 0.15 rad/s 水平
    assert chk.v_peak.max() <= 0.20
    assert chk.v_peak.max() >= 0.05
    # 肩部单关节变动 ≤ 50°
    for k, jn in enumerate(ARM_JOINTS):
        if "shoulder" in jn and abs(spec.amplitudes[k]) > 1e-12:
            assert chk.travel_deg[k] <= 50.0, (jn, chk.travel_deg[k])


def test_non_excited_joint_is_strictly_held():
    """未激励的关节（另一条臂）必须**严格**留在起始姿态，不能被限位裁剪带跑。"""
    specs = _specs()
    for name, arm in (("dyid_train_left", slice(7, 14)), ("dyid_train_right", slice(0, 7))):
        spec = specs[name]
        traj = FourierTrajectory(spec, REF_POSE, _limits(), speed_cap=0.15)
        for t in np.linspace(0.0, traj.duration, 200):
            q, _ = traj.evaluate(float(t))
            assert np.allclose(q[arm], REF_POSE[arm], atol=1e-12), (name, t)


def test_entry_blend_starts_at_entry_position():
    """t=0 必须**恰好**等于起始姿态（否则第一步就是阶跃）。"""
    spec = _specs()["dyid_train_left"]
    traj = FourierTrajectory(spec, REF_POSE, _limits(), speed_cap=0.15)
    q0, _ = traj.evaluate(0.0)
    assert np.allclose(q0, REF_POSE, atol=1e-12)


def test_ramp_duration_respects_speed_cap():
    spec = _specs()["dyid_train_left"]
    traj = FourierTrajectory(spec, REF_POSE, _limits(), speed_cap=0.05)
    travel = float(np.max(np.abs(traj.center - REF_POSE)))
    assert traj.ramp >= 1.5 * travel / 0.05 - 1e-9


def test_mirrored_pair_is_mirrored():
    """前两条的幅值应左/右镜像（pitch 保号、roll/yaw 取反）。"""
    specs = _specs()
    l = specs["dyid_train_left"].amplitudes
    r = specs["dyid_train_right"].amplitudes
    sign = np.array([1, -1, -1, 1, -1, -1, 1], dtype=float)
    assert np.allclose(l[:7], r[7:] * sign)
    assert np.allclose(l[7:], 0.0)
    assert np.allclose(r[:7], 0.0)


def test_static_insert_times():
    spec = _specs()["dyid_train_left"]
    times = spec.joint_static_times()
    assert len(times) == spec.static_insert_count == 20
    assert times == [10.0 * k for k in range(1, 21)]
    assert spec.static_hold_sec == 10.0

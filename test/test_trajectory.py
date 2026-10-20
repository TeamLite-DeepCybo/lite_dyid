# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""默认交付轨迹的离线回归测试。"""

import numpy as np
import pytest

from lite_dyid.common import (MIRROR_SIGN, config_path, velocity_caps)
from lite_dyid.design import TRAJECTORY_NAME, build_specs
from lite_dyid.limits_table import limits_array, midpoint
from lite_dyid.trajectory import FourierTrajectory, load_specs


REF_POSE = np.array([
    0.0564, -1.5582, 0.4550, -0.1300, 0.2127, 0.0625, -0.0307,
    0.0961, 1.5559, -0.1918, 0.1992, 0.7816, 0.0640, -0.1787,
])


def _spec():
    specs = load_specs(config_path("trajectories.yaml"))
    assert list(specs) == [TRAJECTORY_NAME]
    return specs[TRAJECTORY_NAME]


def _trajectory(entry=REF_POSE):
    lo, hi = limits_array()
    return FourierTrajectory(_spec(), np.asarray(entry), (lo, hi),
                             v_cap=np.asarray(velocity_caps()))


def test_default_is_single_300_second_bimanual_train_trajectory():
    spec = _spec()
    assert spec.role == "train"
    assert spec.duration_sec == pytest.approx(300.0)
    assert np.all(np.abs(spec.amplitudes) > 0.0)
    assert spec.min_coverage == pytest.approx(0.80)


def test_low_frequency_harmonics_have_strictly_larger_amplitude():
    weights = _spec().harmonic_weights
    assert np.all(np.diff(weights) < 0.0)
    assert weights.tolist() == pytest.approx([1.0, 0.35, 0.15, 0.05])


def test_effective_travel_is_at_least_80_percent_for_every_joint():
    traj = _trajectory()
    check = traj.check(samples=12000)
    assert check.ok, check.problems
    assert np.all(check.coverage >= 0.798)  # 密采峰值误差 < 0.2%
    assert traj.clamp_count == 0


def test_every_joint_velocity_is_bounded_and_not_too_slow():
    spec = _spec()
    check = _trajectory().check(samples=12000)
    caps = np.asarray(velocity_caps())
    active = np.abs(spec.amplitudes) > 1e-12
    assert np.all(check.v_peak[active] <= caps[active] + 1e-6)
    assert np.all(check.v_peak[active] >= spec.min_peak_speed - 1e-6)


def test_both_arms_move_in_the_same_time_window():
    traj = _trajectory(midpoint())
    moved_together = False
    for t in np.linspace(traj.ramp, traj.duration, 500):
        q, _ = traj.evaluate(float(t))
        dq = np.abs(q - midpoint())
        if np.max(dq[:7]) > 0.1 and np.max(dq[7:]) > 0.1:
            moved_together = True
            break
    assert moved_together


def test_right_arm_amplitudes_are_mirrored_from_left():
    amp = _spec().amplitudes
    assert np.allclose(amp[7:], amp[:7] * np.asarray(MIRROR_SIGN), atol=2e-6)


def test_entry_blend_starts_exactly_at_measured_pose():
    q0, v0 = _trajectory().evaluate(0.0)
    assert np.allclose(q0, REF_POSE, atol=1e-12)
    assert np.allclose(v0, 0.0, atol=1e-12)


def test_static_samples_cover_the_full_motion_timeline():
    spec = _spec()
    assert spec.joint_static_times() == [20.0 * k for k in range(1, 16)]
    assert spec.static_hold_sec == pytest.approx(6.0)
    assert spec.static_settle_sec == pytest.approx(3.0)


def test_generator_and_checked_in_config_describe_same_trajectory():
    generated = build_specs()["trajectories"][0]
    spec = _spec()
    assert generated["name"] == spec.name
    assert generated["duration_sec"] == spec.duration_sec
    assert generated["frequency_hz"] == spec.frequency_hz
    assert generated["amplitudes"] == pytest.approx(spec.amplitudes.tolist())

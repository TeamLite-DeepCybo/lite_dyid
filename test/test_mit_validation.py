"""MIT 跟踪验证的协议进入、轨迹定义与统计单测。"""

from types import SimpleNamespace

import numpy as np
import pytest

from lite_dyid.common import (CSP_MONITOR, MIT_HOLD, MIT_TRACKING,
                              PROTOCOL_CSP, PROTOCOL_MIT, DyidError)
from lite_dyid.csp_client import BridgeClient, ProtocolRestartRequired
from lite_dyid.report import tracking_error_stats
from lite_dyid.tracking_validate import (build_validation_trajectory,
                                         check_live_urdf_compatible,
                                         find_latest_run, sample_reference)


def _status(protocol, mode, *, active=True):
    return SimpleNamespace(
        ok=True, message="", protocol=protocol,
        protocol_name="mit" if protocol == PROTOCOL_MIT else "csp",
        mode=mode, mode_name={0: "ZERO_TORQUE", 1: "HOLD", 2: "TRACKING",
                              5: "CSP_MONITOR"}.get(mode, "?"),
        active_controller="controller", controller_active=active,
        zero_torque_loaded=(protocol == PROTOCOL_MIT),
        hold_loaded=(protocol == PROTOCOL_MIT),
        tracking_loaded=(protocol == PROTOCOL_MIT),
    )


class _FakeMitClient:
    source = "dyid"

    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.mode_calls = []
        self.published = []
        self.auto_calls = 0

    def get_status(self):
        return self.statuses.pop(0)

    def set_mode(self, protocol, mode, **_kwargs):
        self.mode_calls.append((protocol, mode))
        actual_protocol = PROTOCOL_CSP if mode == CSP_MONITOR else PROTOCOL_MIT
        return SimpleNamespace(ok=True, message="", actual_protocol=actual_protocol,
                               actual_protocol_name="csp" if actual_protocol else "mit",
                               actual_mode=mode, actual_mode_name=str(mode),
                               controller_active=True)

    def set_source_auto(self, timeout=10.0):
        self.auto_calls += 1
        return SimpleNamespace(ok=True)

    def lock_source(self, timeout=10.0):
        return SimpleNamespace(ok=True, source_online=True, active_source="dyid", message="")

    def pose_dict(self):
        return {f"j{i}": 0.01 * i for i in range(14)}

    def publish_mit(self, q, v, tau, **kwargs):
        self.published.append((np.asarray(q), np.asarray(v), np.asarray(tau), kwargs))


def test_csp_is_disabled_then_restart_is_required_without_mit_frames():
    fake = _FakeMitClient([_status(PROTOCOL_CSP, CSP_MONITOR)])
    with pytest.raises(ProtocolRestartRequired, match="control_mode:=mit"):
        BridgeClient.ensure_mit_ready(fake)
    assert [mode for _, mode in fake.mode_calls] == [CSP_MONITOR]
    assert fake.auto_calls == 1
    assert fake.published == []


def test_mit_entry_always_goes_through_hold_and_rechecks_tracking():
    fake = _FakeMitClient([
        _status(PROTOCOL_MIT, MIT_TRACKING),
        _status(PROTOCOL_MIT, MIT_TRACKING),
    ])
    text = BridgeClient.ensure_mit_ready(fake)
    assert [mode for _, mode in fake.mode_calls] == [MIT_HOLD, MIT_TRACKING]
    assert len(fake.published) == 6
    assert all(frame[3] == {"kp": 15.0, "kd": 3.0} for frame in fake.published)
    assert "二次回读" in text


def test_mit_finish_switches_hold_before_stopping_stream_and_releasing_source():
    log = []

    class _Node:
        def destroy_publisher(self, _pub):
            log.append("stop")

    class _FinishClient:
        node = _Node()
        pub = object()

        def publish_mit(self, *args, **kwargs):
            log.append("publish")

        def set_mode(self, _protocol, mode, **kwargs):
            log.append(f"mode_{mode}")
            return SimpleNamespace(ok=True, actual_mode=mode, actual_mode_name="HOLD")

        def set_source_auto(self, **kwargs):
            log.append("auto")
            return SimpleNamespace(ok=True)

    text = BridgeClient.finish_mit_session(
        _FinishClient(), entered_tracking=True, held_position=np.zeros(14))
    assert log == ["publish", f"mode_{MIT_HOLD}", "stop", "auto"]
    assert "MIT_HOLD" in text


class _LimitModel:
    def limit_array(self):
        return np.full(14, -2.0), np.full(14, 2.0)


def test_validation_trajectory_has_exactly_f_and_2f_for_all_joints():
    traj = build_validation_trajectory(_LimitModel(), np.zeros(14), duration=60.0,
                                       base_frequency=0.025)
    assert traj.spec.phases.shape == (14, 2)
    assert np.allclose(traj.spec.harmonic_indices, [1.0, 2.0])
    assert np.allclose(traj.omega / (2.0 * np.pi), [0.025, 0.05])
    assert np.all(np.abs(traj.spec.amplitudes) > 0.0)
    t, q, v = sample_reference(traj, rate_hz=20.0)
    assert t.shape[0] == 1201
    assert q.shape == v.shape == (1201, 14)
    assert np.allclose(q[0], 0.0)


def test_tracking_error_stats_reports_rad_and_degree():
    error = np.full((10, 14), 0.1)
    rows = tracking_error_stats(error, condition="with_identified_ff")
    assert len(rows) == 14
    assert rows[0]["condition"] == "with_identified_ff"
    assert rows[0]["rmse_rad"] == pytest.approx(0.1)
    assert rows[0]["rmse_deg"] == pytest.approx(np.degrees(0.1))


def test_find_latest_run_accepts_an_explicit_result(tmp_path):
    explicit = tmp_path / "run_1"
    explicit.mkdir()
    (explicit / "params.json").write_text("{}")
    (explicit / "robot.urdf").write_text("<robot/>")
    assert find_latest_run(explicit) == explicit.resolve()


def test_live_urdf_accepts_only_expected_csp_to_mit_change():
    template = """<robot name="lite">
      <ros2_control name="left" type="system"><hardware>
        <plugin>lite_driver/LiteSystem</plugin><param name="control_mode">{mode}</param>
      </hardware></ros2_control>
      <joint name="j" type="revolute"><limit lower="-1" upper="1"/></joint>
    </robot>"""
    message = check_live_urdf_compatible(template.format(mode="csp"),
                                         template.format(mode="mit"))
    assert "机械模型一致" in message


def test_live_urdf_rejects_joint_limit_change_even_when_protocol_changes():
    reference = """<robot><ros2_control><hardware>
      <param name="control_mode">csp</param></hardware></ros2_control>
      <joint name="j"><limit lower="-1" upper="1"/></joint></robot>"""
    live = """<robot><ros2_control><hardware>
      <param name="control_mode">mit</param></hardware></ros2_control>
      <joint name="j"><limit lower="-2" upper="1"/></joint></robot>"""
    with pytest.raises(DyidError, match="机械模型差异"):
        check_live_urdf_compatible(reference, live)


def test_live_urdf_ignores_xml_whitespace_only():
    assert "语义一致" in check_live_urdf_compatible(
        '<robot name="lite"><link name="base"/></robot>',
        '<robot name="lite">\n  <link name="base" />\n</robot>')

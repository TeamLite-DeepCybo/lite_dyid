# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""启动模式检查：第一帧位置命令前必须确认 CSP + active TRACKING。"""

from types import SimpleNamespace

import numpy as np
import pytest

from lite_dyid.common import (CSP_HOLD, CSP_MONITOR, CSP_TRACKING,
                              PROTOCOL_CSP, PROTOCOL_MIT, DyidError)
from lite_dyid.csp_client import BridgeClient


def _status(protocol=PROTOCOL_CSP, mode=CSP_MONITOR, active=True):
    names = {CSP_MONITOR: "CSP_MONITOR", CSP_HOLD: "CSP_HOLD",
             CSP_TRACKING: "CSP_TRACKING"}
    return SimpleNamespace(
        ok=True, message="", protocol=protocol,
        protocol_name="csp" if protocol == PROTOCOL_CSP else "mit",
        mode=mode, mode_name=names.get(mode, "MIT_TRACKING"),
        active_controller=names.get(mode, "tracking_controller").lower(),
        controller_active=active,
        csp_monitor_loaded=(protocol == PROTOCOL_CSP),
        csp_hold_loaded=(protocol == PROTOCOL_CSP),
        csp_tracking_loaded=(protocol == PROTOCOL_CSP),
    )


def _mode_response(mode, *, ok=True, active=True):
    return SimpleNamespace(
        ok=ok, message="ok" if ok else "failed",
        actual_protocol=PROTOCOL_CSP, actual_protocol_name="csp",
        actual_mode=mode,
        actual_mode_name="CSP_HOLD" if mode == CSP_HOLD else "CSP_TRACKING",
        controller_active=active,
    )


class _FakeClient:
    source = "dyid"

    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.mode_calls = []
        self.lock_calls = 0
        self.published = []

    def get_status(self):
        return self.statuses.pop(0)

    def set_mode(self, protocol, mode, **_kwargs):
        self.mode_calls.append((protocol, mode))
        return _mode_response(mode)

    def lock_source(self, timeout=10.0):
        self.lock_calls += 1
        return SimpleNamespace(ok=True, message="", available_sources=["dyid"],
                               source_online=True, active_source="dyid")

    def pose_dict(self):
        return {f"j{i}": 0.01 * i for i in range(14)}

    def publish_position(self, q):
        self.published.append(np.asarray(q, dtype=float))


def test_mit_start_is_rejected_before_any_mode_switch_or_lock():
    fake = _FakeClient([_status(PROTOCOL_MIT, mode=2)])
    with pytest.raises(DyidError, match="protocol=mit"):
        BridgeClient.ensure_csp_ready(fake)
    assert fake.mode_calls == []
    assert fake.lock_calls == 0


def test_monitor_start_goes_through_hold_then_tracking_and_rechecks():
    fake = _FakeClient([
        _status(PROTOCOL_CSP, CSP_MONITOR, active=True),
        _status(PROTOCOL_CSP, CSP_TRACKING, active=True),
    ])
    summary = BridgeClient.ensure_csp_ready(fake)
    assert [m for _, m in fake.mode_calls] == [CSP_HOLD, CSP_TRACKING]
    assert fake.lock_calls == 1
    assert len(fake.published) == 6
    assert all(np.array_equal(q, fake.published[0]) for q in fake.published)
    assert "二次回读" in summary


def test_existing_tracking_is_first_reset_to_hold():
    fake = _FakeClient([
        _status(PROTOCOL_CSP, CSP_TRACKING, active=True),
        _status(PROTOCOL_CSP, CSP_TRACKING, active=True),
    ])
    BridgeClient.ensure_csp_ready(fake)
    assert [m for _, m in fake.mode_calls] == [CSP_HOLD, CSP_TRACKING]


def test_inactive_hold_is_reapplied_before_tracking():
    fake = _FakeClient([
        _status(PROTOCOL_CSP, CSP_HOLD, active=False),
        _status(PROTOCOL_CSP, CSP_TRACKING, active=True),
    ])
    BridgeClient.ensure_csp_ready(fake)
    assert [m for _, m in fake.mode_calls] == [CSP_HOLD, CSP_TRACKING]


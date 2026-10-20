# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""收尾序列的单测：**顺序**是安全关键，必须钉住。

不需要机器人、不需要 rclpy —— 用假 client 记录调用顺序，
直接调 `BridgeClient.finish_session` 的未绑定函数。

顺序要求：**先停发 → 回安全档 → 再交权**（`csp_client.finish_session` 文档）。
用假对象而不是 mock 库，是为了让"顺序"在断言里一眼可见。
"""

import numpy as np
import pytest

from lite_dyid.csp_client import BridgeClient


class _FakeNode:
    def __init__(self, log):
        self._log = log

    def destroy_publisher(self, pub):
        self._log.append("destroy_publisher")


class _FakeHandle:
    """伪 BridgeClient：只实现 finish_session 会用到的部分。"""

    def __init__(self):
        self.log = []
        self.published = []
        self.node = _FakeNode(self.log)
        self.pub = object()
        self.entered = None
        self.set_mode_calls = []
        self.set_source_calls = 0

    # --- 被 finish_session 调用 ---------------------------------------
    def publish_position(self, q):
        self.published.append(np.asarray(q, dtype=float))

    def set_mode(self, protocol, mode, *, force=False, timeout=30.0):
        self.log.append("set_mode")
        self.set_mode_calls.append((protocol, mode))
        return type("R", (), {"ok": True, "actual_mode": mode,
                              "actual_mode_name": "CSP_HOLD"})()

    def set_source_auto(self, timeout=10.0):
        self.log.append("set_source_auto")
        self.set_source_calls += 1
        return type("R", (), {"ok": True, "active_source": "",
                              "arbitration_state_name": "IDLE"})()


def _run(entered_tracking, held=None, settle=0.0):
    fake = _FakeHandle()
    # 未绑定调用：把 fake 当 self
    summary = BridgeClient.finish_session(
        fake, entered_tracking=entered_tracking,
        held_position=held, settle_sec=settle)
    return fake, summary


def test_stop_publishing_happens_first():
    """① 必须**先**停发，再动任何模式/仲裁。"""
    fake, _ = _run(True)
    assert fake.log[0] == "destroy_publisher", fake.log
    assert fake.log.index("destroy_publisher") < fake.log.index("set_mode")


def test_order_is_stop_then_hold_then_release():
    """严格顺序：停发 → set_mode(CSP_HOLD) → set_source(AUTO)。"""
    fake, _ = _run(True)
    assert fake.log == ["destroy_publisher", "set_mode", "set_source_auto"], fake.log


def test_no_mode_change_when_never_entered_tracking():
    """⚠️ 没进过 TRACKING（电机未使能）**绝不能**切 CSP_HOLD —— 那会凭空使能电机。"""
    fake, summary = _run(False)
    assert "set_mode" not in fake.log, fake.log
    assert fake.log == ["destroy_publisher", "set_source_auto"], fake.log
    assert "未进入过 TRACKING" in summary


def test_last_frame_is_the_position_we_want_to_hold():
    """给了 held_position 就把它作为最后一帧发出去（钉住目标再停发）。"""
    held = np.linspace(-0.5, 0.5, 14)
    fake, _ = _run(True, held=held)
    assert len(fake.published) == 1
    assert np.allclose(fake.published[0], held)


def test_no_extra_frame_without_held_position():
    fake, _ = _run(True, held=None)
    assert fake.published == []


def test_release_always_happens_even_if_mode_switch_fails():
    """③ 失败也要做 ④：控制权必须被释放，否则操作员卡在"占着源"。"""

    class _Boom(_FakeHandle):
        def set_mode(self, *a, **k):
            raise RuntimeError("bridge 挂了")

    fake = _Boom()
    summary = BridgeClient.finish_session(fake, entered_tracking=True,
                                          held_position=None, settle_sec=0.0)
    assert fake.set_source_calls == 1, "set_source(AUTO) 未被调用"
    assert "切 CSP_HOLD 失败" in summary

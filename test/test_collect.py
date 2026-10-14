# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""主流程里"最后一道人工闸门"的单测：验证询问的三态。

不需要机器人、不需要 rclpy（collect 模块顶层不 import rclpy）。
"""

import builtins

import pytest

from lite_dyid.collect import ask_validate, build_parser


class _Args:
    def __init__(self, auto=False, noprompt=False):
        self.auto_validate = auto
        self.no_validate_prompt = noprompt


def test_auto_validate_skips_prompt(monkeypatch):
    def _boom(*a, **k):
        raise AssertionError("--auto-validate 不该再问用户")

    monkeypatch.setattr(builtins, "input", _boom)
    assert ask_validate(_Args(auto=True)) is True


def test_no_validate_prompt_returns_false(monkeypatch):
    monkeypatch.setattr(builtins, "input", lambda *a: pytest.fail("不该问"))
    assert ask_validate(_Args(noprompt=True)) is False


@pytest.mark.parametrize("answer,expected", [
    ("y", True), ("Y", True), ("yes", True), ("YES", True), ("是", True),
    (" y ", True), ("n", False), ("", False), ("no", False), ("随便", False),
])
def test_prompt_answers(monkeypatch, answer, expected):
    monkeypatch.setattr(builtins, "input", lambda *a: answer)
    assert ask_validate(_Args()) is expected


def test_closed_stdin_is_not_validate(monkeypatch):
    def _eof(*a, **k):
        raise EOFError

    monkeypatch.setattr(builtins, "input", _eof)
    assert ask_validate(_Args()) is False


def test_cli_flags_exist_and_default_off():
    a = build_parser().parse_args([])
    assert a.auto_validate is False and a.no_validate_prompt is False
    a = build_parser().parse_args(["--auto-validate"])
    assert a.auto_validate is True

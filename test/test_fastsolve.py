# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""快速辨识内核的单测：并行顺序、零相位滤波、法方程等价性。

不需要机器人、不需要 ROS 话题；`fork_map` 用 `os.fork`，在测试进程里可用。
"""

import numpy as np
import pytest

from lite_dyid.fastsolve import BLK, fork_map, lowpass, solve_ne


# ---------------------------------------------------------------------------
# 并行：**顺序必须保持**
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("workers", [1, 2, 3, 4, 7, 16])
def test_fork_map_preserves_order(workers):
    """⚠️ 回归测试：早期版本用 `items[i::workers]` 分组后按组顺序拼接，
    结果整体错位（耗时正常、数值全错：验证残差 0.116 → 0.507）。"""
    items = list(range(23))
    got = fork_map(lambda x: x * x, items, workers)
    assert got == [x * x for x in items]


def test_fork_map_single_item_and_serial():
    assert fork_map(lambda x: x + 1, [41], 8) == [42]
    assert fork_map(lambda x: x + 1, [1, 2, 3], 1) == [2, 3, 4]


def test_fork_map_child_init_called():
    calls = []

    def init():
        calls.append(1)

    fork_map(lambda x: x, [1, 2, 3, 4], 4, child_init=init)
    # 子进程里 append 不影响父进程；这里只要求不抛异常（init 可调用）
    assert fork_map(lambda x: x, [1, 2], 2, child_init=init) == [1, 2]


def test_fork_map_propagates_child_failure():
    def boom(_):
        raise RuntimeError("nope")

    with pytest.raises(RuntimeError):
        fork_map(boom, [1, 2, 3, 4], 4)


# ---------------------------------------------------------------------------
# 滤波：零相位（不能有延迟）
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("fc", [1.0, 5.0])
def test_lowpass_is_zero_phase(fc):
    """有相位延迟的话 τ 与 q 会对不齐，辨识会被"时间错位"污染。

    ⚠️ 信号要足够长：`filtfilt` 两端仍有暂态，窗口太短会把暂态当成本体。
    """
    fs = 200.0
    t = np.arange(0, 30.0, 1.0 / fs)          # 30 s = 9 个周期
    sig = np.sin(2 * np.pi * 0.3 * t)          # 通带内 → 应几乎原样通过
    y = lowpass(sig[:, None], fc, fs)[:, 0]
    a, b = int(2 * fs), int(28 * fs)           # 只比中段，避开两端暂态
    lag = int(np.argmax(np.correlate(y[a:b] - y[a:b].mean(),
                                     sig[a:b] - sig[a:b].mean(), "full"))) - (b - a - 1)
    assert abs(lag) <= 1, f"相位延迟 {lag} 个样本"
    assert np.corrcoef(y[a:b], sig[a:b])[0, 1] > 0.999


def test_lowpass_removes_high_frequency_but_keeps_motion_band():
    """0.2 Hz 截止：0.1 Hz 的运动要保住，20 Hz 的噪声要去掉。

    60 s = 6 个 0.1 Hz 周期，保证 `filtfilt` 的暂态不吃掉主要信号。
    """
    fs = 200.0
    t = np.arange(0, 60.0, 1.0 / fs)
    motion = np.sin(2 * np.pi * 0.1 * t)
    noise = 0.3 * np.sin(2 * np.pi * 20.0 * t)
    y = lowpass((motion + noise)[:, None], 0.2, fs)[:, 0]
    a, b = int(5 * fs), int(55 * fs)
    assert np.sqrt(np.mean((y[a:b] - motion[a:b]) ** 2)) < 0.02
    assert np.corrcoef(y[a:b], motion[a:b])[0, 1] > 0.99


def test_lowpass_disabled_when_zero():
    x = np.random.default_rng(0).normal(size=(500, 3))
    assert np.allclose(lowpass(x, 0.0, 200.0), x)


# ---------------------------------------------------------------------------
# 求解：法方程 == 列归一化 + 岭（与旧实现同一估计量）
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("ridge", [0.0, 1e-8, 1e-4])
def test_solve_ne_matches_ridge_ls(ridge):
    """法方程必须与旧 `solve_bounded` **同一个估计量**。

    ⚠️ 旧实现的岭是在**列归一化之后**加的：最小化
       ``||A_s x_s − b||² + λ||x_s||²``，x = x_s / scale。
    等价于对 x 施加 ``λ·diag(scale)²`` 的加权岭 —— **不是**均匀 λI。
    所以参考实现必须先按列归一化再加岭，否则比的是两回事。
    """
    rng = np.random.default_rng(1)
    A = rng.normal(size=(400, 12))
    b = rng.normal(size=400)
    x_ne = solve_ne(A.T @ A, A.T @ b, ridge)

    scale = np.linalg.norm(A, axis=0)
    scale[scale < 1e-12] = 1.0
    As = A / scale
    if ridge > 0:
        aug = np.vstack([As, np.sqrt(ridge) * np.eye(12)])
        rhs = np.concatenate([b, np.zeros(12)])
    else:
        aug, rhs = As, b
    x_ref, *_ = np.linalg.lstsq(aug, rhs, rcond=None)
    x_ref = x_ref / scale
    assert np.allclose(x_ne, x_ref, atol=1e-7, rtol=1e-5)


def test_solve_ne_handles_dead_column():
    """未被激励的列（对角线为 0）不能出 NaN —— 靠岭把不可辨识方向压住。"""
    rng = np.random.default_rng(3)
    A = rng.normal(size=(200, 6))
    A[:, 4] = 0.0                                 # 死列
    b = rng.normal(size=200)
    x = solve_ne(A.T @ A, A.T @ b, 1e-6)
    assert np.all(np.isfinite(x))
    assert abs(x[4]) < 1e-3


def test_blk_is_sane():
    assert isinstance(BLK, int) and BLK >= 16

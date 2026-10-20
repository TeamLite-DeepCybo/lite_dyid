# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""CSV 落盘 / 读回。

约定
====
* 第一列固定是 ``traj``（轨迹名，字符串），其余列都是浮点。
* 列名用 ``q/<joint>``、``v/<joint>``、``tau/<joint>``、``cmd/<joint>`` 前缀，
  与 tmp/bar_dynamics_identify 的 csv 风格一致，便于人眼与脚本定位。
* 轨迹很长（300 s @200 Hz ≈ 6 万行），所以用**预分配**的行缓冲，
  避免用 dict 逐行累积（那是几十倍的内存）。
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

import numpy as np

from .common import ARM_JOINTS, DyidError


def _prefix(prefix: str) -> List[str]:
    return [f"{prefix}/{j}" for j in ARM_JOINTS]


def dynamics_header() -> List[str]:
    return (["motion_t", "wall_t"]
            + _prefix("cmd") + _prefix("q") + _prefix("v") + _prefix("tau"))


def statics_header() -> List[str]:
    return (["static_index", "motion_t", "n_samples", "duration_sec"]
            + _prefix("q") + _prefix("tau")
            + _prefix("q_std") + _prefix("tau_std"))


def statics_raw_header() -> List[str]:
    return ["static_index", "wall_t"] + _prefix("q") + _prefix("v") + _prefix("tau")


DYN_NCOL = len(dynamics_header())
STA_NCOL = len(statics_header())
STARAW_NCOL = len(statics_raw_header())


class RowBuffer:
    """预分配的定长行缓冲。"""

    def __init__(self, ncols: int, max_rows: int) -> None:
        self.ncols = ncols
        self.buf = np.zeros((max(1, int(max_rows)), ncols), dtype=float)
        self.n = 0

    def add(self, row: Sequence[float]) -> None:
        if self.n >= self.buf.shape[0]:
            self.buf = np.vstack([self.buf, np.zeros_like(self.buf)])
        self.buf[self.n, :] = row
        self.n += 1

    def array(self) -> np.ndarray:
        return self.buf[:self.n, :]


def save_csv(path: Path, header: Sequence[str], data: np.ndarray,
             traj_name: str = "", fmt: str = "%.6g") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = np.atleast_2d(np.asarray(data, dtype=float))
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["traj"] + list(header))
        for row in data:
            writer.writerow([traj_name] + [fmt % v for v in row])


def append_csv(path: Path, header: Sequence[str], data: np.ndarray,
               traj_name: str = "", fmt: str = "%.6g") -> None:
    """追加到已有文件（不存在则写表头）。"""
    data = np.atleast_2d(np.asarray(data, dtype=float))
    if data.size == 0:
        return
    exists = path.is_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        if not exists:
            writer.writerow(["traj"] + list(header))
        for row in data:
            writer.writerow([traj_name] + [fmt % v for v in row])


@dataclass
class Table:
    header: List[str]
    data: np.ndarray            # (n, len(header)) 浮点
    traj: List[str]             # 每行的 traj 名

    def col(self, name: str) -> np.ndarray:
        try:
            idx = self.header.index(name)
        except ValueError as exc:
            raise DyidError(f"CSV 缺少列 {name}（表头：{self.header[:8]}…）") from exc
        return self.data[:, idx]

    def cols(self, names: Sequence[str]) -> np.ndarray:
        idx = [self.header.index(n) for n in names]
        return self.data[:, idx]

    @property
    def n(self) -> int:
        return self.data.shape[0]


def read_table(path: Path) -> Table:
    if not Path(path).is_file():
        raise DyidError(f"CSV 不存在：{path}")
    header: List[str] = []
    traj: List[str] = []
    rows: List[List[float]] = []
    with Path(path).open("r", newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        for i, raw in enumerate(reader):
            if i == 0:
                if not raw or raw[0] != "traj":
                    raise DyidError(f"{path} 表头第一列必须是 traj，得到 {raw[:3]}")
                header = raw[1:]
                continue
            # ⚠️ 必须校验宽度：列数对不上时 numpy 会把数据**整体错位**读进来，
            #    结果是"每个数值看着都合理、但对应的是别的关节/别的量"，
            #    而且不会报任何错（实测踩过：少写 13 列 → 残差全是垃圾）。
            if len(raw) - 1 != len(header):
                raise DyidError(
                    f"{path} 第 {i + 1} 行有 {len(raw) - 1} 列，表头有 {len(header)} 列 —— "
                    "列数不符会让数据整体错位，拒绝读取。"
                )
            traj.append(raw[0])
            rows.append([_to_float(x) for x in raw[1:]])
    data = np.asarray(rows, dtype=float) if rows else np.zeros((0, len(header)))
    return Table(header=header, data=data, traj=traj)


def _to_float(text: str) -> float:
    try:
        return float(text)
    except (TypeError, ValueError):
        return float("nan")


def joint_matrix(table: Table, prefix: str) -> np.ndarray:
    """取 ``prefix/<joint>`` 这 14 列，返回 (n, 14)。"""
    return table.cols(_prefix(prefix))

# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

"""启动 lite_dyid 主流程。

⚠️ 本 launch **不启动** bringup：默认已经有一套 CSP 组的 bringup 在跑
   （见 docs/RUNBOOK.md 的第一步）。本 launch 只负责按参数拉起 dyid_collect。

用法：
    ros2 launch lite_dyid dyid.launch.py
    ros2 launch lite_dyid dyid.launch.py speed_cap:=0.12 source:=dyid
    ros2 launch lite_dyid dyid.launch.py skip_small_check:=true   # 不推荐
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

#: 名称 → (默认值, 说明)。空字符串表示"不传该命令行参数"。
ARGS = {
    "output_dir": ("", "结果目录（默认 ~/.ros/lite_dyid/runs）"),
    "source": ("", "数据源名（默认用 config/dyid.yaml 里的 dyid）"),
    "trajectories": ("", "轨迹 YAML（默认包内 config/trajectories.yaml）"),
    "rate_hz": ("", "下发频率 Hz"),
    "speed_cap": ("", "轨迹速度上限 rad/s"),
    "ridge": ("", "重力辨识岭系数"),
    "small_delta": ("", "小幅验证单关节幅度 rad"),
    "skip_small_check": ("false", "跳过小幅运动验证（不推荐）"),
    "small_check_only": ("false", "只运行 CSP 检查和小幅验证，不运行大幅轨迹"),
    "auto_validate": ("false", "若自定义 YAML 含 role=validate，采集后自动运行它"),
    "no_validate_prompt": ("false", "不询问、也不验证"),
}

#: launch 名 → dyid_collect 的命令行开关
FLAGS = {
    "skip_small_check": "--skip-small-check",
    "small_check_only": "--small-check-only",
    "auto_validate": "--auto-validate",
    "no_validate_prompt": "--no-validate-prompt",
}

VALUE_ARGS = {
    "output_dir": "--output-dir",
    "source": "--source",
    "trajectories": "--trajectories",
    "rate_hz": "--rate-hz",
    "speed_cap": "--speed-cap",
    "ridge": "--ridge",
    "small_delta": "--small-delta",
}


def _spawn(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    argv = []
    for name, (default, _doc) in ARGS.items():
        value = LaunchConfiguration(name).perform(context)
        if name in FLAGS:
            if str(value).strip().lower() in ("1", "true", "yes"):
                argv.append(FLAGS[name])
            continue
        if str(value).strip():
            argv += [VALUE_ARGS[name], str(value)]
    return [Node(
        package="lite_dyid",
        executable="dyid_collect",
        name="dyid_collect",
        output="screen",
        emulate_tty=True,
        arguments=argv,
    )]


def generate_launch_description():
    decls = [DeclareLaunchArgument(name, default_value=default, description=doc)
             for name, (default, doc) in ARGS.items()]
    return LaunchDescription(decls + [OpaqueFunction(function=_spawn)])

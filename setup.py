# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

from glob import glob
from setuptools import find_packages, setup

package_name = "lite_dyid"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/docs", glob("docs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marki",
    maintainer_email="marki@example.com",
    description="Lite 双臂 CSP 动力学辨识（傅里叶激励 + 重力/摩擦辨识 + 验证）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 主入口：确认 CSP → 小幅验证 → 双臂同步 300 s 采集 → 自动解算
            "dyid_collect = lite_dyid.collect:main",
            # 离线重解（只读 CSV，不碰机器人）
            "dyid_identify = lite_dyid.identify:main",
            # 可选外部验证轨迹（残差 + 可视化；默认交付配置不含额外运动）
            "dyid_validate = lite_dyid.validate:main",
            # 轨迹设计/校核工具（限位 + 速度 + mesh 碰撞，不碰机器人）
            "dyid_design = lite_dyid.design:main",
        ],
    },
)

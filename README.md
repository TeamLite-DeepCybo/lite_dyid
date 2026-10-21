# lite_dyid

Lite 双臂动力学辨识包。它通过 CSP 位置模式执行可复现的傅里叶激励，记录关节
位置、速度和力矩，再估计重力参数与摩擦参数，输出供 `Lite_move_controller`
直接读取的 `params.json`。

## 原理

辨识流程分为四步：

```text
状态与模型检查
   ▼
逐关节小幅通路检查
   ▼
双臂同步傅里叶轨迹 + 周期静止段
   ▼
重力回归 → 固定质量/质心 → 摩擦回归 → params.json
```

运动段提供不同位置和速度下的关节力矩样本；静止段降低速度项对重力估计的影响。
默认模型先利用 Pinocchio 回归每个连杆的质量与一阶矩，再从剩余力矩拟合正反向
库仑摩擦和速度项。`--with-inertia` 可进一步辨识惯量项。

默认训练轨迹为双臂同步 300 秒傅里叶运动，每 20 秒插入 6 秒静止段。轨迹生成器
按 URDF 限位、速度与碰撞约束做离线校核；采集与求解产物保存在同一个运行目录中。

## 依赖

- ROS 2 Jazzy 与 Python 3.12
- `numpy`、`scipy`、`matplotlib`、`PyYAML`
- Pinocchio 与 hpp-fcl
- Lite 底层运行包：`lite_bringup`、`lite_bridge`、`lite_controller`、
  `lite_driver`、`lite_description`、`robstride_msgs`

本包作为辨识数据源，通过桥的 CSP 通路运动机械臂。它不依赖
`Lite_move_controller` 或 `Lite_IK_node` 才能完成辨识。

## 构建

把三个仓库并列放在工作空间 `src` 下，然后构建：

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lite_dyid
source install/setup.bash
```

Pinocchio 若未由系统包提供，可按部署环境安装后验证：

```bash
/usr/bin/python3 -c "import pinocchio, hppfcl; print('ok')"
```

## 运行

先启动 CSP 控制链，并把 `dyid` 加入桥的数据源：

```bash
ros2 launch lite_bringup lite_bringup.launch.py \
  mode:=arms controller:=csp control_mode:=csp source_names:=dyid
```

主流程：

```bash
ros2 run lite_dyid dyid_collect
```

也可通过 launch 启动：

```bash
ros2 launch lite_dyid dyid.launch.py
```

常用入口：

| 命令 | 用途 |
|---|---|
| `dyid_collect` | 通路检查、采集和自动解算 |
| `dyid_design --check` | 离线检查轨迹、限位、速度和碰撞 |
| `dyid_identify --run-dir <dir>` | 对已有数据重新解算 |
| `dyid_validate --run-dir <dir>` | 生成残差统计和图表 |
| `dyid_tracking_validate --run-dir <dir>` | 对比无前馈与辨识前馈的 MIT 跟踪误差 |

主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--output-dir` | `~/.ros/lite_dyid/runs` | 辨识运行目录根路径 |
| `--rate-hz` | `200` | 采集与命令频率 |
| `--speed-cap` | `0.15` | 轨迹速度上限，rad/s |
| `--lpf` | `0.2` | 求解前零相位低通截止频率，Hz |
| `--workers` | `4` | 并行求解进程数 |
| `--with-inertia` | 关闭 | 增加惯量项辨识 |
| `--small-check-only` | 关闭 | 只执行小幅通路检查 |

完整现场步骤见 [docs/RUNBOOK.md](docs/RUNBOOK.md)。

## 输出

每次运行创建 `run_<时间戳>` 目录，主要文件包括：

```text
run_<时间戳>/
├── robot.urdf
├── run_meta.json
├── raw.csv
├── static.csv
├── params.json
├── metrics.json
└── plots/
```

`params.json` 使用 schema 2，包含关节顺序、URDF 标识、连杆动力学参数、摩擦参数、
模型类型与拟合指标。`Lite_move_controller` 在加载时按关节名和顺序映射参数。

## 交付给 Lite_move_controller

当本次辨识通过残差和跟踪验证后，将结果复制到控制器规定位置：

```bash
cp ~/.ros/lite_dyid/runs/run_<时间戳>/params.json \
  <workspace>/src/Lite_move_controller/lite_move_controller/config/dynamics/identified_params.json

cd <workspace>
colcon build --packages-select lite_move_controller
```

`Lite_move_controller` 仓库随版本带有一份当前 Lite 构型的默认结果。机械结构、负载、
执行器、减速器或关节零位发生变化时，应重新采集并替换该文件，使控制器、IK 模型
和辨识参数对应同一构型。

## 验证建议

先执行离线残差验证，再执行 MIT A/B 跟踪验证。对同一数据用默认求解器和
`--legacy` 求解器交叉对照，可以确认矩阵构建与并行求解没有改变数值结果。

```bash
ros2 run lite_dyid dyid_identify --run-dir <run>
ros2 run lite_dyid dyid_validate --run-dir <run>
ros2 run lite_dyid dyid_tracking_validate --run-dir <run>
```

## 注意事项

- 正式采集前确认工作区无人员和障碍物，并完成小幅通路检查。
- 辨识与验证应使用同一 URDF、关节顺序和低通设置。
- 原始运行目录应保留，便于复算和审计参数来源。

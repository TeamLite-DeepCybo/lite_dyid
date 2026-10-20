# lite_dyid Runbook（手动调试）

> 本文件是**手动调试**用的逐步指南。自动化流程见 `README.md`。
> 适用机型：Lite 双臂（`mode:=arms`，14 个臂关节），CAN 1 Mbit/s，`can0`=左臂、`can1`=右臂。

---

## 0. 三十秒速览

```
① 确认 CAN 干净（没有别的 bringup 在抢同一条总线）
② 用户终端启动 CSP 组 bringup：controller:=csp control_mode:=csp source_names:=dyid
③ 确认 mode5(CSP_MONITOR)、safety=OK、14 关节都有数
④ 跑 dyid_collect：它自动做 CSP 状态确认 → 小幅验证 → 双臂 300 s 采集 → 解算
⑤ 收尾停在 CSP_HOLD；要松开用 §7 的命令
```

**最容易踩的坑**：同时起两套 bringup。两套 `controller_manager` 会同时往同一批电机发帧，
表现为 `TX_QUEUE_OVERRUN` / `RX_TIMEOUT`、桥 `FAULT`、`rejected_commands` 疯涨。
上机前先按 §1.1 查一遍。

---

## 1. 上机前

### 1.1 确认没有残留的 bringup（**必做**）

```bash
ps -eo pid,etime,args | grep -E "lite_bringup|ros2_control_node|bridge_node|mode_manager" | grep -v grep
```

预期：**空**。若有输出：先 `kill -INT <ros2 launch 的 pid>`（优雅退出会发 Disable 帧），
等 10 s，仍不死再 `kill -TERM` / `kill -KILL`。

> ⚠️ 不同 `ROS_DOMAIN_ID` 的栈**不会**在 `ros2 node list` 里互相看见，但**共用同一条 CAN** ——
> 所以"列表里只有一套"并不能证明总线干净。用 `ps` 看才是地面真相。

### 1.2 CAN 接口

```bash
for c in can0 can1; do ip -br link show $c; done
for c in can0 can1; do ip -details link show $c | grep -E "bitrate|can state"; done
```

预期：`UP`、`bitrate 1000000`、`can state ERROR-ACTIVE`。

### 1.3 启动 CSP 组 bringup（**在你自己终端里跑**）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms controller:=csp control_mode:=csp source_names:=dyid
```

三个参数**都要**：

| 参数 | 作用 | 漏了会怎样 |
|---|---|---|
| `controller:=csp` | 加载 csp_monitor / csp_hold / csp_tracking | 加载的是 MIT 组，跑不了 CSP |
| `control_mode:=csp` | 驱动侧协议（写 loc_ref） | 控制器是 CSP、驱动还是 MIT，错配 |
| `source_names:=dyid` | 授权本包的数据源 | 桥丢弃本包的所有命令（日志：`丢弃未知数据源`） |

**预期日志（按顺序）**：

```
[lite_driver]: control_mode=csp —— **位置模式（写 loc_ref）**
[BridgeNode]: 命令下发话题：CSP → /csp_tracking_controller/command（当前模式：CSP）
[BridgeNode]: 命令关节列表（14 个，use_grippers=false）：left_shoulder_pitch, ...
[BridgeNode]: lite_bridge 已启动：1 个数据源，14 个关节，协议=CSP
  left_shoulder_roll    (12) rs-00  保护   5.00 N·m（额定 5.00 / 峰值 14.00） → 4.7777 Apk
[lite_driver]: CSP：**不自动使能**（安全默认 = mode5 只读监视）
```

> ⚠️ CSP 下 `所有关节的最大出力` 由 URDF 的 `max_safe_torque` 决定（本机左肩 roll = 5 N·m）。
> 想更保守就把 `lite.ros2_control.xacro` 里对应关节的 `max_safe_torque` 调小后重新 build。

### 1.4 确认状态

```bash
ros2 control list_controllers
#  csp_monitor_controller   ...   active      ← 只读、不使能
#  csp_hold_controller      ...   inactive
#  csp_tracking_controller  ...   inactive
#  joint_state_broadcaster  ...   active

ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"
#  期望：protocol=csp  mode=5  mode_name=CSP_MONITOR
#        csp_hold_loaded=True  csp_tracking_loaded=True
#        safety_level=0  safety_level_name=OK  rejected_commands=0

ros2 topic echo /safety_status --once      # level: 0  flags: 0
ros2 topic hz /joint_states                # ≈100 Hz
ros2 topic echo /joint_states --once       # 14 个关节，数值有限
```

**此时电机是自由的**（mode5 不使能），机械臂可以手推 —— 这也是确认"没被别的东西使能"的最快方法。

---

## 2. 一键运行

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run lite_dyid dyid_collect --output-dir /tmp/dyid_runs
# 或： ros2 launch lite_dyid dyid.launch.py
```

它会依次做：

| 阶段 | 做什么 | 你要盯什么 |
|---|---|---|
| 预检 | 等 `/joint_states`、抓 URDF、查 `/safety_status`、`ensure_csp_ready()` | 起始姿态打印是否合理；`已进 CSP_HOLD/TRACKING` |
| 小幅验证 | 逐关节 ±8.6°、0.1 rad/s，出去→停 1.5s→回来 | 14 行是否都 ✓；有 ❌ 会**立即中止** |
| 采集 | 左右臂同步运行一条 300 s 轨迹；14 关节各覆盖至少 80%；15×6 s 静止 | `静止 k/15`；`σ_pos_max` 应在 0.0x° 量级 |
| 解算 | 重力 → 冻结 → 摩擦；打印残差表 | 重力拟合 RMS、`k`（应≈1） |

第一次先按 §3.1 只跑小幅通路检查；大幅默认轨迹留给正式采集。

---

## 3. 逐步手动（调试用）

### 3.1 只跑小幅真机检查

```bash
ros2 run lite_dyid dyid_collect --small-check-only --output-dir /tmp/dyid_runs
```

该命令仍会先确认协议是 CSP，并按 HOLD→TRACKING 进入；随后只做 14 个关节各
8.6°、0.1 rad/s 的小幅往返。结束时停止下发、解除数据源锁并停在 CSP_HOLD，
不会进入 80% 大幅轨迹。

### 3.2 轨迹只校核、不动电机

```bash
ros2 run lite_dyid dyid_design --check \
    --urdf <某次采集目录>/robot.urdf \
    --pose-file <某次采集目录>/run_meta.json
```

打印每个被激励关节的稳态区间、速度/加速度峰值、限位余量，以及 mesh 碰撞采样点数。
**不连机器人、不发任何命令。**

### 3.3 离线重解 / 离线验证

```bash
ros2 run lite_dyid dyid_identify --run-dir <采集目录>              # 重解参数 + 训练集图
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --method ridge --ridge 1e-9
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --legacy      # 旧实现（交叉对照）
ros2 run lite_dyid dyid_validate --run-dir <采集目录>              # 重出验证残差与图

# 求解前会先按 LPF 低通（默认 0.2 Hz），并可用多进程：
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --lpf 0.2 --workers 16
ros2 run lite_dyid dyid_validate --run-dir <采集目录> --lpf 0.2 --workers 16
# ⚠️ 验证的 --lpf 必须与辨识一致，否则残差虚高（0.116 → 0.507 N·m）
```

`dyid_identify` 只读 CSV，**不碰机器人** —— 想换判据/换正则不用重新采数。

---

## 4. 产物说明

一次采集目录：

```
run_YYYYmmdd_HHMMSS/
├── robot.urdf                 采集时的 URDF（**冻结证据**：辨识必须用它，别用后来的）
├── run_meta.json              配置、轨迹名、起始姿态
├── small_check.csv            小幅验证逐关节结果
├── statics.csv                静力学（**平均后**）：每个静止段一行
├── statics_raw_<traj>.csv     静止段原始样本（排查用）
├── dynamics.csv               双臂同步训练轨迹的运动段原始样本
├── dynamics_<traj>.csv        分轨迹版本
├── params.json                辨识结果（重力参数 + 摩擦 + 拟合指标）
├── train_gravity_fit.png      静止样本：实测 vs 计算
├── train_friction.png         摩擦参数柱状图
└── validation/                （答 y 才有）
    ├── dynamics_validate.csv
    ├── residual_timeseries.png / torque_compare.png / residual_hist.png
    ├── residual_summary.png
    └── residual_stats.csv
```

### 4.1 CSV 列

| 文件 | 列 |
|---|---|
| `statics.csv` | `traj, static_index, motion_t, n_samples, duration_sec, q/<j>×14, tau/<j>×14, q_std/<j>×14, tau_std/<j>×14` |
| `dynamics.csv` | `traj, motion_t, wall_t, cmd/<j>×14, q/<j>×14, v/<j>×14, tau/<j>×14` |

- `q`/`v`/`tau` 是 **`/joint_states` 实测值**（关节坐标系）。
- `cmd` 是那一拍**下发的目标**，用于"命令 vs 实际"对比（不参与辨识）。
- 单位：rad、rad/s、N·m。
- ⚠️ 读取时**列数必须匹配**：`lite_dyid.dataset.read_table` 会校验，列数不符直接报错
  （错位会让每个数值看着都合理、其实对应别的关节）。

---

## 5. 辨识怎么算的（看懂输出）

### 5.1 第一段：静力学 → 重力（质量/质心）

静止时 `τ_meas = Y(q)·π`，其中 `π = [m_i, (m·c)_i]` 每连杆 4 个参数，
`Y` 由 pinocchio 用"单位惯性基"构造（见 `lite_dyid/model.py` 的文档）。

求解器默认 `method=bounded`：**质量 ≥ 0** 的带约束最小二乘（`scipy.optimize.lsq_linear`）。

**必须理解的限制（本包会明确打印）**：

```
数值秩 28/72
```

重力只能定出 **28 个独立组合**（≈每个关节 2 个），而参数有 72 个。也就是说：

- ✅ **重力力矩本身**被数据确定得很好（看 `拟合 RMS`）—— 补偿用它就够；
- ❌ **逐连杆的质量和质心不是唯一的**。打印出来的 `m_id` / `com_id` 只是
  "一组满足质量非负、且与数据一致"的解；换一组同样能完美解释数据。
- ❌ 连"整臂总质量"也**不可辨识**（只有 `Σm·c` 的某些组合可辨识）。

> ⚠️ 本机 URDF 的 `<inertial>` 是**占位值**（整机总质量 0.88 g），
> 所以脚本不会把它当先验（日志会写 `先验来源: 零（...判为占位值）`）。
> 如果你以后拿到了真实质量表，填进 URDF 后脚本会自动改用它当先验。

**要真正定出每根连杆的质量/质心**，需要：加速度激励（惯性项）+ 更多姿态，
即把 `dynamics.csv` 也用于惯性回归 —— 本包按需求只把动力学段当"重力+摩擦"。

### 5.2 第二段：动力学 → 摩擦（重力已冻结）

```
τ_fric(v) = c1⁺ (v>0) / c1⁻ (v<0) + c2·v
```

逐关节 3 参数最小二乘。`c2` 共用（不随符号变），`c1` 分速度符号。

默认模型不引入惯性项；需要惯量张量时用 `--with-inertia`。若残差里出现明显与
`v̇` 同相的分量，应对比带惯性模型。

---

## 5.3 怎么读验证图（本机一次真实结果的判读）

`validation/torque_compare.png` 是最有用的一张：蓝=实测、橙=计算（重力+摩擦）。

典型形态（本机实测）：

| 观察 | 含义 |
|---|---|
| 低频趋势（几十秒级的起伏）被橙色**跟得很紧** | 重力 + 库仑摩擦建对了 —— 这是模型真正抓住的部分 |
| 实测上叠着**高频波纹**（周期几秒，对应 4 次谐波），橙色没有 | **未建模的惯性/速度相关项**（M(q)q̈ + C(q,q̇)q̇）。本包按需求把动力学段当"重力+摩擦"，所以这部分一定在残差里 |
| 少数**尖峰**（肩部 ±1~3 N·m） | 速度过零附近的摩擦符号切换 / 静-动摩擦跳变。模型里的 `c1⁺/c1⁻` 是**阶跃**，而真实摩擦是平滑过渡 |
| 腕部残差呈**规则周期振荡** | 该关节力矩小、重力贡献弱，模型误差占比就大（相关系数也低，辨识日志会点名） |

**本机一次完整结果（可作基线）**：

```
全局残差 RMS = 0.229 N·m   max|·| = 2.97 N·m    n = 113456
逐关节 RMS 0.08 ~ 0.53 N·m（占额定 2.6% ~ 10.6%）
重力拟合 RMS（训练集）= 0.113 N·m，整体比例自检 k = 1.0000
```

想进一步压低残差，按性价比排序：

1. **加惯性项**：把 `dynamics.csv` 也用于 `M(q)q̈ + C(q,q̇)q̇` 的回归（需要加速度，用位置二次差分或滤波）；
2. **摩擦建模加 Stribeck / 黏滞-库仑混合**（`c1` 随 `|v|` 平滑过渡而不是阶跃）；
3. **降低速度/加速度**（让准静态假设更成立），但会更慢。



改 `src/lite_dyid/config/trajectories.yaml`（或先 `dyid_design --generate` 再改）：

| 想改什么 | 改哪里 | 注意 |
|---|---|---|
| 振幅更大/更小 | `amplitudes`（rad/关节） | 变大必须重跑 `dyid_design --check` 看限位与碰撞 |
| 更快/更慢 | `frequency_hz` | 当前基频 0.011 Hz；修改后必须重跑速度与碰撞校核 |
| 静止段数/时长 | `static_insert_count` / `static_hold_sec` / `static_period_sec` | 当前 15×6 s，额外增加 90 s 墙钟时间 |
| 双臂镜像 | `dyid_design.PHASES_LEFT` 与 `MIRROR_SIGN` | 两臂同时间运动；pitch 保号，roll/yaw 取反 |
| 前馈 | — | 本包不发前馈力矩（CSP 没有力矩入口） |

改 `src/lite_dyid/config/dyid.yaml`：

| 参数 | 说明 |
|---|---|
| `speed_cap` | 轨迹速度上限（同时决定进入段的斜坡时长） |
| `small_delta_rad` / `small_speed` | 小幅验证的幅度/速度（默认 8.6° / 0.1 rad/s） |
| `small_tolerance_rad` | 小幅验证的稳态误差上限 |
| `ridge` | 只作用于不可辨识方向；默认 1e-6（几乎不影响可辨识方向） |
| `collision_samples` | 运行期 mesh 碰撞检查的采样点数 |

---

## 7. 安全：怎么松开、怎么停

**CSP 没有"松劲"**：切到 mode5 会让电机**自由**（可能直接掉下来）。

```bash
# ① 想让它继续顶住（默认收尾状态）：什么都不用做，已在 CSP_HOLD

# ② 想让它自由（**先托住机械臂**）
ros2 service call /lite_bridge/set_mode robstride_msgs/srv/SetMode \
    "{protocol: 1, mode: 5, force: true}"

# ③ 安全待机
ros2 service call /lite_bridge/standby std_srvs/srv/Trigger "{}"

# ④ 排查/清故障后重试
ros2 service call /lite_bridge/clear_fault std_srvs/srv/Trigger "{}"
```

**任何时刻按 Ctrl+C**：`dyid_collect` 会停止下发（控制器保持最后目标，**不会塌**），
然后尝试切到 CSP_HOLD 并打印上面 ② 的命令。

**急停**：直接按硬件急停 / 断电。软件侧没有真正的急停入口。

---

## 8. 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| `服务不可用：/lite_bridge/set_mode` | bringup 是**旧构建**（还有 mode_manager、没有 set_mode）。重新 `colcon build` 后按 §1.3 重启 |
| `启动检查检测到 protocol=mit` | 停止当前 bringup，用 `controller:=csp control_mode:=csp source_names:=dyid` 重启；程序尚未下发位置命令 |
| `服务不可用：...` / spawner 报 `PermissionError: .../locks/ros2-control-controller-spawner.lock` | `~/.ros` 不可写（容器/沙箱）。`export ROS_HOME=/tmp/ros_home` 后再 launch |
| `命令只覆盖 x/14 个关节`（桥日志） | 本包永远发全 14；出现即说明有人在用 `ros2 topic pub` 发单关节命令 |
| 小幅验证某关节 ❌ | 该关节顶不动/被限位钳住/不在伺服态。`ros2 run lite_bridge motor_monitor --once` 看 目标/实际/力矩占额 |
| 关节**完全不动**但频率正常 | `csp_tracking_controller` 没 active。`ros2 control list_controllers` 核对 |
| `/joint_states 断流` | 驱动/广播器掉线。脚本会立即中止；先排障再重跑 |
| `level=2 (FAULT)` | 看 `ros2 topic echo /safety_status --once` 的 `flags`：位 1=RX 超时，位 2=TX 队列满。多半是**两套 bringup 抢总线**（§1.1） |
| 静止段 `σ_pos_max` 很大（>0.5°） | 没停稳：加大 `static_settle_sec`，或查该关节是否在抖动/啸叫 |
| 验证残差比预期大 3~4 倍 | **辨识与验证的 LPF 不一致**（模型在滤波数据上拟合、却拿未滤波实测去比）。两边都用 `--lpf 0.2` |
| 想确认滤波/求解没改结论 | `dyid_identify --legacy` 跑旧实现，对比 `params.json` 的 `friction` 与验证 RMS（应 ≤1e-8 / ≤1e-11） |
| 并行结果看着"正常但不对" | 检查 `fork_map` 的顺序回填（`test_fastsolve.py::test_fork_map_preserves_order` 是回归测试） |
| 重力 `k` 明显偏离 1 | 符号/量纲约定不对（例如混用了电机端与关节端角度）。**先别信任何残差**，查 `robot.urdf` 是否是采集时的那份 |
| 图是空的 / 报 `PermissionError: .../matplotlib-lock` | `~/.cache/matplotlib` 不可写。包内已自动回退到临时目录；仍失败就 `export MPLCONFIGDIR=/tmp/mpl` |
| 图里中文是方框 | 缺 CJK 字体。`sudo apt install fonts-noto-cjk`（包内已尝试常见字体名） |
| 残差整体偏大且与 `v̇` 同相 | 准静态假设不够：降速降加速度，或把惯性项也纳入回归 |

---

## 9. 已知限制

1. **逐连杆质量/质心不可辨识**（见 §5.1）。输出的是"一组一致解"，用于重力补偿没问题。
2. **URDF 无真实惯性参数**（占位值），所以没有有意义的先验，也无法用 URDF 交叉验证。
3. **本包按需求忽略了惯性项**（把动力学段当重力+摩擦）。要辨识惯量需另加回归。
4. **夹爪不参与辨识**：`mode:=arms` 下夹爪不在链路里，URDF 里它们的关节固定在 0，
   其质量会被合并进腕部连杆（pinocchio 的固定关节合并）。
5. **`left_wrist_pitch`（can0 dev 17）历史上会间歇性静默**（`docs/ROBSTRIDE_issue_report.md`）。
   小幅验证会把它暴露出来；若它偶发 ❌ 而其它关节都 ✓，先查这个电机。
6. **CSP 无法收到"松劲"**：任何异常下本包都不会自动切 mode5（那会让机械臂自由落体）。

---

## 10. 快速命令卡

```bash
# 状态
ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"
ros2 topic echo /safety_status --once
ros2 control list_controllers
ros2 run lite_bridge motor_monitor --once          # 目标/实际/误差 + 力矩占额%

# 手动切模式
ros2 service call /lite_bridge/set_mode robstride_msgs/srv/SetMode "{protocol: 1, mode: 5, force: false}"  # 监视(不使能)
ros2 service call /lite_bridge/set_mode robstride_msgs/srv/SetMode "{protocol: 1, mode: 6, force: false}"  # 保持(使能)
ros2 service call /lite_bridge/set_mode robstride_msgs/srv/SetMode "{protocol: 1, mode: 7, force: false}"  # 跟踪

# 本包
ros2 run lite_dyid dyid_design   --check --pose-file <run>/run_meta.json --urdf <run>/robot.urdf
ros2 run lite_dyid dyid_collect  --output-dir /tmp/dyid_runs
ros2 run lite_dyid dyid_identify --run-dir <run>
ros2 run lite_dyid dyid_validate --run-dir <run>
```

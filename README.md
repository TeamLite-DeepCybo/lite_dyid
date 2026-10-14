# lite_dyid

Lite 双臂 **CSP 动力学辨识**：傅里叶激励 + 插入静止段 → 重力（质量/质心）辨识 →
摩擦辨识 → 第三条轨迹验证与残差可视化。

```bash
ros2 run lite_dyid dyid_collect       # 预检 → 切 CSP → 小幅验证 → 采集 → 解算 → 问你是否验证
ros2 launch lite_dyid dyid.launch.py  # 同上（launch 包装）
```

---

## 0. 依赖与文件排布（**先读这一节**）

> ⚠️⚠️ **本包是 [`lite_core_controller`](https://github.com/marki-nishicono/lite_core_controller)
> 的配套包，不能独立运行。** 它只做"数据源 + 上位机解算"，**不提供**驱动、
> 控制器、桥、URDF、消息定义 —— 那些全在 `lite_core_controller` 里。

### 0.1 必须这样排布

本包必须位于 `lite_core_controller` **工作空间的 `src/` 下、目录名 `dyid`**：

```
lite_core_controller/
├── src/
│   ├── dyid/                    ← 本包（包名 lite_dyid，目录名 dyid）
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   ├── resource/lite_dyid
│   │   ├── lite_dyid/           ← Python 源码
│   │   ├── config/  launch/  docs/  test/
│   │   └── README.md            ← 本文件
│   ├── lite_bridge/             ← 桥（模式入口 + 多源仲裁）—— **必需**
│   ├── lite_controller/         ← 三态 + CSP 控制器插件 —— **必需**
│   ├── lite_driver/             ← ros2_control 硬件插件（写 CAN）—— **必需**
│   ├── lite_description/        ← URDF + mesh（本包用它建 pinocchio 模型）—— **必需**
│   ├── lite_bringup/            ← launch（CSP 组启动参数由它提供）—— **必需**
│   └── robstride_msgs/          ← MotionCommand / SetMode / GetBridgeStatus —— **必需**
└── build/ install/ log/
```

**为什么必须是这个名字和位置：**

| 约束 | 原因 |
|---|---|
| 目录名 `dyid` | `setup.py` 的 `data_files` 用相对路径；colcon 按目录发现包，与 `package.xml` 的 `<name>lite_dyid</name>` 解耦 |
| 必须在同一 `src/` 下 | 本包不 `import` 任何 `lite_*` 的 Python 模块，但**运行期**依赖它们的**话题/服务/URDF**；同一 colcon 工作空间才能保证 `source install/setup.bash` 后 `robstride_msgs` 与 `lite_description` 可用 |
| 不能放进 `src/` 更深层 | colcon 只递归一层 `src/*/` |

### 0.2 运行期依赖（缺一不可）

| 依赖 | 用途 | 缺失后果 |
|---|---|---|
| `robstride_msgs` | `MotionCommand` / `JointCommand` / `SetMode` / `GetBridgeStatus` | import 失败 |
| `lite_bridge` | **唯一模式入口** `~/set_mode`、命令仲裁、话题转发 | 无命令通路 |
| `lite_controller` | `csp_monitor` / `csp_hold` / `csp_tracking` 三个控制器 | 无接收方 |
| `lite_driver` | ros2_control 硬件插件（真正写 CAN） | 电机不动 |
| `lite_description` | URDF（本包用它建 pinocchio 模型 + 限位 + mesh 碰撞） | 无法建模 |
| `lite_bringup` | 提供 `controller:=csp control_mode:=csp source_names:=dyid` 的 launch | 环境起不来 |

**本包只当数据源，不改动上面任何一个包的一行。**

### 0.3 必须先起 CSP 组的 bringup

```bash
# 在 lite_core_controller 根目录
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms controller:=csp control_mode:=csp source_names:=dyid
```

三个参数缺一不可（漏 `source_names` → 桥丢弃本包所有命令；漏 `controller:=csp`
→ 加载的是 MIT 组，跑不了 CSP）。详见 [docs/RUNBOOK.md](docs/RUNBOOK.md) §1.3。

### 0.4 其它环境要求

- ROS 2 **Jazzy**
- Python 3.12（`/usr/bin/python3`，**不要用 anaconda 的 python3** —— 它没有 `yaml`）
- `numpy` / `scipy` / `matplotlib` / `yaml`
- ⚠️ **`pinocchio`（含 `hppfcl`）不是 rosdep key，需手工装**：
  ```bash
  /usr/bin/python3 -m pip install pin
  /usr/bin/python3 -c "import pinocchio, hppfcl; print('ok')"
  ```

---

## 1. 它做什么

```
        ┌── 预检 ─────────────────────────────────────────────┐
        │ /joint_states 齐备？safety=OK？抓 URDF？              │
        │ ensure_csp_ready(): 协议=CSP → mode6(HOLD) → mode7(TRACKING) │
        └──────────────────┬──────────────────────────────────┘
                           ▼
        ┌── 小幅运动验证（每关节 ≤8.6°、0.1 rad/s）────────────┐
        │ 逐关节：出去 → 停 1.5s → 回来；命令没到位就**中止**    │
        └──────────────────┬──────────────────────────────────┘
                           ▼
   ┌── 训练轨迹 ×2（左臂 / 右臂镜像）──────────────────────────┐
   │ 200 s 傅里叶运动（峰值 0.13 rad/s）                        │
   │  每 10 s 插入 10 s 静止 → 记录**平均**姿态与力矩 → 静力学 CSV │
   │ 运动段原始 q/v/τ → 动力学 CSV                              │
   │ 整条结束后**平滑回起始姿态**，再跑下一条                     │
   └──────────────────┬──────────────────────────────────────┘
                      ▼
   ┌── 自动解算（默认：静态 + 摩擦）──────────────────────────┐
   │ ① 静力学 → pinocchio 重力回归 → 每连杆 (m, m·c) → 冻结    │
   │ ② 动力学 → τ−τ_grav → 摩擦 c1⁺/c1⁻/c2                    │
   └──────────────────┬──────────────────────────────────────┘
                      ▼
   ┌── 询问：用第三条轨迹验证？──────────────────────────────┐
   │ 跑 120 s 验证轨迹 → 残差 = τ_实测 − (τ_grav + τ_fric)     │
   │ → validation/ 下出 5 张图 + residual_stats.csv          │
   └───────────────────────────────────────────────────────┘
```

---

## 2. 安装

```bash
cd <lite_core_controller>            # 工作空间根
colcon build --packages-select lite_dyid --symlink-install
source install/setup.bash
```

---

## 3. 命令行

### 3.1 `dyid_collect`（主流程）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--output-dir` | `~/.ros/lite_dyid/runs` | 结果根目录（每次自动建 `run_<时间戳>/`） |
| `--source` | `dyid` | 数据源名，必须在 `source_names` 里 |
| `--trajectories` | 包内 `config/trajectories.yaml` | 轨迹定义 |
| `--config` | 包内 `config/dyid.yaml` | 默认参数 |
| `--rate-hz` | 100 | 下发频率 |
| `--speed-cap` | 0.15 | 轨迹速度上限 rad/s |
| `--lpf` | **0.2** | 求解前的零相位低通截止频率 Hz（0 = 不滤） |
| `--workers` | **4** | 辨识并行进程数（1 = 串行） |
| `--with-inertia` | 关 | **可选**：额外辨识惯性项（模型 C/D），见 §6.3 |
| `--ridge` | 1e-6 | 正则系数（只作用于不可辨识方向） |
| `--small-delta` | 0.15 | 小幅验证单关节幅度 rad |
| `--skip-small-check` | 关 | 跳过小幅验证（**不推荐**） |
| `--traj` | 训练轨迹 | 只跑指定轨迹（逗号分隔） |
| `--auto-validate` | 关 | 不问，直接跑验证 |
| `--no-validate-prompt` | 关 | 不问，也不验证 |

### 3.2 其余入口

| 命令 | 用途 |
|---|---|
| `dyid_design --check` | 轨迹校核（限位/速度/肩部幅度/mesh 碰撞），**不碰机器人** |
| `dyid_design --generate <path>` | 重新生成轨迹定义（固定种子，可复现） |
| `dyid_identify --run-dir <dir>` | 离线重解（只读 CSV）；默认 `--lpf 0.2 --workers 4` |
| `dyid_validate --run-dir <dir>` | 离线重出验证残差与图 |
| `--legacy` | 用旧实现（`lsq_linear` 大矩阵）做交叉对照 |

```bash
ros2 run lite_dyid dyid_identify --run-dir <采集目录>                  # 默认：静态+摩擦
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --with-inertia    # 可选：+惯性项
ros2 run lite_dyid dyid_validate --run-dir <采集目录> --workers 16
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --legacy          # 交叉对照
```

---

## 4. 轨迹设计

| 约束 | 达成 |
|---|---|
| 肩部单关节变动 ≤ 50° | 实测 **≤ 35.5°** |
| 速度 ≈ 0.15 rad/s | 峰值 **0.130 rad/s**（f=0.035 Hz） |
| 速度变化慢 | 峰值加速度 **0.075 rad/s²** |
| 不越 URDF 限位 | 稳态余量 ≥ 0.10 rad；运行期仍逐点校核 |
| 当前 mesh 无碰撞 | pinocchio + hppfcl，排除父子/兄弟/夹爪对；运行期再抽查 400 个位形 |
| 前两条左右镜像 | `amplitudes` 镜像（pitch 保号、roll/yaw 取反），相位相同 |

**关键设计：`center` 在运行期才决定。** YAML 里存的是相对起始姿态的 `bias`：

```
center_j = clip(entry_j + bias_j,  lo_j + |amp_j| + margin,  hi_j − |amp_j| − margin)
```

因为每次上电姿态不同，而且本机左肩 roll 停在 −89.3°（下限 −90°，只剩 0.7° 余量）。
`|amp| = 0` 的关节（不激励的那条臂）**严格**保持在 `entry`。
进入段用 smoothstep 从实测起始姿态过渡，斜坡时长按 `speed_cap` 反算。

---

## 5. 求解前的低通（默认 LPF = 0.2 Hz）

**求解阶段**（不是采集阶段）会先对 `cmd / q / v / tau` 做**零相位** Butterworth 低通。
零相位很重要：有相位延迟的话 `τ` 与 `q` 会对不齐，辨识会被"时间错位"污染。

为什么是 0.2 Hz：本轨迹成分 = 基频 0.035 Hz 的 1/2/3/4 次谐波，最高 **0.14 Hz**，
0.2 Hz 是"能保住运动本身"的最低带宽；再低就开始削信号而不是噪声。

> ⚠️ **验证时必须用同一带宽**（`dyid_validate --lpf` 与辨识一致），否则模型在滤波
> 数据上拟合、却拿未滤波实测去比 —— 残差虚高（实测 0.116 → 0.507 N·m）。

加速度取自**参考轨迹**（下发 `cmd` 的二阶导），与实测信号独立 ——
所以滤波不会毁掉加速度信息。

### 5.1 为什么快

| 阶段（旧实现） | 秒 | 占比 |
|---|---|---|
| `solve_bounded` 联合 A_D (488698×150) | 17.75 | 53.5% |
| `solve_bounded` 惯性 A_I (488698×108) | 8.93 | 26.9% |
| build_regressor ×2 | 3.12 | 9.4% |
| 滤波 + 重采样 + SG | 0.64 | **1.9%** |
| **合计 / 峰值 RSS** | **33.2** | **5.0 GB** |

瓶颈是**求解器**而不是滤波。三个改动：

1. **默认只做静态+摩擦**：每样本只调一次 `computeGeneralizedGravity`，
   **完全不调 regressor**（最贵的一步，~35 µs/样本）。
2. **法方程替代稠密矩阵**：42×42（可选 108×108）小矩阵，`numpy.linalg.solve`
   微秒级；峰值 RSS 5 GB → **< 60 MB**。
3. **`os.fork` + 管道并行**（`--workers`，默认 4）：worker 只回传几十 KB 的法方程。
   不用 `multiprocessing` —— 它依赖 POSIX 信号量（在 `/dev/shm`），容器里常不可写。

| 实现 | 墙钟 | 加速 |
|---|---|---|
| 旧实现（`lsq_linear` 大矩阵，4 模型） | 35.9 s | 1.0× |
| 本内核 `--with-inertia --workers 1` | 5.5 s | 6.5× |
| 本内核 `--with-inertia --workers 16` | 1.8 s | 20.4× |
| **默认（静态+摩擦，`--workers 4`）** | **~3.3 s** | **~11×** |

结果与旧实现**逐项一致**（参数级 ≤2e-8，模型输出 ≤1e-11）。

> 若在 ROS 节点内遇到 fork 相关异常，把 `workers` 设回 1（`dyid_collect --workers 1`）。

---

## 6. 辨识（两段式）

### 6.1 第一段：静力学 → 重力

```
τ_meas = Y(q)·π ,   π = [m_i, (m·c)_i]  (每连杆 4 个)
```

`Y` 由"单位惯性基"构造：把某个连杆的惯性设成 `(m=1, c=0)` 或 `(m=1, c=e_a)`、
其余置零，直接调 `pinocchio.computeGeneralizedGravity`。

求解：**质量 ≥ 0** 的带约束最小二乘（`scipy.optimize.lsq_linear`，只有 560×72，本来就快）。

### 6.2 第二段：动力学 → 摩擦（**默认**）

```
τ_fric(v) = c1⁺(v>0) / c1⁻(v<0) + c2·v
```

冻结重力后逐关节线性最小二乘（3 参数/关节，共 42 个未知量）。

### ⚠️ 必读限制：逐连杆质量/质心不可辨识

重力数据只能定出 **28 个独立组合**（≈每关节 2 个），而参数有 **72 个**：

- ✅ **重力力矩**被数据确定得很好（看 `拟合 RMS`）—— 补偿用它足够；
- ❌ **逐连杆 m / c 不是唯一的**：输出的只是一组"质量非负、且与数据一致"的解；
- ❌ 连整臂总质量也不可辨识（只有 `Σm·c` 的某些组合可辨识）。

程序会把数值秩、奇异值谱、条件数、整体比例自检 `k` 全部打印出来。

> 本机 URDF 的 `<inertial>` 是**占位值**（整机 0.88 g），脚本检测到后会**不用它当先验**
> 并明确打印 `先验来源: 零（...判为占位值）`。

---

## 6.3 可选：静态 + 摩擦 + 惯性（`--with-inertia`）

默认**不开**。开启后额外辨识 108 个惯量张量分量（模型 C/D）：

```bash
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --with-inertia
ros2 run lite_dyid dyid_validate --run-dir <采集目录> --with-inertia   # 四个模型一起报
```

**本机实测（2026-10-13 采集，LPF=0.2）——惯性项不改善验证残差：**

| 模型 | 训练 RMS | 验证 RMS（LPF=0.2） |
|---|---|---|
| A 纯静力学 | 0.1785 | 0.1443 |
| **B 静力学+摩擦（默认）** | **0.1706** | **0.1157** |
| C 静力学+惯性项 | 0.1762 | **0.1487**（比 A 还**差** 3%） |
| D 静力学+摩擦+惯性 | 0.1690 | 0.1207 |

**为什么没用**（两点都能量化）：

1. **惯量项本身极小**：RMS **0.029 N·m**，只有残差（0.179 N·m）的 **16%** ——
   0.13 rad/s、0.075 rad/s² 的工况下 `τ = I·q̈` 本来就没多大；
2. **惯性回归严重欠定**：108 个未知量、数值秩只有 62、条件数 ~4e29。
   它会把训练集里别的未建模效应（摩擦非线性、重力模型误差）**吸收进惯量参数**，
   换一条轨迹就不成立（典型过拟合：训练集降、验证集升）。

**什么时候值得开：**

- 轨迹速度/加速度显著提高（例如峰值 > 0.5 rad/s 或 > 0.5 rad/s²），惯性项量级上来之后；
- 需要**惯量张量本身**（力矩前馈、碰撞检测、仿真标定）；
- 排查"残差里是否有与 `q̈` 同相的分量"。

> 带宽扫描（0 = 不滤 … 0.2 Hz）显示**模型排序在所有带宽下完全不变**：
> B 永远最低、C 永远最高。所以"加惯性项没用"不是带宽或噪声问题。

---

## 7. 产物

```
run_YYYYmmdd_HHMMSS/
├── robot.urdf                 采集时的 URDF（**冻结证据**：辨识必须用它）
├── run_meta.json              配置、轨迹名、起始姿态
├── small_check.csv            小幅验证逐关节结果
├── statics.csv                静力学（**平均后**）：每个静止段一行
├── statics_raw_<traj>.csv     静止段原始样本（排查用）
├── dynamics.csv               两条训练轨迹的运动段原始样本
├── dynamics_<traj>.csv        分轨迹版本
├── params.json                辨识结果（重力 + 摩擦；开了 --with-inertia 才有 inertia）
├── train_gravity_fit.png      静止样本：实测 vs 计算
├── train_friction.png         摩擦参数柱状图
└── validation/                （答 y 才有）
    ├── dynamics_validate.csv
    ├── residual_timeseries.png / torque_compare.png / residual_hist.png
    ├── residual_summary.png
    └── residual_stats.csv
```

CSV 列：`statics.csv` = `traj, static_index, motion_t, n_samples, duration_sec, q/<j>×14, tau/<j>×14, q_std/<j>×14, tau_std/<j>×14`；
`dynamics.csv` = `traj, motion_t, wall_t, cmd/<j>×14, q/<j>×14, v/<j>×14, tau/<j>×14`。
单位 rad / rad/s / N·m。读取时列数不匹配会**直接报错**（错位会让数值看着合理却对应错关节）。

---

## 8. 代码结构

```
src/dyid/
├── lite_dyid/
│   ├── common.py        常量（关节表、话题、模式、镜像符号）
│   ├── model.py         URDF → pinocchio：限位、碰撞对、重力回归基
│   ├── trajectory.py    傅里叶轨迹 + 运行期 center/ramp + 校核
│   ├── csp_client.py    /lite_bridge 的唯一出口（模式、命令、状态）
│   ├── motion.py        限速平滑移动 / 保持 / 节拍器
│   ├── dataset.py       CSV 读写（**校验列数**，防错位）
│   ├── fastsolve.py     **快速内核**：单遍滤波 + 流式法方程 + fork 并行
│   ├── collect.py       主流程（预检 → 采集 → 解算 → 询问）
│   ├── identify.py      重力 + 摩擦辨识 + 离线 CLI
│   ├── validate.py      第三条轨迹残差
│   ├── report.py        可视化（Agg，字体，缓存目录自动回退）
│   └── design.py        轨迹设计/校核工具
├── config/              dyid.yaml（默认参数）、trajectories.yaml（3 条轨迹）
├── launch/dyid.launch.py
├── docs/RUNBOOK.md      手动调试手册（上机前检查、故障排查、结果判读）
└── test/                47 个单测（**不需要真机**）
```

---

## 9. 测试

```bash
colcon test --packages-select lite_dyid          # 或
PYTHONPATH=src/dyid:$PYTHONPATH python3 -m pytest src/dyid/test -q
```

覆盖：轨迹限位/速度/肩部幅度、未激励关节严格不动、进入段无阶跃、斜坡时长、
镜像一致性、静止段时刻表；重力回归精确复现力矩、质量非负约束、摩擦系数恢复、
CSV 列数校验；**`fork_map` 并行顺序（回归测试）**、零相位滤波、法方程与旧求解器等价。

---

## 10. 安全设计

| 风险 | 行为 |
|---|---|
| 目标越限 | 运行期逐点校核；构造轨迹时就拒绝振幅容不下的关节 |
| 碰撞 | 设计期 + 运行期双重 mesh 检查，命中就拒绝执行 |
| 命令没到电机 | 小幅验证逐关节确认"实动 ≥ 50% 命令幅度"，不通过**立即中止** |
| `/joint_states` 断流 / `/safety_status` FAULT | 立即中止采集 |
| Ctrl+C / 异常 | 停止下发（控制器**保持最后目标，不会塌**），切 CSP_HOLD 并打印松开命令 |
| 预检阶段就失败 | **不切 CSP_HOLD** —— 那会把还没使能的电机凭空使能起来 |

> ⚠️ **CSP 没有"松劲"**：切到 mode5 会让电机自由、机械臂可能直接掉下来。
> 所以本包在任何异常下都**不会**自动切 mode5；要松开必须人工确认
> （命令见 [docs/RUNBOOK.md](docs/RUNBOOK.md) §7）。

---

## 11. 与 `lite_core_controller` 的版本对应

| 宿主 tag | 本包状态 |
|---|---|
| `base` | 首个版本：CSP 采集 + 重力/摩擦/惯性辨识（`lsq_linear` 旧求解器） |
| `lpf-fast` | 求解前 LPF=0.2 + 快速内核（法方程/流式/fork 并行） |
| 本仓库起点 | 默认模型改为**静态+摩擦**、`--workers 4`；惯性项改为 `--with-inertia` 可选 |

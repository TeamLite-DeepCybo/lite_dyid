# lite_dyid

Lite 双臂 **CSP 动力学辨识**：启动状态确认 → 小幅通路检查 → 双臂同步大幅
傅里叶激励 → 重力（质量/质心）与摩擦辨识。

```bash
ros2 run lite_dyid dyid_collect       # 确认 CSP → 小幅验证 → 双臂 300 s 采集 → 解算
ros2 launch lite_dyid dyid.launch.py  # 同上（launch 包装）
```

---

## 0. 依赖与文件排布（**先读这一节**）

> ⚠️⚠️ **本包是 [`lite_core_controller`](https://github.com/marki-nishicono/lite_core_controller)
> 的配套包，不能独立运行。** 它只做"数据源 + 上位机解算"，**不提供**驱动、
> 控制器、桥、URDF、消息定义 —— 那些全在 `lite_core_controller` 里。

### 0.1 必须这样排布

本包必须位于 `lite_core_controller` **工作空间的 `src/` 下、目录名 `lite_dyid`**：

```
lite_core_controller/
├── src/
│   ├── lite_dyid/               ← 本包（包名 lite_dyid，目录名 lite_dyid）
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
| 目录名 `lite_dyid` | `setup.py` 的 `data_files` 用相对路径；colcon 按目录发现包，与 `package.xml` 的 `<name>lite_dyid</name>` 一致（曾用名 `dyid`，已更名）|
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
   ┌── 一条双臂同步训练轨迹 ────────────────────────────────────┐
   │ 300 s 傅里叶运动；14 关节各覆盖至少 80% 有效区间             │
   │ 低频大幅、高频小幅；每 20 s 插入 6 s 静止                    │
   │ 运动段原始 q/v/τ → 动力学 CSV；静止段平均 → 静力学 CSV       │
   └──────────────────┬──────────────────────────────────────┘
                      ▼
   ┌── 自动解算（默认：静态 + 摩擦）──────────────────────────┐
   │ ① 静力学 → pinocchio 重力回归 → 每连杆 (m, m·c) → 冻结    │
   │ ② 动力学 → τ−τ_grav → 摩擦 c1⁺/c1⁻/c2                    │
   └──────────────────┬──────────────────────────────────────┘
   └───────────────────────────────────────────────────────────┘
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
| `--rate-hz` | **200** | 下发与记录循环频率；硬上限 200 Hz |
| `--speed-cap` | 0.15 | 轨迹速度上限 rad/s |
| `--lpf` | **0.2** | 求解前的零相位低通截止频率 Hz（0 = 不滤） |
| `--workers` | **4** | 辨识并行进程数（1 = 串行） |
| `--with-inertia` | 关 | **可选**：额外辨识惯性项（模型 C/D），见 §6.3 |
| `--ridge` | 1e-6 | 正则系数（只作用于不可辨识方向） |
| `--small-delta` | 0.15 | 小幅验证单关节幅度 rad |
| `--skip-small-check` | 关 | 跳过小幅验证（**不推荐**） |
| `--small-check-only` | 关 | 只做 CSP 启动检查和小幅验证，不进入 80% 大幅轨迹 |
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
| `dyid_tracking_validate [--run-dir <dir>]` | MIT 真机 A/B 跟踪验证：无辨识前馈 vs 重力+摩擦前馈 |
| `--legacy` | 用旧实现（`lsq_linear` 大矩阵）做交叉对照 |

```bash
ros2 run lite_dyid dyid_identify --run-dir <采集目录>                  # 默认：静态+摩擦
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --with-inertia    # 可选：+惯性项
ros2 run lite_dyid dyid_validate --run-dir <采集目录> --workers 16
ros2 run lite_dyid dyid_identify --run-dir <采集目录> --legacy          # 交叉对照
```

### 3.3 MIT 跟踪误差 A/B 验证

该入口自动选择最新一次含 `params.json` 与 `robot.urdf` 的辨识目录，也可以用
`--run-dir` 固定结果。两组均使用 `Kp=15, Kd=3` 和完全相同的 14 关节傅里叶轨迹；
频率为 `0.025/0.05 Hz`（倍率 2），每组默认 120 s。第一组前馈力矩为零，第二组
使用最新辨识的 `gravity(q_des)+friction(v_des)`。

若当前 bringup 是 CSP，第一次运行只会检测状态并切回 `CSP_MONITOR`，然后退出并
打印 MIT 重启参数。协议不能在运行中切换，因此按提示重启后再次运行同一命令：

```bash
ros2 run lite_dyid dyid_tracking_validate

# 若上一步提示当前为 CSP，停止 bringup 后重启：
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms controller:=modes control_mode:=mit source_names:=dyid

# MIT bringup 就绪后再次运行；也可显式指定辨识目录
ros2 run lite_dyid dyid_tracking_validate \
    --run-dir /tmp/dyid_runs/run_YYYYmmdd_HHMMSS
```

默认输出目录为持久路径
`~/.ros/lite_dyid/tracking_validations/<辨识运行名>/tracking_validation_<时间>/`；
也可以用 `--output-dir` 明确指定。目录包含两组原始 CSV、两张实际/期望关节角度图、
逐关节误差对比图、`tracking_error_stats.csv`、`summary.json` 和产物完整性清单
`artifacts.json`。程序只有在全部必需文件存在且非空后才打印“完成”。运行前会校验
URDF 机械语义、限位、速度、mesh 碰撞和前馈额定力矩。
其中在线 URDF 采用 XML 语义校验：允许验证所必需的 `control_mode: csp→mit`
变化及纯格式差异，关节、限位、传动、惯性或几何变化仍会拒绝。

---

## 4. 轨迹设计

| 约束 | 达成 |
|---|---|
| 默认轨迹 | `dyid_train_bimanual`，左右臂在同一时间窗运动 |
| 运动时间 | **300 s**（插入的静止保持不消耗运动时间轴） |
| 行程覆盖 | 14 个关节各 **≥80%**；现场 URDF 运行前再次硬校核 |
| 频谱 | 基频 0.011 Hz；1～4 次谐波权重 `1.00/0.35/0.15/0.05` |
| 速度 | 各关节峰值 ≥0.05 rad/s；肩/肘 ≤0.2，腕 ≤0.4 rad/s |
| 碰撞 | 固定相位将多关节极值错开；运行前对 300 s 轨迹检查 6001 个位形 |
| 下发 | 默认 200 Hz；每帧覆盖全部 14 个关节 |

`peak_normalize` 按每个关节一个完整基频周期的最小值/最大值把波形精确映射到
`[-1,+1]`。因此 YAML 的 `amplitudes` 是实际半行程，`0.4 × (upper-lower)`
对应 80% 峰峰值覆盖。工作点取 URDF 限位中点，进入段用 smoothstep 从实测姿态
平滑过渡；运行期仍逐拍钳位，并要求 `clamp_count == 0`。

默认配置就是交付的大幅版本，没有额外的标准档、左右单臂档或小幅训练轨迹。
唯一保留的小幅运动是正式采集前逐关节 8.6° 的通路检查。

### 离线校核

```bash
ros2 run lite_dyid dyid_design --generate \
    /tmp/trajectories.yaml

ros2 run lite_dyid dyid_design --check --urdf <展开后的 urdf> \
    --trajectories /tmp/trajectories.yaml
```

---

## 5. 求解前的低通（默认 LPF = 0.2 Hz）

**求解阶段**（不是采集阶段）会先对 `cmd / q / v / tau` 做**零相位** Butterworth 低通。
零相位很重要：有相位延迟的话 `τ` 与 `q` 会对不齐，辨识会被"时间错位"污染。

为什么是 0.2 Hz：本轨迹成分 = 基频 0.011 Hz 的 1/2/3/4 次谐波，最高 **0.044 Hz**，
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
   这是旧版小幅轨迹数据；当前 80% 交付轨迹需要重新采集后再评估；
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
├── dynamics.csv               双臂同步训练轨迹的运动段原始样本
├── dynamics_<traj>.csv        分轨迹版本
├── params.json                辨识结果（**schema 2**，见 §7.1）
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

### 7.1 `params.json` 格式（schema 2）

```jsonc
{
  "schema": 2,
  "source": { "package": "lite_dyid", "model": "static_friction_only", "model_desc": "…" },
  "robot":  { "urdf_sha256": "…", "joints": [14 个关节，顺序 = canonical], "links": [...] },
  "inertia": {
    "mode": "static_friction_only",        // ★ 用的是哪个辨识模型
    "mode_desc": "静态+摩擦（惯量张量全部置 0；质量与一阶矩来自静态辨识）",
    "usable_for_mass_matrix": false,       // ★ 下游据此 fail-closed：false = 不许算 M(q)
    "links": {
      "left_elbow_pitch": {
        "identified": ["mass","mx","my","mz"],
        "mass": 0.605986,                  // null = 不可辨识（下游取 1.0）
        "com":  [0.232410, 0.283940, -0.271983],
        "I":    [[0,0,0],[0,0,0],[0,0,0]]  // ★ 静态+摩擦时**全 0**
      }
    }
  },
  "joints": [...], "links": [...],          // ↓ 以下与 schema 1 **完全一致**（向后兼容）
  "gravity": { "params": [...], "masses": [...], "coms": [...], "rank": …, "cond": … },
  "friction": { "c1_pos": [...], "c1_neg": [...], "c2": [...] },
  "fit": { "gravity_rms_joint": [...], "friction_rms_joint": [...], "scale_check": … },
  "meta": { "model": "static_friction_only", "rate_hz": 200.0, "trajectories": [...] }
}
```

**三条必须守住的语义**（都有单测 `test/test_schema2.py` 钉住）：

| # | 语义 | 为什么 |
|---|---|---|
| 1 | `static_friction_only` 时 **`I` 全写 0** | 惯量根本没辨识，填任何"看起来合理"的值都是编造 |
| 2 | 但 **`mass` 与 `com` 必须保留** | 重力矩 `τ_g = Σ Jᵀ(m·g)` 全靠它；置 0 ⇒ `g(q)≡0` ⇒ 前馈完全失效 |
| 3 | **必须注明 `mode`**，并给出 `usable_for_mass_matrix` | 下游（`lite_move_controller` 的导纳/阻抗）能 **fail-closed** 地拒绝"需要 M(q) 但没有惯性"的用法，而不是静默退化 |

> ⚠️ `--with-inertia` 时 `usable_for_mass_matrix=true`，但逐分量**仍非唯一**
> （数值秩 < 参数个数），所以 `I` 的正确用法是"整体写回模型后用 `rnea`"，
> 而不是单独解读某个 `Ixx`。原始拟合量放在 `inertia.raw_fit` 里备查。

---

## 8. 代码结构

```
src/lite_dyid/
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
│   ├── validate.py      可选外部验证数据的残差
│   ├── tracking_validate.py  MIT 两频轨迹的无/有辨识前馈跟踪对比
│   ├── report.py        可视化（Agg，字体，缓存目录自动回退）
│   └── design.py        轨迹设计/校核工具
├── config/              dyid.yaml（默认参数）、trajectories.yaml（唯一交付轨迹）
├── launch/dyid.launch.py
├── docs/RUNBOOK.md      手动调试手册（上机前检查、故障排查、结果判读）
└── test/                离线单测（**不需要真机**）
```

---

## 9. 测试

```bash
colcon test --packages-select lite_dyid          # 或
PYTHONPATH=src/lite_dyid:$PYTHONPATH python3 -m pytest src/lite_dyid/test -q
```

覆盖：14 关节至少 80% 行程、速度上下限、双臂同步、进入段无阶跃、
镜像一致性、静止段时刻表；MIT 启动拒绝和 CSP 模式回读；重力回归、摩擦系数恢复、
CSV 列数校验；**`fork_map` 并行顺序（回归测试）**、零相位滤波、法方程与旧求解器等价。

---

## 10. 安全设计

| 风险 | 行为 |
|---|---|
| 启动时仍是 MIT | 在第一帧位置命令前停止，打印 CSP bringup 重启参数 |
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

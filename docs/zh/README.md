# 七自由度机械臂动力学 / 静力学参数辨识

基于 **Pinocchio** 的辨识算法 + 可选 **Isaac Lab** 物理采集层。算法层（回归矩阵、QR 基参数、OLS / Huber / 鲁棒 WLS）与数据层（Pinocchio 合成或 Isaac 采集）通过统一 NPZ 接口解耦，切换数据源时**无需修改核心辨识代码**。

| 层次 | 目录 | 说明 |
|------|------|------|
| 算法层 | `param_id/` | 回归器、列主元 QR、OLS / Huber / 白化鲁棒 WLS |
| 数据层 | `utils/` | NPZ 读写、轨迹、PD 控制律、噪声、重力前馈 |
| 采集层 | `scripts/collect_data_isaaclab.py` | 位置伺服 / PD 力矩、摩擦、传感器噪声 |
| 验证层 | `scripts/compare_*`、`verify_gravity_compensation.py` | 基准对齐、交叉对比、重力补偿闭环 |

> **说明：** 默认 Pinocchio 路径从 URDF 真值合成力矩（可加噪声/异常点），用于**方法与管线验证**，不代表真机辨识精度。

**English:** [`../../README.md`](../../README.md) · **算法原理：** [`METHOD.md`](METHOD.md) · **自学：** [`LEARNING.md`](LEARNING.md)

---

## 文档导航

| 文档 | 内容 |
|------|------|
| [`README.md`](README.md) | 本页：项目总览、环境、Isaac Demo、工作流 |
| [`METHOD.md`](METHOD.md) | **算法原理**（回归器、QR、鲁棒估计） |
| [`LEARNING.md`](LEARNING.md) | **自学实操**（逐步跑通 + 理论↔代码对照） |
| [`BASELINE_ALIGNMENT.md`](BASELINE_ALIGNMENT.md) | Pinocchio vs Isaac 理想工况对齐（&lt; 5%） |
| [`RESULTS_ANALYSIS.md`](RESULTS_ANALYSIS.md) | 如何读对比图与重力补偿结果 |
| [`UPLOAD.md`](UPLOAD.md) | 公开仓库推送前检查 |
| [`../README.md`](../README.md) | 文档总索引（中英对照） |
| [`../../results/README.md`](../../results/README.md) | `results/` 输出目录说明 |

英文原文在 `docs/` 根目录（如 `docs/METHOD.md`）；中文在 `docs/zh/`。

**建议阅读顺序：** 本 README → [`METHOD.md`](METHOD.md) → [`LEARNING.md`](LEARNING.md) → [`RESULTS_ANALYSIS.md`](RESULTS_ANALYSIS.md)

---

## 整体流程

```text
URDF（教学 7-DoF 或自有机械臂）
        │
        ├─ Pinocchio 合成力矩 ──┐
        │   (RNEA + 可选噪声)   │
        │                       │
        └─ Isaac Lab 采集 ──────┼──► 统一 NPZ {q, dq, ddq, tau, dt, traj_type}
                                │
                                ▼
                   identify_static / identify_dynamic
                   (--data-source pinocchio | file)
                                │
                                ▼
                     列主元 QR → OLS / Huber / robust WLS
                                │
                   ┌────────────┼────────────┐
                   ▼            ▼            ▼
             图表 / NPZ   results/comparison/   重力补偿验证
```

---

## 仓库结构

```text
param_id/                          # 项目根目录
│
├── param_id/                      # 核心辨识包
│   ├── regressor.py               # 重力 / 动力学 / 摩擦回归矩阵
│   ├── base_params.py             # 列主元 QR 基参数选取
│   ├── estimators.py              # OLS、Huber-IRLS、白化鲁棒 WLS
│   ├── trajectory.py              # 傅里叶 / 余弦激励轨迹
│   └── robot_model.py             # URDF 解析、Pinocchio 模型构建
│
├── utils/                         # 数据层（与算法解耦）
│   ├── data_io.py                 # NPZ 读写与校验
│   ├── traj_generator.py          # 轨迹生成门面
│   ├── ddq.py                     # 理想 / 实测加速度
│   ├── collect_extras.py          # PD 控制律、高斯噪声、异常点
│   └── gravity_comp.py            # URDF 或辨识结果的重力矩
│
├── scripts/                       # 命令行入口
│   ├── 00_sanity_check.py         # 回归器 vs RNEA 自检
│   ├── identify_static.py         # 静力学辨识（重力 + 库仑摩擦）
│   ├── identify_dynamic.py        # 动力学辨识（惯量 + 摩擦）
│   ├── collect_data_isaaclab.py   # Isaac Lab 数据采集
│   ├── export_pinocchio_dataset.py# 导出理想 Pinocchio NPZ
│   ├── compare_baseline_alignment.py
│   ├── run_comparison.py          # 三组交叉对比
│   ├── verify_gravity_compensation.py
│   ├── collision_view.py          # 轨迹碰撞检查 / 可视化
│   ├── run_demo.sh                # 一键 Pinocchio 演示
│   ├── run_baseline_alignment.sh
│   └── run_comparison_experiments.sh
│
├── models/demo_7dof/              # 公开教学 URDF（MIT）
│   └── demo_arm.urdf
│
├── configs/
│   └── comparison_experiment.yaml # 交叉对比实验参数
│
├── assets/proprietary/            # 本地私有 CAD（已 gitignore）
│   └── README.md
│
├── docs/                          # 文档（见 docs/README.md）
│   ├── METHOD.md, LEARNING.md, BASELINE_ALIGNMENT.md, UPLOAD.md
│   └── zh/                        # 中文文档（本目录）
│       ├── README.md
│       └── RESULTS_ANALYSIS.md
│
├── results/                       # 本地输出（见 results/README.md）
│   ├── examples/                  # 已提交的示例图
│   ├── comparison/                # 交叉对比展示
│   ├── gravity_comp/              # 重力补偿验证
│   └── baseline/                  # 基准对齐 NPZ 与指标
│
├── environment.yml                # Conda 环境（仅 Pinocchio）
├── requirements.txt               # Pip 依赖（Isaac 环境补充）
└── LICENSE
```

`meshes/` 与 `assets/proprietary/urdf/` 已加入 gitignore，**请勿将公司 CAD 推送到公开仓库**。

---

## 环境配置

```bash
cd /path/to/param_id

# 方案 A：仅 Pinocchio（轻量）
conda env create -f environment.yml
conda activate param-id

# 方案 B：含 Isaac Lab（推荐，支持 PhysX 采集）
conda activate env_isaaclab
pip install -r requirements.txt   # 缺啥补啥
```

指定 URDF：

```bash
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

**Cursor / VS Code：** 解释器选  
`/home/zj/miniconda3/envs/env_isaaclab/bin/python`  
（不要用系统 `/bin/python3`，否则会缺 `matplotlib` / `pinocchio`）。

---

## 快速上手（纯 Pinocchio，无需 Isaac）

```bash
bash scripts/run_demo.sh
```

或分步执行：

```bash
python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls --outlier-ratio 0.05
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05 --n-periods 3
python scripts/collision_view.py   --traj fourier --check-only
```

输出在 `results/`（NPZ + PNG）。自定义机型：`--urdf /path/to/arm.urdf`。

---

## Isaac Lab Demo（带 GUI / 无头）

本仓库**没有**单独的 Isaac 官方场景包。演示入口就是  
`scripts/collect_data_isaaclab.py`（加载 URDF → 跟踪激励轨迹 → 写出 NPZ）。

### 前置

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

> **导入顺序：** 脚本必须在 `AppLauncher` **之前**导入 Pinocchio。  
> 若先启动 Kit 再加载 Pinocchio，会出现 `Startup Complete` 后立刻  
> `Shutting Down`（pybind 冲突）。不要改动脚本里的 import 顺序。

### A) 带 GUI — 看机械臂运动

**去掉** `--headless`，Isaac Sim 会开窗口：

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --control-mode position_servo \
  --save-path results/baseline/isaac_demo_fourier.npz
```

可选：`--device cuda:0`。录制结束或关窗后，NPZ 写在 `results/baseline/`。

### B) 无头 — 只采数据

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --headless \
  --save-path results/baseline/isaac_demo_fourier.npz
```

### C) 工程化采集（更接近真机）

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 2 --dt 0.01 --seed 0 \
  --control-mode pd_torque --kp 200 --kd 20 \
  --enable-friction --q-noise-std 1e-4 --tau-noise-std 0.05 \
  --ddq-mode ideal --headless \
  --save-path results/baseline/isaac_eng_fourier.npz
```

若要带 GUI，同一命令去掉 `--headless` 即可。

### D) 用采集到的 NPZ 辨识

```bash
python scripts/identify_dynamic.py --method robust_wls \
  --data-source file --data-path results/baseline/isaac_demo_fourier.npz \
  --results-dir results/isaac_dynamic
```

### E) 重力补偿 Demo（Isaac）

```bash
# 无头：保持姿态 + 末端拖拽指标
python scripts/verify_gravity_compensation.py --headless \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# 带 GUI：去掉 --headless
python scripts/verify_gravity_compensation.py \
  --id-result results/static_ols.npz --out-dir results/gravity_comp
```

纯离线（不启 Isaac）：加 `--offline`。

---

## 统一数据格式

`utils/data_io.py` 定义的 NPZ 字段：

| 字段 | 类型 | 形状 / 含义 |
|------|------|-------------|
| `q` | `float64` | `(N, n_joint)` 关节角 [rad] |
| `dq` | `float64` | `(N, n_joint)` 角速度 |
| `ddq` | `float64` | `(N, n_joint)` 角加速度 |
| `tau` | `float64` | `(N, n_joint)` 关节力矩 [N·m] |
| `dt` | `float` | 采样周期 [s] |
| `traj_type` | `str` | 轨迹标签 |

```python
from utils.data_io import save_dataset, load_dataset
```

---

## 工作流

### 1. 静力学辨识（重力补偿）

| | |
|--|--|
| 轨迹 | 低速余弦 |
| 估计量 | 重力相关项 + 库仑摩擦 |
| 用途 | 重力前馈 |

```bash
python scripts/identify_static.py --method robust_wls
```

### 2. 动力学辨识（力矩前馈）

| | |
|--|--|
| 轨迹 | 傅里叶激励 |
| 估计量 | 惯量 + 科氏/离心 + 重力 + 库仑/粘性摩擦 |
| 用途 | 模型力矩控制 |

```bash
python scripts/identify_dynamic.py --method robust_wls --n-periods 3
```

### 3. Isaac Lab 数据采集（参数表）

需要 **Isaac Lab**（测试环境约 0.54 / Isaac Sim 5.x）。完整 GUI / 无头命令见上文 **[Isaac Lab Demo](#isaac-lab-demo带-gui--无头)**。

| 参数 | 说明 |
|------|------|
| `--headless` | 无 GUI。**去掉此参数** 即可开窗看机械臂 |
| `--control-mode` | `position_servo`（隐式 PD）/ `pd_torque`（显式 τ = Kp·e + Kd·ė） |
| `--ideal-physics` | 仅重力，关关节摩擦（基准对齐用） |
| `--enable-friction` | PhysX 静摩擦 / 库仑 / 粘性 |
| `--ddq-mode ideal\|measured` | 轨迹 ddq 或对 dq 中心差分 + 滑动平均 |
| `--q-noise-std` / `--tau-noise-std` | 位置 / 力矩传感器噪声 |
| `--data-source file` | 在 identify_* 中加载 NPZ，而非脚本内合成 |

### 4. 基准对齐（理想物理）

Pinocchio（RNEA、无摩擦）与 Isaac（`--ideal-physics`）在相同激励下对比，通过门槛：基参数相对误差 **&lt; 5%**。

```bash
bash scripts/run_baseline_alignment.sh
# 详见 ../BASELINE_ALIGNMENT.md
```

### 5. 交叉对比实验

三组对照 → `results/comparison/`：

1. **基准组** — Pinocchio 理想数据 + OLS
2. **物理组** — 工程数据（PD + 摩擦 + 噪声）+ OLS
3. **鲁棒组** — 同一工程数据 + robust WLS

```bash
bash scripts/run_comparison_experiments.sh          # 优先 Isaac，失败则代理数据
bash scripts/run_comparison_experiments.sh --skip-isaac   # 纯离线
```

产出：`fig_rmse_per_joint.png`、`fig_rel_param.png`、`fig_torque_joint0.png`、`conclusion.md`、`summary.json`。

### 6. 重力补偿闭环验证

```bash
# 离线残差（无需 Isaac）
python scripts/verify_gravity_compensation.py --offline \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# Isaac 无头：保持姿态 + 末端拖拽
python scripts/verify_gravity_compensation.py --headless \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# Isaac 带 GUI：去掉 --headless
python scripts/verify_gravity_compensation.py \
  --id-result results/static_ols.npz --out-dir results/gravity_comp
```

前馈律：`τ = Kp(q*−q) + Kd(dq*−dq) + τ_g(q)`，`τ_g` 来自 URDF RNEA 或辨识的 `π̂_g`。

---

## 如何读结果

| 实验 | 看什么 |
|------|--------|
| 理想基准组 | 力矩 RMSE ≈ 0，基参数误差 ≈ 0% → 管线正确 |
| 工程 OLS | 全样本 RMSE 高（含异常点）；**内点 RMSE** 约 0.24 N·m |
| 工程 robust WLS | **内点 RMSE** 约 0.05 N·m（比 OLS 约好 5 倍） |
| 重力补偿离线 | 辨识重力补偿残差 ≈ 机器精度 |

### 关于「基参数相对误差 300%」

工程组力矩 = 惯量动力学 + **摩擦** + 噪声 + 稀疏异常，但对照真值是「URDF 惯量 + **摩擦=0**」。辨识器会把摩擦也拟合进参数向量，与无摩擦真值相比，相对误差被系统性拉到 200%–300%。

**这不代表辨识失败。** 工程上应看：

- **力矩内点 RMSE**（OLS 0.24 → robust 0.05）
- **重力补偿残差**

详见 [`RESULTS_ANALYSIS.md`](RESULTS_ANALYSIS.md)。

示例图：[`../../results/examples/`](../../results/examples/)、[`../../results/comparison/`](../../results/comparison/)。

---

## 两条辨识链路对照

| | 静力学 | 动力学 |
|--|--------|--------|
| 轨迹 | 低速 cosine | 傅里叶 |
| 回归 | 重力 + 库仑摩擦 | 惯量 + 科氏/离心 + 重力 + 库仑/粘性 |
| 典型用途 | 重力补偿 | 力矩前馈 |
| 推荐估计器 | robust_wls（含异常点时） | robust_wls |

---

## 公开仓库注意事项

- `meshes/`、`assets/proprietary/` 下机型文件已 gitignore
- 推送前确认无未授权 URDF / 网格（见 [`../UPLOAD.md`](../UPLOAD.md)）
- 仿真误差 ≠ 真机辨识精度

---

## 作者与许可

**仝小可 (Tong Xiaoke)** — 东北大学控制科学与工程  
GitHub: [txk1228](https://github.com/txk1228)

MIT License — 见 [LICENSE](../../LICENSE)

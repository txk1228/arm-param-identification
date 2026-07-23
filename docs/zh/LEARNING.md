# 学习指南 — 自己跑通整条管线

目标：在教学（或自有）URDF 上跑完仿真流程，弄懂  
\(τ = Yπ\)、QR 基参数、Huber / 白化 WLS、傅里叶激励。

英文原文：[`../LEARNING.md`](../LEARNING.md)

---

## 0. 环境

```bash
conda activate param-id   # 或 env_isaaclab
cd <repo-root>
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

目录结构（完整树见 [中文 README](README.md) / [文档索引](../README.md)）：

```
param_id/                 # Python 包（算法）
utils/                    # 数据 I/O 层
scripts/                  # 命令行入口
models/demo_7dof/         # 公开教学 URDF
assets/proprietary/       # 本地私有 URDF/网格（已 gitignore）
docs/                     # 文档索引 → docs/README.md
results/                  # 运行输出 → results/README.md
```

私有多自由度 CAD（若有）放在 `assets/proprietary/`，**不要提交**。  
公开默认模型：`models/demo_7dof/demo_arm.urdf`。

---

## 1. 建议顺序

### Step 0 — 回归器一致性（必做）

```bash
python scripts/00_sanity_check.py
```

两项残差都应接近 `0`：

| 检查 | 公式 |
|------|------|
| 静力学 | \(τ_g = Y_g(q) π_g\)（数值 RNEA 列） |
| 动力学 | \(τ = Y(q,\dot q,\ddot q) π\)（`computeJointTorqueRegressor`） |

### Step 1 — 静力学辨识

```bash
python scripts/identify_static.py --method ols --outlier-ratio 0.05
python scripts/identify_static.py --method huber --outlier-ratio 0.05
python scripts/identify_static.py --method robust_wls --outlier-ratio 0.05
```

关注：

1. QR：重力列被压缩（例如 28 → 约 12–14）
2. 日志末尾的 **重力补偿误差**（有异常点时比全样本 RMSE 更有意义）
3. `results/static_*.png` — 红点是注入的尖峰

### Step 2 — 动力学辨识

```bash
python scripts/identify_dynamic.py --method ols --n-periods 3
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05
```

关注：

1. `results/dynamic_*.png` 里的傅里叶 \(q\) 曲线
2. 全参数 → 基参数维数
3. 内点力矩 RMSE vs 全样本 RMSE
4. 真值 vs 估计的基参数柱状图

### Step 3 — 轨迹碰撞回放

不是辨识本身，而是激励轨迹的几何可行性检查。

```bash
python scripts/collision_view.py --traj fourier --check-only
python scripts/collision_view.py --traj cosine --check-only
python scripts/collision_view.py --traj cv --check-only
python scripts/collision_view.py --traj fourier --view
```

操作：打开后自动播放；`SPACE` 暂停/继续；`N`/`B` 单步；方向键环视；`ESC` 退出。  
若总是撞 `base_link`，调试时可试 `--amplitude-scale 0.4` 或 `--ignore-base`。

### Step 4 — 消融实验

| 实验 | 改动 | 预期 |
|------|------|------|
| 更弱激励 | 更小的 `--amplitude-scale` | 条件更差 / 误差更大 |
| 更脏数据 | `--outlier-scale 80 --outlier-ratio 0.1` | OLS 变差；鲁棒方法更扛 |
| 更长轨迹 | `--n-periods 10` | 估计更稳，更慢 |

---

## 2. 代码 ↔ 算法对照

| 概念 | 位置 |
|------|------|
| 数值 \(Y_g\)（RNEA） | `param_id/regressor.py` → `gravity_regressor_numeric` |
| JointTorqueRegressor + 摩擦 | `param_id/regressor.py` → `dynamics_regressor` |
| 列主元 QR 基参数 | `param_id/base_params.py` |
| OLS / Huber / 白化 WLS | `param_id/estimators.py` |
| 傅里叶 + 异常点 | `param_id/trajectory.py` + `scripts/identify_*.py` |
| URDF 加载 | `param_id/robot_model.py` |

伪惯量 SDP 约束（cvxpy）**默认未启用**。主路径：  
QR + robust WLS；SDP 可之后作为可选后处理加上。

算法原理详解见 [`METHOD.md`](METHOD.md)。

---

## 3. 本仿真的适用范围

- 力矩由 URDF 真值参数合成，再叠加噪声/异常点。
- **不要**把仿真 RMSE 当成真机辨识精度。
- 原理说明另见 [`METHOD.md`](METHOD.md)。

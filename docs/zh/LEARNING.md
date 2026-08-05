# 管线复现与模块对照

按阶段复现完整辨识流程，并对照 \(τ = Yπ\)、QR 基参数、Huber / 白化 WLS、傅里叶激励在代码中的实现位置。

英文原文：[`../LEARNING.md`](../LEARNING.md)

---

## 0. 环境

```bash
conda activate param-id   # 或 env_isaaclab
cd <repo-root>
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

目录结构（完整树见 [中文 README](../../README.md) / [文档索引](../README.md)）：

```
param_id/                 # 算法包
utils/                    # 数据 I/O
scripts/                  # CLI 入口
models/demo_7dof/         # 公开教学 URDF
assets/proprietary/       # 本地私有 URDF/网格（已 gitignore）
docs/                     # 文档索引 → docs/README.md
results/                  # 运行输出 → results/README.md
```

私有 CAD 置于 `assets/proprietary/`，勿提交。公开默认模型：`models/demo_7dof/demo_arm.urdf`。

---

## 1. 复现顺序

流程按依赖递进：回归一致性 → 静力学 → 动力学 → 轨迹可行性 → 消融。

### 阶段 0 — 回归器一致性

```bash
python scripts/00_sanity_check.py
```

两项残差应接近 `0`：

| 检查 | 公式 |
|------|------|
| 静力学 | \(τ_g = Y_g(q) π_g\)（数值 RNEA 列） |
| 动力学 | \(τ = Y(q,\dot q,\ddot q) π\)（`computeJointTorqueRegressor`） |

### 阶段 1 — 静力学辨识

```bash
python scripts/identify_static.py --method ols --outlier-ratio 0.05
python scripts/identify_static.py --method huber --outlier-ratio 0.05
python scripts/identify_static.py --method robust_wls --outlier-ratio 0.05
```

检查项：

1. QR：重力列压缩（例如 28 → 约 12–14）
2. 日志末尾的**重力补偿误差**（含异常点时优于全样本 RMSE）
3. `results/static_*.png` — 红色标记为注入尖峰

### 阶段 2 — 动力学辨识

```bash
python scripts/identify_dynamic.py --method ols --n-periods 3
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05
```

检查项：

1. `results/dynamic_*.png` 中的傅里叶 \(q\) 轨迹
2. 全参数 → 基参数维数
3. 内点力矩 RMSE vs 全样本 RMSE
4. 真值 vs 估计基参数柱状图

### 阶段 3 — 轨迹碰撞检查

激励轨迹的几何可行性验证（非辨识本身）。

```bash
python scripts/collision_view.py --traj fourier --check-only
python scripts/collision_view.py --traj cosine --check-only
python scripts/collision_view.py --traj cv --check-only
python scripts/collision_view.py --traj fourier --view
```

交互：打开后自动播放；`SPACE` 暂停/继续；`N`/`B` 单步；方向键环视；`ESC` 退出。  
若持续碰撞 `base_link`，可试 `--amplitude-scale 0.4` 或 `--ignore-base`。

### 阶段 4 — 消融实验

| 实验 | 改动 | 预期 |
|------|------|------|
| 弱激励 | 减小 `--amplitude-scale` | 条件数变差 / 误差增大 |
| 脏数据 | `--outlier-scale 80 --outlier-ratio 0.1` | OLS 退化；鲁棒估计更稳 |
| 长轨迹 | `--n-periods 10` | 估计更稳，耗时增加 |

---

## 2. 概念 ↔ 代码对照

| 概念 | 位置 |
|------|------|
| 数值 \(Y_g\)（RNEA） | `param_id/regressor.py` → `gravity_regressor_numeric` |
| JointTorqueRegressor + 摩擦 | `param_id/regressor.py` → `dynamics_regressor` |
| 列主元 QR 基参数 | `param_id/base_params.py` |
| OLS / Huber / 白化 WLS | `param_id/estimators.py` |
| 傅里叶 + 异常点 | `param_id/trajectory.py` + `scripts/identify_*.py` |
| URDF 加载 | `param_id/robot_model.py` |

伪惯量 SDP 约束（cvxpy）默认未启用。主路径：QR + robust WLS。

算法原理见 [`METHOD.md`](METHOD.md)。

---

## 3. 适用范围

- 力矩由 URDF 真值参数合成，再叠加噪声/异常点。
- 仿真 RMSE 不代表真机辨识精度。
- 详见 [`METHOD.md`](METHOD.md)。

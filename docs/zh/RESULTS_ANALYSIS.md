# 结果分析指南

对照仓库已生成图表，说明指标含义、读图要点与复现命令。

算法原理见 [`METHOD.md`](METHOD.md)；管线复现见 [`LEARNING.md`](LEARNING.md)；总览见根目录 [`README.md`](../../README.md)。

> **数据说明（当前 `results/comparison/`）**  
> 交叉对比默认尝试 Isaac Lab 实采；若仿真未启动，回退为  
> `Pinocchio + 摩擦/噪声/异常点` 代理数据（`--skip-isaac`）。  
> **算法与接口与 Isaac 路径相同**；说明结果时需标明数据来源。

相关图：

| 图 | 路径 |
|----|------|
| 分关节扭矩 RMSE | [`results/comparison/fig_rmse_per_joint.png`](../../results/comparison/fig_rmse_per_joint.png) |
| 基参数相对误差 | [`results/comparison/fig_rel_param.png`](../../results/comparison/fig_rel_param.png) |
| 关节 0 力矩拟合 | [`results/comparison/fig_torque_joint0.png`](../../results/comparison/fig_torque_joint0.png) |
| 重力补偿残差 | [`results/gravity_comp/fig_residual_bars.png`](../../results/gravity_comp/fig_residual_bars.png) |

---

## 1. Demo 入口

### 1.1 纯算法（不启 Isaac）

```bash
conda activate env_isaaclab   # 或 param-id
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
bash scripts/run_demo.sh
```

执行：回归自检 → 静力学/动力学辨识 → 碰撞检查。输出目录：`results/`。

### 1.2 Isaac 采集（需本机 Isaac Lab / GPU）

完整命令见：  
[根目录 README → Isaac Lab Demo](../../README.md#isaac-lab-demo带-gui--无头) · 英文 [`README_EN.md`](../../README_EN.md#isaac-lab-demo-gui--headless)

```bash
conda activate env_isaaclab
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# 可视化：去掉 --headless 可开 GUI
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --control-mode position_servo \
  --save-path results/baseline/isaac_demo_fourier.npz
```

工程化采集（PD + 摩擦 + 噪声，默认无头）：

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 2 \
  --control-mode pd_torque --kp 200 --kd 20 \
  --enable-friction --q-noise-std 1e-4 --tau-noise-std 0.05 \
  --headless --save-path results/baseline/isaac_eng_fourier.npz
```

采集后辨识：

```bash
python scripts/identify_dynamic.py --method robust_wls \
  --data-source file --data-path results/baseline/isaac_eng_fourier.npz \
  --results-dir results/isaac_dynamic
```

一键对照（含 Isaac；失败则代理）：

```bash
bash scripts/run_comparison_experiments.sh
# 或纯离线：
bash scripts/run_comparison_experiments.sh --skip-isaac
```

本仓库无独立 Isaac 官方场景包；演示路径为：教学 URDF → Lab 跟踪激励 → 统一 NPZ → 复用辨识脚本。

---

## 2. 三组实验的递进逻辑

| 顺序 | 组别 | 目的 |
|------|------|------|
| 1 | 基准组 | 理想数据误差 ≈ 0 → 验证 \(τ=Yπ\)、QR、求解链路 |
| 2 | 工程组 | 摩擦 + 噪声 + 异常点 → 观察 OLS 退化 |
| 3 | 鲁棒组 | 同一脏数据 + robust WLS → 压低内点误差；再接重力补偿闭环 |

数据层（Pinocchio 合成 / Isaac 采集）与算法层解耦；同一 NPZ 接口切换估计器。

---

## 3. 交叉对比图解读

配置见 `results/comparison/experiment_config.yaml`（轨迹、PD、摩擦、噪声统一）。

| 组别 | 数据 | 算法 |
|------|------|------|
| Baseline OLS | Pinocchio 理想 RNEA（无摩擦/噪声） | OLS |
| Eng. OLS | 工程数据（摩擦+噪声+稀疏异常） | OLS |
| Eng. robust WLS | 同上工程数据 | 白化 Huber-WLS |

### 3.1 分关节扭矩 RMSE（`fig_rmse_per_joint.png`）

纵轴：各关节拟合力矩 RMSE（N·m），含全部样本。

| 现象 | 含义 |
|------|------|
| 蓝柱（Baseline）≈ 0 | 理想工况下回归复现力矩 → 链路正确 |
| 橙/绿柱升高 | 引入摩擦、噪声、异常后，全样本 RMSE 约 1.2–4.3 N·m |
| 关节不均（如 j3、j4） | 与构型、力矩量级、摩擦激励有关 |
| 橙 ≈ 绿（全样本） | 鲁棒法降权/剔除异常；全样本 RMSE 未必更低，应看**内点 RMSE** |

本次 `summary.json`：

| | 全样本 RMSE | 内点 RMSE |
|--|------------:|----------:|
| Baseline | ~0 | ~0 |
| Eng. OLS | 2.70 | **0.24** |
| Eng. robust WLS | 2.71 | **0.05** |

优先对比内点：0.24 → 0.05（约 5×），即 robust 的工程意义。

### 3.2 基参数相对误差（`fig_rel_param.png`）

相对 URDF 投影到 QR 基参数空间的 \(\|\hatπ - π^{\mathrm{QR}}\|/\|π^{\mathrm{QR}}\|\)。

| 组别 | 相对误差 |
|------|----------|
| Baseline | **0.0%** |
| Eng. OLS | ~299% |
| Eng. robust WLS | ~308% |

解读要点：

- 工程力矩 = 惯性动力学 + **摩擦** + 噪声 + 异常；对照真值为「URDF 惯量 + **摩擦=0**」。
- 相对误差被摩擦/异常系统性抬高，**不能**单独作为失败判据。
- 工程指标优先：**力矩内点 RMSE、重力补偿残差、交叉验证**。
- Baseline = 0%：在匹配的数据生成模型下，基参数可唯一恢复。

### 3.3 关节 0 力矩曲线（`fig_torque_joint0.png`）

三行子图：灰线 meas、红线 pred。

| 子图 | 读法 |
|------|------|
| Baseline | 两线重合，RMSE=0 → 数学链路正确 |
| Eng.+OLS | 可见尖峰；红线跟趋势但受尖峰影响；全样本 RMSE≈2.4，内点≈0.24 |
| Eng.+robust WLS | 外形接近中图，内点≈0.05：异常被抑制，主趋势更干净 |

尖峰/丢包在真机采集中常见；robust 估计用于抑制其对拟合的主导作用。

---

## 4. 重力补偿（`fig_residual_bars.png`）

离线验证（Pinocchio RNEA 为真值；辨识结果来自理想静力学 NPZ）：

| 模式 | mean ‖残差‖ | max ‖残差‖ |
|------|------------:|-----------:|
| No GC | ~9.84 N·m | ~16.4 N·m |
| URDF GC | ~0 | ~0 |
| Identified GC | ~1e-14 | ~1e-14 |

- 无补偿时残差为重力矩量级。
- 理想静力学辨识的 \(\hatπ_g\) 与 URDF 一致时，补偿残差至数值噪声 → 辨识→力控前馈闭环在仿真上成立。
- Isaac：`python scripts/verify_gravity_compensation.py --headless ...`  
  关注 hold 时 PD 残余与拖动时末端受力是否平滑。

---

## 5. 复现命令

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# A. 算法 demo
bash scripts/run_demo.sh

# B. 再生成本文图表（离线代理）
bash scripts/run_comparison_experiments.sh --skip-isaac
cat results/comparison/conclusion.md

# C. 重力补偿
python scripts/verify_gravity_compensation.py --offline \
  --id-result results/baseline/id_pinocchio_static/static_ols.npz
cat results/gravity_comp/conclusion.md

# D. 有 GPU/Isaac 时：最短实采
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --ddq-mode ideal --headless \
  --save-path results/baseline/isaac_smoke.npz
```

---

## 6. 常见问题

**Q1：理想数据误差为 0，工程数据基参数误差却很大？**  
工程力矩含摩擦与异常，相对误差对照「无摩擦 URDF 基参数」。应同时报力矩内点误差与补偿残差。

**Q2：robust WLS 全样本 RMSE 并不更低，为何仍用？**  
全样本含被降权的异常点；看 **inlier RMSE**（本次约 0.24→0.05）及曲线是否被尖峰带偏。

**Q3：Isaac 与 Pinocchio 的分工？**  
Pinocchio：解析回归器与基参数；Isaac：PhysX 采集与力控验证。中间以统一 NPZ 解耦。

---

## 7. 目录速查

完整结构见根目录 [`README.md`](../../README.md) 与 [`results/README.md`](../../results/README.md)。

```text
results/comparison/     # 三组对照图 + conclusion.md + summary.json
results/gravity_comp/   # 重力补偿残差图
results/examples/       # 静/动力学示例图
results/baseline/       # 对齐/导出 NPZ、静力学 ID 结果等
scripts/run_demo.sh
scripts/collect_data_isaaclab.py
scripts/run_comparison_experiments.sh
scripts/verify_gravity_compensation.py
docs/README.md
```

# 结果分析指南

本文对照仓库已生成的图表，说明**怎么读结果、Isaac 演示怎么跑**。  
适合自学复盘。

算法原理见 [`METHOD.md`](METHOD.md)；实操步骤见 [`LEARNING.md`](LEARNING.md)；总览见 [`README.md`](README.md)。

> **数据说明（当前 `results/comparison/`）**  
> 交叉对比默认尝试 Isaac Lab 实采；若仿真未启动成功，会回退为  
> `Pinocchio + 摩擦/噪声/异常点` 代理数据（`--skip-isaac`）。  
> **算法与接口与 Isaac 路径相同**；若数据来自代理而非 Isaac，说明时需标明来源。

相关图：

| 图 | 路径 |
|----|------|
| 分关节扭矩 RMSE | [`results/comparison/fig_rmse_per_joint.png`](../../results/comparison/fig_rmse_per_joint.png) |
| 基参数相对误差 | [`results/comparison/fig_rel_param.png`](../../results/comparison/fig_rel_param.png) |
| 关节 0 力矩拟合 | [`results/comparison/fig_torque_joint0.png`](../../results/comparison/fig_torque_joint0.png) |
| 重力补偿残差 | [`results/gravity_comp/fig_residual_bars.png`](../../results/gravity_comp/fig_residual_bars.png) |

---

## 1. 项目里有没有 “Isaac Demo”？

**有演示入口，但分两层：**

### 1.1 纯算法 Demo（不启 Isaac，必跑）

```bash
conda activate env_isaaclab   # 或 param-id
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
bash scripts/run_demo.sh
```

会跑：回归自检 → 静力学/动力学辨识 → 碰撞检查。结果在 `results/`。

### 1.2 Isaac 采集 Demo（需要本机 Isaac Lab / GPU）

完整命令（**带 GUI / 无头 / 工程化 / 辨识**）见：  
[`docs/zh/README.md` → Isaac Lab Demo](README.md#isaac-lab-demo带-gui--无头) · 英文 [`README.md`](../../README.md#isaac-lab-demo-gui--headless)

```bash
conda activate env_isaaclab
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# 最短可视化：去掉 --headless 可开 GUI
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --control-mode position_servo \
  --save-path results/baseline/isaac_demo_fourier.npz
```

工程化一点（PD + 摩擦 + 噪声，默认无头）：

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 2 \
  --control-mode pd_torque --kp 200 --kd 20 \
  --enable-friction --q-noise-std 1e-4 --tau-noise-std 0.05 \
  --headless --save-path results/baseline/isaac_eng_fourier.npz
```

采集完辨识：

```bash
python scripts/identify_dynamic.py --method robust_wls \
  --data-source file --data-path results/baseline/isaac_eng_fourier.npz \
  --results-dir results/isaac_dynamic
```

一键对照（含 Isaac，失败自动代理）：

```bash
bash scripts/run_comparison_experiments.sh
# 或纯离线展示：
bash scripts/run_comparison_experiments.sh --skip-isaac
```

**没有单独的 “Isaac 官方示例场景包”**；本仓库的 demo 就是：  
用教学 URDF → Lab 跟踪激励轨迹 → 写出统一 NPZ → 复用原辨识脚本。

---

## 2. 三组实验怎么串起来看

建议阅读顺序：

1. **基准组** — 理想数据上误差 ≈ 0，证明 \(τ=Yπ\)、QR、求解链路正确。  
2. **工程组** — 摩擦 + 噪声 + 异常点后，普通 OLS 变差。  
3. **鲁棒组** — 同一份脏数据上，robust WLS **压低内点误差**；再接重力补偿闭环。

核心结构：

> 数据层（Pinocchio 合成 / Isaac 采集）与算法层解耦；同一 NPZ 接口切换估计器。

---

## 3. 交叉对比图怎么读

实验配置见 `results/comparison/experiment_config.yaml`（轨迹、PD、摩擦、噪声统一）。

三组定义：

| 组别 | 数据 | 算法 |
|------|------|------|
| Baseline OLS | Pinocchio 理想 RNEA（无摩擦/噪声） | OLS |
| Eng. OLS | 工程数据（摩擦+噪声+稀疏异常） | OLS |
| Eng. robust WLS | 同上工程数据 | 白化 Huber-WLS |

### 3.1 分关节扭矩 RMSE（`fig_rmse_per_joint.png`）

**纵轴**：各关节拟合力矩 RMSE（N·m），含全部样本。

**怎么读：**

- **蓝柱（Baseline）≈ 0**：理想工况下回归几乎完美复现力矩 → 链路正确。  
- **橙/绿柱明显升高**：引入摩擦、噪声、异常后，**全部样本 RMSE** 到约 1.2–4.3 N·m。  
- **关节不均**：例如 j3、j4 往往更大 —— 与构型、力矩量级、摩擦激励强弱有关，不是“程序坏了”。  
- **橙 ≈ 绿（全样本）**：鲁棒方法会**降权/剔除异常点**；全样本 RMSE 不一定更低，要看 **内点 RMSE**。

**对应数字（本次 summary）：**

| | 全样本 RMSE | 内点 RMSE |
|--|------------:|----------:|
| Baseline | ~0 | ~0 |
| Eng. OLS | 2.70 | **0.24** |
| Eng. robust WLS | 2.71 | **0.05** |

→ 读图时优先看：**内点 0.24 → 0.05（约 5×）**，这才是 robust 的意义。

### 3.2 基参数相对误差（`fig_rel_param.png`）

相对 URDF 投影到 QR 基参数空间的 \(\|\hatπ - π^{\mathrm{QR}}\|/\|π^{\mathrm{QR}}\|\)。

| 组别 | 相对误差 |
|------|----------|
| Baseline | **0.0%** |
| Eng. OLS | ~299% |
| Eng. robust WLS | ~308% |

**务必正确解读（容易讲错）：**

- 工程数据里力矩 = 惯性动力学 + **摩擦** + 噪声 + 异常；  
  对比真值却是「URDF 惯量 + **摩擦真值=0**」。  
- 因此相对误差会被摩擦/异常**系统性拉大**，**不能**说成“辨识完全失败”。  
- 更合理的工程指标是：**力矩预测内点误差、重力补偿残差、交叉验证**。  
- Baseline = 0% 用来证明：在匹配的数据生成模型下，基参数可唯一恢复。

### 3.3 关节 0 力矩曲线（`fig_torque_joint0.png`）

三行子图，灰线 meas、红线 pred。

1. **上：Baseline** — 两线重合，RMSE=0 → “数学链路 OK”。  
2. **中：Eng.+OLS** — 可见尖峰异常、曲线变“糙”；红线跟大趋势但吃尖峰；全样本 RMSE≈2.4，内点≈0.24。  
3. **下：Eng.+robust WLS** — 外形与中图接近，但 **inlier≈0.05**：异常被抑制，主趋势拟合更干净。

讲解时可指着尖峰说：真机采集常见尖峰/丢包，robust 的价值在此。

---

## 4. 重力补偿结果（`fig_residual_bars.png`）

离线验证（Pinocchio RNEA 为真值，辨识结果来自理想静力学 NPZ）：

| 模式 | mean ‖残差‖ | max ‖残差‖ |
|------|------------:|-----------:|
| No GC | ~9.84 N·m | ~16.4 N·m |
| URDF GC | ~0 | ~0 |
| Identified GC | ~1e-14 | ~1e-14 |

**含义：**

- 无补偿时残余就是重力矩量级。  
- 理想静力学辨识得到的 \(\hatπ_g\) 与 URDF 一致时，补偿残差到数值噪声 → **辨识→力控前馈**闭环在仿真上成立。  
- Isaac 版：`python scripts/verify_gravity_compensation.py --headless ...`  
  看 hold 时 PD 残余是否下降、拖动时末端受力是否平滑运动。

---

## 5. 推荐复现命令（学习用）

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# A. 算法 demo
bash scripts/run_demo.sh

# B. 再生成本文图表（离线代理，稳定可复现）
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

**Q1：为什么理想数据误差是 0，工程数据基参数误差却很大？**  
因为工程力矩含摩擦与异常，而相对误差对照的是「无摩擦 URDF 基参数」。应同时报力矩内点误差与补偿残差。

**Q2：robust WLS 全样本 RMSE 并不更低，为什么还要用？**  
全样本含被降权的异常点；看 **inlier RMSE**（本次约 0.24→0.05）和曲线是否被尖峰带偏。

**Q3：Isaac 和 Pinocchio 各干什么？**  
Pinocchio：解析回归器与基参数理论；Isaac：PhysX 采集与力控验证。中间用统一 NPZ 解耦。

---

## 7. 目录速查

完整结构见根目录 [`README.md`](../../README.md) 与 [`results/README.md`](../../results/README.md)。

```text
results/comparison/     # 三组对照图 + conclusion.md + summary.json
results/gravity_comp/   # 重力补偿残差图
results/examples/       # 早期静/动力学示例图
results/baseline/       # 对齐/导出 NPZ、静力学 ID 结果等
scripts/run_demo.sh
scripts/collect_data_isaaclab.py
scripts/run_comparison_experiments.sh
scripts/verify_gravity_compensation.py
docs/README.md          # 文档索引
```

# 动力学 / 静力学参数辨识 —— 自学复现指南

目标：用你自己的 `i7.urdf` 把实习 PDF 里的流程**亲手跑通**，能对着代码讲清
`τ = Yπ`、QR 基参数、Huber / 白化 WLS、傅里叶激励。这样简历条目才站得住。

参考文档：`~/txk/动力学_静力学参数辨识算法介绍及仿真结果.pdf`

---

## 0. 环境

```bash
conda activate param-id   # 或 env_isaaclab
cd <repo-root>
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

数据布局：

```
param_id/                 # Python 包
scripts/                  # CLI
models/demo_7dof/         # 公开教学 URDF
assets/proprietary/       # 本地私有 URDF/网格（gitignore）
docs/zh/                  # 中文文档
results/                  # 运行输出
```

---

## 1. 建议动手顺序（每步都要自己看输出）

### Step 0 — 回归矩阵是否正确（必做）

```bash
python scripts/00_sanity_check.py
```

应看到两个误差接近 `0`。含义：

| 检查 | 公式 | 对应 PDF |
|------|------|----------|
| 静力学 | `τ_g = Y_g(q) π_g`，`Y_g` 由 RNEA 数值差分构造 | 第 1 步 |
| 动力学 | `τ = Y(q,dq,ddq) π`，`Y` 来自 `computeJointTorqueRegressor` | 动力学第 1 步 |

若这里不过，后面辨识全是空中楼阁。

### Step 1 — 静力学辨识

```bash
python scripts/identify_static.py --method ols --outlier-ratio 0.05
python scripts/identify_static.py --method huber --outlier-ratio 0.05
python scripts/identify_static.py --method robust_wls --outlier-ratio 0.05
```

对照看：

1. QR：重力参数 28 → 基参数约 14（冗余被压掉）
2. **重力补偿误差**（脚本末尾 `gravity compensation check`）比「全样本 torque RMSE」更有意义——离群尖峰会抬高 all-RMSE
3. 本机一次跑数参考：OLS 重力补偿 ~0.6–0.8 N·m；robust_wls ~0.01 N·m
4. 打开 `results/static_*.png`：红点是故意注入的尖峰

### Step 2 — 动力学辨识

```bash
python scripts/identify_dynamic.py --method ols --n-periods 3
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05
```

对照：

1. 傅里叶轨迹：`q/dq/ddq` 解析一致（看 `results/dynamic_*.png` 第一行）
2. 满参数 → 基参数维数
3. hold-out RMSE（交叉验证味道）
4. 蓝/橙柱：真实基参数 vs 估计

### Step 3 — 激励轨迹碰撞回放（PDF 录屏对应）

文档里的视频不是 Isaac 动力学仿真，而是 **轨迹回放 + 碰撞检查**，用系统录屏保存。

```bash
# 先无窗口检查三种轨迹
python scripts/collision_view.py --traj fourier --check-only
python scripts/collision_view.py --traj cosine --check-only
python scripts/collision_view.py --traj cv --check-only

# 再打开 Trimesh 窗口（可自行录屏）
python scripts/collision_view.py --traj fourier --view
```

窗口操作：打开后应**自动连续播放**；`SPACE` 暂停/继续，`N`/`B` 单帧，方向键转相机，`ESC` 退出。  
标题栏 / 终端会显示 `PLAY` 或 `PAUSE`。  
若总是撞 `base_link`，先试 `--amplitude-scale 0.4`，或调试时加 `--ignore-base`。

### Step 4 — 自己改实验（加深印象）

| 实验 | 改什么 | 期望观察 |
|------|--------|----------|
| 无激励 | `--harmonics 0` 不适用；可把 `amplitude_scale` 改小 | 条件数变差、误差升 |
| 更脏数据 | `--outlier-scale 80 --outlier-ratio 0.1` | OLS 崩、鲁棒仍可用 |
| 更长轨迹 | `--n-periods 10` | 估计更稳、更慢 |
| 只重力 | 静力学里把摩擦列去掉（自己改一行） | 低速仍有符号摩擦残差 |

---

## 2. 代码 ↔ 简历 bullet 对照

| 简历说法 | 代码位置 |
|----------|----------|
| RNEA 数值差分得 \(Y_g\) | `regressor.py` → `gravity_regressor_numeric` |
| JointTorqueRegressor + 摩擦 | `regressor.py` → `dynamics_regressor` |
| 列主元 QR 最小参数集 | `base_params.py` → `select_base_columns` |
| OLS / Huber-IRLS / 白化 WLS | `estimators.py` |
| 傅里叶激励 + 噪声/离群点 | `trajectory.py` + `identify_*.py` 合成数据 |
| 左臂 URDF 流程复核 | `robot_model.py` → `build_left_arm_model` |

SDP 伪惯量约束（cvxpy）在本复现中**未默认启用**（环境无 cvxpy）。面试可说：
「仿真主路径是 QR + 鲁棒 WLS；物理可行 SDP 是可选后处理，我复现时先保证主链路。」
若要加，可再装 `cvxpy` + CLARABEL。

---

## 3. 面试时按这条因果链讲（90 秒）

1. **要什么**：力矩前馈 / 重力补偿 → 需要 \(π\)（惯量+摩擦），不是只要控制器增益。
2. **线性化**：刚体力矩对惯性参数仿射 → \(τ=Yπ\)；静力学只要重力+库仑。
3. **为什么 QR**：杆件参数线性相关（基座、平行轴等），满参数不可唯一辨识。
4. **为什么傅里叶**：解析一致的 \(q,\dot q,\ddot q\)，能量集中在可执行低频，条件数更好。
5. **为什么鲁棒**：力矩尖峰/碰撞采样会毁掉 OLS；Huber 降权，白化处理关节异方差，硬阈值剔点。
6. **怎么用**：辨识 \(π\) → 实时 \(τ_{ff}=Y(q,\dot q,\ddot q)\hatπ\) 或静力学重力补偿。

---

## 4. 诚实边界（简历表述）

- 本仓库是**对照 PDF 的仿真复现**，用 URDF 真值参数合成力矩，再加噪声/离群点反辨识。
- 实习原工作若含真机采数、碰撞检查、带教代码，面试时区分：「算法链路我按文档复现并吃透；真机部分按实际参与度说。」
- 不要把「仿真 RMSE」说成「真机辨识精度」。

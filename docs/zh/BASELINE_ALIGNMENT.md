# 基准对齐（理想物理）

理想工况：仅重力（动力学再加惯量）；Isaac 上关节库仑/粘性摩擦与 armature 关闭（`--ideal-physics`）。  
两侧使用相同激励种子与参数。辨识用 OLS。

英文原文：[`../BASELINE_ALIGNMENT.md`](../BASELINE_ALIGNMENT.md)

## 通过门槛

| 实验 | 指标 | 阈值 |
|------|------|-----:|
| 静力学（cosine） | 相对 URDF QR 投影的重力**基参数**相对误差 | **< 5%** |
| 动力学（fourier） | 相对 URDF QR 投影的全基参数相对误差 | **< 5%** |

同时报告力矩拟合 RMSE 与交叉差 `|π̂^P - π̂^I|`。

## A) 静力学 — cosine

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# 1) Pinocchio 理想数据（RNEA，无摩擦）
python scripts/export_pinocchio_dataset.py --mode static --traj cosine \
  --n-periods 2 --dt 0.01 --seed 0 --fundamental-freq 0.1 \
  --save-path results/baseline/pinocchio_static_cosine.npz

# 2) Isaac 理想采集
python scripts/collect_data_isaaclab.py --mode static --traj cosine \
  --n-periods 2 --dt 0.01 --seed 0 --fundamental-freq 0.1 \
  --ddq-mode ideal --ideal-physics --headless \
  --save-path results/baseline/isaac_static_cosine_ideal.npz

# 3) 辨识（同一方法）
python scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/pinocchio_static_cosine.npz \
  --results-dir results/baseline/id_pinocchio_static
python scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/isaac_static_cosine_ideal.npz \
  --results-dir results/baseline/id_isaac_static

# 4) 对比 + 柱状图 + PASS/FAIL
python scripts/compare_baseline_alignment.py --kind static \
  --pinocchio-data results/baseline/pinocchio_static_cosine.npz \
  --isaac-data results/baseline/isaac_static_cosine_ideal.npz \
  --out-dir results/baseline/compare_static
```

## B) 动力学 — fourier

```bash
python scripts/export_pinocchio_dataset.py --mode dynamic --traj fourier \
  --n-periods 2 --fourier-harmonics 5 --dt 0.01 --seed 0 \
  --save-path results/baseline/pinocchio_dynamic_fourier.npz

python scripts/collect_data_isaaclab.py --mode dynamic --traj fourier \
  --n-periods 2 --fourier-harmonics 5 --dt 0.01 --seed 0 \
  --ddq-mode ideal --ideal-physics --headless \
  --save-path results/baseline/isaac_dynamic_fourier_ideal.npz

python scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/pinocchio_dynamic_fourier.npz \
  --results-dir results/baseline/id_pinocchio_dynamic
python scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/isaac_dynamic_fourier_ideal.npz \
  --results-dir results/baseline/id_isaac_dynamic

python scripts/compare_baseline_alignment.py --kind dynamic \
  --pinocchio-data results/baseline/pinocchio_dynamic_fourier.npz \
  --isaac-data results/baseline/isaac_dynamic_fourier_ideal.npz \
  --out-dir results/baseline/compare_dynamic
```

一键：`bash scripts/run_baseline_alignment.sh`
